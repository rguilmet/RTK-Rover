#include "rover_sensors.h"
#include "../../user_config.h"
#include <Field_Instrument_Core.h>
#include "../settings_manager/settings_manager.h"
#include "../nav_engine/nav_engine.h"
#include "../rover_audio/rover_audio.h"
#include "../storage/serial_logger.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>

#define MAG_CAL_MAX_SAMPLES 300

typedef struct {
    float          *pts_buf;
    float           min_xy[2];
    float           max_xy[2];
    float           last_pt[3];
    float           rms_pct;
    uint32_t        start_ms;
    uint32_t        result_ms;
    uint16_t        samples;
    uint16_t        sector_mask;
    uint8_t         progress_pct;
    MagCalStatus_t  status;
} MagCalWizard_t;

static volatile bool     pending_tare_level     = false;
static volatile bool     pending_mag_cal_toggle = false;
static volatile bool     pending_mag_cal_cancel = false;
static Ahrs9DofState_t  *s_beam_ahrs            = NULL;
static FicSen0696_t      s_beam_ctx             = {};
static sen0696_state_t   s_boot_10dof           = {};
static portMUX_TYPE      s_imu_snapshot_mux     = portMUX_INITIALIZER_UNLOCKED;
static float             s_snapshot_bubble_x    = 0.0f;
static float             s_snapshot_bubble_y    = 0.0f;
static float             s_snapshot_pole_tilt   = 0.0f;

static FicMmc5983maData_t s_cached_mag           = {};
static sen0696_state_t   s_beam_state           = {};
static Ahrs9DofOutput_t  s_beam_ahrs_out        = {};
static bool              s_beam_ahrs_has_sample = false;
static bool              s_beam_10dof_active    = false;
static uint32_t          s_last_imu_poll_ms     = 0;
static uint32_t          s_last_imu_log_ms      = 0;
static uint32_t          s_last_10dof_poll_ms   = 0;
static uint32_t          s_last_10dof_ahrs_ms   = 0;
static uint32_t          s_last_10dof_log_ms    = 0;
static uint32_t          s_beam_poll_ticks      = 0;
static float             s_cached_imu_pitch     = 0.0f;
static float             s_cached_imu_roll      = 0.0f;
static float             s_cached_bubble_x      = 0.0f;
static float             s_cached_bubble_y      = 0.0f;
static DisplayPose_t     s_cached_pose          = DISPLAY_POSE_FLAT;
static float             s_cached_acc[3]        = {0.0f, 0.0f, 0.0f};
static float             s_cached_gyr[3]        = {0.0f, 0.0f, 0.0f};
static float             s_prev_imu_acc[3]      = {0.0f, 0.0f, 0.0f};
static bool              s_prev_imu_valid       = false;
static uint32_t          s_last_imu_motion_ms   = 0;
static float             s_prev_10dof_acc[3]    = {0.0f, 0.0f, 0.0f};
static bool              s_prev_10dof_valid     = false;
static float             s_baro_elev_bias_m     = 0.0f;
static bool              s_baro_bias_valid      = false;
static MagCalWizard_t    s_mag_cal_wiz          = {};

extern "C" void trigger_tare_pole_level(void)
{
    pending_tare_level = true;
}

extern "C" void trigger_mag_cal_start_or_finish(void)
{
    pending_mag_cal_toggle = true;
}

extern "C" void trigger_mag_cal_cancel(void)
{
    pending_mag_cal_cancel = true;
}

// =========================================================================
// Core 1 Helper: 360-Deg Magnetometer Ellipsoid Calibration Collector/Solver
// =========================================================================
static void mag_cal_run_fit(MagCalWizard_t *w, uint32_t now_ms)
{
    if (!w || !w->pts_buf || w->samples < 40U) {
        w->status    = MAG_CAL_FAILED;
        w->result_ms = now_ms;
        play_rover_beep(400, 180);
        Serial.printf("[CAL MAG] Failed: only %u samples collected (need >= 40)\n", w ? (unsigned)w->samples : 0U);
        return;
    }

    const float *xs = w->pts_buf;
    const float *ys = w->pts_buf + MAG_CAL_MAX_SAMPLES;
    const float *zs = w->pts_buf + (MAG_CAL_MAX_SAMPLES * 2U);

    float center[3] = {0.0f, 0.0f, 0.0f};
    float soft[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    float rms_pct = 0.0f;

    if (!fit_ellipsoid_3d_f32(xs, ys, zs, (int)w->samples, center, soft, &rms_pct)) {
        w->status    = MAG_CAL_FAILED;
        w->result_ms = now_ms;
        w->rms_pct   = 100.0f;
        play_rover_beep(400, 180);
        Serial.printf("[CAL MAG] Ellipsoid fit failed (%u pts)\n", (unsigned)w->samples);
        return;
    }

    double sum_r = 0.0;
    for (uint16_t i = 0; i < w->samples; i++) {
        float dx = xs[i] - center[0];
        float dy = ys[i] - center[1];
        float dz = zs[i] - center[2];
        float cx = soft[0][0] * dx + soft[0][1] * dy + soft[0][2] * dz;
        float cy = soft[1][0] * dx + soft[1][1] * dy + soft[1][2] * dz;
        float cz = soft[2][0] * dx + soft[2][1] * dy + soft[2][2] * dz;
        sum_r += sqrtf(cx * cx + cy * cy + cz * cz);
    }
    float mean_r = (float)(sum_r / (double)w->samples);
    if (!isfinite(mean_r) || mean_r < 20.0f || mean_r > 85.0f) {
        w->status    = MAG_CAL_FAILED;
        w->result_ms = now_ms;
        w->rms_pct   = 100.0f;
        play_rover_beep(400, 180);
        Serial.printf("[CAL MAG] Rejected implausible solved |B|=%.1fuT (%u pts)\n",
                      mean_r, (unsigned)w->samples);
        return;
    }

    memcpy(current_settings.mag_hard_iron_uT, center, sizeof(center));
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            current_settings.mag_soft_iron[i * 3 + j] = soft[i][j];
        }
    }
    current_settings.mag_ellipsoid_cal_valid = true;
    current_settings.mag_field_ref_uT        = mean_r;
    if (s_beam_ahrs) {
        ahrs_9dof_set_mag_cal(s_beam_ahrs,
                              current_settings.mag_hard_iron_uT,
                              current_settings.mag_soft_iron,
                              true);
    }
    mark_settings_dirty();

    w->rms_pct      = rms_pct;
    w->progress_pct = 100U;
    w->status       = MAG_CAL_SUCCESS;
    w->result_ms    = now_ms;
    play_rover_dual_beep(1400, 60, 30, 2200, 100);
    Serial.printf("[CAL MAG] OK [%s] (%u pts): Center=(%+.1f,%+.1f,%+.1f)uT | |B|=%.1fuT | RMS=%.2f%%\n",
                  (fabsf(center[2]) > 0.001f) ? "3D" : "2D-XY",
                  (unsigned)w->samples, center[0], center[1], center[2], mean_r, rms_pct);
}

static uint8_t popcount_u16(uint16_t v)
{
    uint8_t c = 0;
    while (v) {
        c += (uint8_t)(v & 1U);
        v >>= 1U;
    }
    return c;
}

static void mag_cal_update_wizard(MagCalWizard_t *w, const float raw_mag[3], bool mag_sample_ok, uint32_t now_ms)
{
    if (!w) return;

    if (w->pts_buf == NULL) {
        w->pts_buf = (float *)heap_caps_malloc(MAG_CAL_MAX_SAMPLES * 3U * sizeof(float),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (w->pts_buf == NULL) {
            w->pts_buf = (float *)malloc(MAG_CAL_MAX_SAMPLES * 3U * sizeof(float));
        }
    }

    if (pending_mag_cal_cancel) {
        pending_mag_cal_cancel = false;
        w->status       = MAG_CAL_IDLE;
        w->samples      = 0;
        w->sector_mask  = 0;
        w->progress_pct = 0;
        play_rover_beep(700, 80);
        return;
    }

    if (pending_mag_cal_toggle) {
        pending_mag_cal_toggle = false;
        if (w->status == MAG_CAL_COLLECTING) {
            mag_cal_run_fit(w, now_ms);
            return;
        }
        if (w->pts_buf == NULL) {
            w->status    = MAG_CAL_FAILED;
            w->result_ms = now_ms;
            play_rover_beep(400, 180);
            return;
        }
        w->status       = MAG_CAL_COLLECTING;
        w->samples      = 0;
        w->sector_mask  = 0;
        w->progress_pct = 0;
        w->rms_pct      = 0.0f;
        w->start_ms     = now_ms;
        play_rover_beep(1600, 60);
        Serial.println("[CAL MAG] Started 360-deg ellipsoid capture (rotate + gently tilt pole)...");
    }

    if (w->status == MAG_CAL_SUCCESS || w->status == MAG_CAL_FAILED) {
        if ((now_ms - w->result_ms) >= 5000U) {
            w->status = MAG_CAL_IDLE;
        }
        return;
    }

    if (w->status != MAG_CAL_COLLECTING) {
        return;
    }

    if (mag_sample_ok && raw_mag && w->pts_buf && w->samples < MAG_CAL_MAX_SAMPLES) {
        float *xs = w->pts_buf;
        float *ys = w->pts_buf + MAG_CAL_MAX_SAMPLES;
        float *zs = w->pts_buf + (MAG_CAL_MAX_SAMPLES * 2U);

        bool accept = (w->samples == 0U);
        if (!accept) {
            float dx = raw_mag[0] - w->last_pt[0];
            float dy = raw_mag[1] - w->last_pt[1];
            float dz = raw_mag[2] - w->last_pt[2];
            accept = ((dx * dx + dy * dy + dz * dz) >= 4.0f);
        }

        if (accept) {
            uint16_t idx = w->samples;
            xs[idx] = raw_mag[0];
            ys[idx] = raw_mag[1];
            zs[idx] = raw_mag[2];
            w->last_pt[0] = raw_mag[0];
            w->last_pt[1] = raw_mag[1];
            w->last_pt[2] = raw_mag[2];
            w->samples++;

            if (idx == 0U) {
                w->min_xy[0] = w->max_xy[0] = raw_mag[0];
                w->min_xy[1] = w->max_xy[1] = raw_mag[1];
            } else {
                if (raw_mag[0] < w->min_xy[0]) w->min_xy[0] = raw_mag[0];
                if (raw_mag[0] > w->max_xy[0]) w->max_xy[0] = raw_mag[0];
                if (raw_mag[1] < w->min_xy[1]) w->min_xy[1] = raw_mag[1];
                if (raw_mag[1] > w->max_xy[1]) w->max_xy[1] = raw_mag[1];
            }

            if ((w->max_xy[0] - w->min_xy[0]) > 10.0f && (w->max_xy[1] - w->min_xy[1]) > 10.0f) {
                float cx = 0.5f * (w->min_xy[0] + w->max_xy[0]);
                float cy = 0.5f * (w->min_xy[1] + w->max_xy[1]);
                float ang = atan2f(raw_mag[1] - cy, raw_mag[0] - cx);
                int sec = (int)floorf(((ang + (float)M_PI) / (2.0f * (float)M_PI)) * 12.0f);
                if (sec < 0) sec = 0;
                if (sec >= 12) sec = 11;
                w->sector_mask |= (uint16_t)(1U << sec);
            }
        }
    }

    uint8_t sec_cnt = popcount_u16(w->sector_mask);
    uint32_t pt_pct  = (w->samples >= 120U) ? 30U : (((uint32_t)w->samples * 30U) / 120U);
    uint32_t sec_pct = ((uint32_t)sec_cnt * 70U) / 12U;
    uint32_t pct = pt_pct + sec_pct;
    if (pct > 99U) pct = 99U;
    w->progress_pct = (uint8_t)pct;

    if ((w->samples >= 120U && sec_cnt >= 10U) ||
        (w->samples >= (MAG_CAL_MAX_SAMPLES - 20U)) ||
        ((now_ms - w->start_ms) >= 30000U)) {
        mag_cal_run_fit(w, now_ms);
    }
}

/**
 * @brief Initializes DFRobot SEN0696 (10DOF IMU) preferring I2C1 (IO47/48), falling back to I2C0.
 */
static void run_beam_10dof_boot_check(sen0696_state_t *out_report)
{
    sen0696_state_t report;
    (void)fic_sen0696_init(&s_beam_ctx, ROVER_I2C_ADDR_BEAM_IMU, ROVER_I2C_ADDR_BEAM_IMU_MAX, &report);
    if (out_report) {
        *out_report = report;
    }

    if (report.i2c_addr == 0U) {
        Serial.printf("[10DOF] DFRobot SEN0696 (0x%02X..0x%02X) not detected on I2C1 or I2C0 -- skipping.\n",
                      (unsigned)ROVER_I2C_ADDR_BEAM_IMU,
                      (unsigned)ROVER_I2C_ADDR_BEAM_IMU_MAX);
        return;
    }

    if (!report.id_valid) {
        Serial.printf("[10DOF] Device at 0x%02X (I2C%u) failed SEN0696 ID check (VID=0x%04X PID=0x%04X FW=0x%04X).\n",
                      (unsigned)report.i2c_addr,
                      (unsigned)report.i2c_bus,
                      (unsigned)report.vid,
                      (unsigned)report.pid,
                      (unsigned)report.fw_version);
        return;
    }

    Serial.printf("[10DOF] DFRobot SEN0696 found @ 0x%02X on %s | VID=0x%04X%s PID=0x%04X FW=0x%04X | Cfg:%s\n",
                  (unsigned)report.i2c_addr,
                  (report.i2c_bus == 1U) ? "I2C1 (IO47/48 Isolated)" : "I2C0 (IO7/8 Fallback)",
                  (unsigned)report.vid,
                  (report.vid == SEN0696_EXPECTED_VID) ? "" : "(unexpected)",
                  (unsigned)report.pid,
                  (unsigned)report.fw_version,
                  report.configured ? "OK" : "ERR");

    if (report.accel_gyro_valid) {
        float acc_norm = sqrtf((report.accel_g[0] * report.accel_g[0]) +
                               (report.accel_g[1] * report.accel_g[1]) +
                               (report.accel_g[2] * report.accel_g[2]));
        Serial.printf("[10DOF] Accel: X=%.3fg Y=%.3fg Z=%.3fg (|a|=%.3fg) | Gyro: X=%.2fdps Y=%.2fdps Z=%.2fdps\n",
                      report.accel_g[0], report.accel_g[1], report.accel_g[2], acc_norm,
                      report.gyro_dps[0], report.gyro_dps[1], report.gyro_dps[2]);
    } else {
        Serial.println("[10DOF] Accel/Gyro: READ_ERR");
    }

    if (report.mag_valid) {
        Serial.printf("[10DOF] Mag:   X=%.2fuT Y=%.2fuT Z=%.2fuT (|B|=%.2fuT)\n",
                      report.mag_ut[0], report.mag_ut[1], report.mag_ut[2], report.mag_norm_ut);
    } else {
        Serial.println("[10DOF] Mag: READ_ERR");
    }

    if (report.press_valid) {
        Serial.printf("[10DOF] Press: %.2f hPa (%.0f Pa, BaroAlt=%.1f m) | Temp: N/A (not in SEN0696 I2C bridge reg map)\n",
                      report.pressure_hpa, report.pressure_pa, report.altitude_m);
    } else {
        Serial.println("[10DOF] Press: READ_ERR | Temp: N/A (not in SEN0696 I2C bridge reg map)");
    }
}

extern "C" void rover_sensors_hw_init(void)
{
    fic_qmi8658_set_log_callback(rover_printf);
    fic_qmi8658_init();
    fic_mmc5983ma_set_log_callback(rover_printf);
    (void)fic_mmc5983ma_init_default();
    fic_sen0696_set_log_callback(rover_printf);
    run_beam_10dof_boot_check(&s_boot_10dof);
}

extern "C" void rover_sensors_ahrs_init(void)
{
    s_beam_ahrs = (Ahrs9DofState_t *)heap_caps_malloc(sizeof(Ahrs9DofState_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_beam_ahrs == NULL) {
        s_beam_ahrs = (Ahrs9DofState_t *)malloc(sizeof(Ahrs9DofState_t));
    }
    if (s_beam_ahrs != NULL) {
        Ahrs9DofConfig_t ahrs_cfg;
        ahrs_9dof_default_config(&ahrs_cfg, current_settings.beam_imu_orient_mode);
        ahrs_9dof_init(s_beam_ahrs, &ahrs_cfg);
        if (current_settings.mag_ellipsoid_cal_valid &&
            s_boot_10dof.mag_valid &&
            s_boot_10dof.mag_norm_ut >= 25.0f &&
            s_boot_10dof.mag_norm_ut <= 75.0f) {
            float mb[3];
            ahrs_9dof_transform_vec(s_beam_ahrs->cfg.orient_matrix, s_boot_10dof.mag_ut, mb);
            float dx = mb[0] - current_settings.mag_hard_iron_uT[0];
            float dy = mb[1] - current_settings.mag_hard_iron_uT[1];
            float dz = mb[2] - current_settings.mag_hard_iron_uT[2];
            float cx = current_settings.mag_soft_iron[0] * dx +
                       current_settings.mag_soft_iron[1] * dy +
                       current_settings.mag_soft_iron[2] * dz;
            float cy = current_settings.mag_soft_iron[3] * dx +
                       current_settings.mag_soft_iron[4] * dy +
                       current_settings.mag_soft_iron[5] * dz;
            float cz = current_settings.mag_soft_iron[6] * dx +
                       current_settings.mag_soft_iron[7] * dy +
                       current_settings.mag_soft_iron[8] * dz;
            float cal_norm = sqrtf(cx * cx + cy * cy + cz * cz);
            if (!isfinite(cal_norm) || cal_norm < 22.0f || cal_norm > 80.0f) {
                Serial.printf("[AHRS WARN] Stored mag cal collapsed |B| (%.1f -> %.1f uT) -- cleared stale cal; run CAL MAG\n",
                              s_boot_10dof.mag_norm_ut, cal_norm);
                memset(current_settings.mag_hard_iron_uT, 0, sizeof(current_settings.mag_hard_iron_uT));
                memset(current_settings.mag_soft_iron, 0, sizeof(current_settings.mag_soft_iron));
                current_settings.mag_soft_iron[0] = 1.0f;
                current_settings.mag_soft_iron[4] = 1.0f;
                current_settings.mag_soft_iron[8] = 1.0f;
                current_settings.mag_ellipsoid_cal_valid = false;
                if (current_settings.mag_field_ref_uT < 25.0f || current_settings.mag_field_ref_uT > 75.0f) {
                    current_settings.mag_field_ref_uT = s_boot_10dof.mag_norm_ut;
                }
                mark_settings_dirty();
            }
        }
        ahrs_9dof_set_mag_cal(s_beam_ahrs,
                              current_settings.mag_hard_iron_uT,
                              current_settings.mag_soft_iron,
                              current_settings.mag_ellipsoid_cal_valid);
        Serial.printf("[AHRS] Beam 9-DOF filter ready in PSRAM | OrientMode:%u | EllipsoidCal:%s\n",
                      (unsigned)current_settings.beam_imu_orient_mode,
                      current_settings.mag_ellipsoid_cal_valid ? "YES" : "NO");
    }
}

extern "C" void rover_sensors_poll_core1(uint32_t now_ms)
{
    // Hardware bus thread-safety across tasks is guaranteed by i2c_bus_mutex in i2c_bsp.
    // No dependency on the LVGL graphics mutex so sensor polling is never starved.
    const bool beam_online_fast = fic_sen0696_is_present(&s_beam_ctx) && s_beam_state.valid &&
                                  s_beam_ahrs_has_sample && (fic_sen0696_get_consec_fails(&s_beam_ctx) < 15U);
    const uint32_t imu_poll_period_ms = beam_online_fast ? 2500U : 100U;
    if (now_ms - s_last_imu_poll_ms >= imu_poll_period_ms || s_last_imu_poll_ms == 0) {
        s_last_imu_poll_ms = (now_ms == 0) ? 1 : now_ms;
        int16_t imu_temp = 0;
        fic_qmi8658_set_debug_logging(current_settings.enable_imu_logging);
        fic_qmi8658_read(s_cached_acc, s_cached_gyr, &imu_temp);
        FicDisplayPose_t fic_pose = FIC_DISPLAY_POSE_FLAT;
        fic_qmi8658_compute_pose_and_bubble(s_cached_acc,
                                            current_settings.pole_tilt_tare_pitch_deg,
                                            current_settings.pole_tilt_tare_roll_deg,
                                            &s_cached_imu_pitch,
                                            &s_cached_imu_roll,
                                            &s_cached_bubble_x,
                                            &s_cached_bubble_y,
                                            &fic_pose);
        s_cached_pose = (DisplayPose_t)fic_pose;

        const bool beam_online_now = fic_sen0696_is_present(&s_beam_ctx) && s_beam_state.valid &&
                                     s_beam_ahrs_has_sample && (fic_sen0696_get_consec_fails(&s_beam_ctx) < 15U);
        if (!beam_online_now) {
            float imu_pole_tilt = sqrtf(s_cached_bubble_x * s_cached_bubble_x +
                                        s_cached_bubble_y * s_cached_bubble_y);
            portENTER_CRITICAL(&s_imu_snapshot_mux);
            s_snapshot_bubble_x  = s_cached_bubble_x;
            s_snapshot_bubble_y  = s_cached_bubble_y;
            s_snapshot_pole_tilt = imu_pole_tilt;
            portEXIT_CRITICAL(&s_imu_snapshot_mux);

            // Poll outboard MMC5983MA 3-Axis 18-bit AMR Compass as Tier-B fallback when SEN0696 is absent
            (void)fic_mmc5983ma_poll(s_cached_bubble_y, s_cached_bubble_x, &s_cached_mag);
        }

        float gyr_mag = sqrtf(s_cached_gyr[0] * s_cached_gyr[0] +
                              s_cached_gyr[1] * s_cached_gyr[1] +
                              s_cached_gyr[2] * s_cached_gyr[2]);
        float acc_delta = s_prev_imu_valid
            ? (fabsf(s_cached_acc[0] - s_prev_imu_acc[0]) +
               fabsf(s_cached_acc[1] - s_prev_imu_acc[1]) +
               fabsf(s_cached_acc[2] - s_prev_imu_acc[2]))
            : 0.0f;
        if (gyr_mag >= 8.0f || acc_delta >= 0.08f) {
            s_last_imu_motion_ms = (now_ms == 0) ? 1 : now_ms;
        }
        s_prev_imu_acc[0] = s_cached_acc[0];
        s_prev_imu_acc[1] = s_cached_acc[1];
        s_prev_imu_acc[2] = s_cached_acc[2];
        s_prev_imu_valid  = true;

        if (current_settings.enable_imu_logging && (now_ms - s_last_imu_log_ms >= 1000)) {
            s_last_imu_log_ms = now_ms;
            Serial.printf("[IMU POSE] Pitch: %+.1f deg | Roll: %+.1f deg | Acc(g): %+.2f %+.2f %+.2f | Gyr(dps): %+.1f %+.1f %+.1f | Pose: %d\n",
                          s_cached_imu_pitch, s_cached_imu_roll,
                          s_cached_acc[0], s_cached_acc[1], s_cached_acc[2],
                          s_cached_gyr[0], s_cached_gyr[1], s_cached_gyr[2],
                          (int)s_cached_pose);
        }
    }

    // High-rate SEN0696 10DOF polling: 25-30 Hz on isolated I2C1 (IO47/48), or staggered ~5 Hz fallback on I2C0
    static uint8_t s_10dof_step     = 0;
    static bool    s_10dof_in_cycle = false;
    if (fic_sen0696_is_present(&s_beam_ctx)) {
        bool cycle_done = false;
        bool ok = false;
        if (fic_sen0696_get_i2c_bus(&s_beam_ctx) == 1U) {
            if (fic_sen0696_get_consec_fails(&s_beam_ctx) < 15U || (now_ms - s_last_10dof_poll_ms >= 5000U)) {
                s_last_10dof_poll_ms = (now_ms == 0) ? 1U : now_ms;
                s_beam_poll_ticks++;
                const bool read_baro = ((s_beam_poll_ticks % 10U) == 0U);
                ok = fic_sen0696_poll_fast(&s_beam_ctx, read_baro, &s_beam_state);
                cycle_done = true;
            }
        } else {
            const uint32_t period_ms = (fic_sen0696_get_consec_fails(&s_beam_ctx) >= 15U) ? 5000U : 200U;
            if (!s_10dof_in_cycle && (now_ms - s_last_10dof_poll_ms >= period_ms)) {
                s_last_10dof_poll_ms = (now_ms == 0) ? 1U : now_ms;
                s_10dof_in_cycle     = true;
                s_10dof_step         = 0;
            }
            if (s_10dof_in_cycle) {
                ok = fic_sen0696_poll_step(&s_beam_ctx, s_10dof_step, &s_beam_state);
                s_10dof_step++;
                if (s_10dof_step >= 3U) {
                    s_10dof_in_cycle = false;
                    s_10dof_step     = 0;
                    cycle_done       = true;
                }
            }
        }

        if (cycle_done) {
            if (ok && s_beam_state.accel_gyro_valid) {
                float dt_ahrs = (s_last_10dof_ahrs_ms > 0U && now_ms >= s_last_10dof_ahrs_ms)
                                ? ((float)(now_ms - s_last_10dof_ahrs_ms) / 1000.0f)
                                : 0.035f;
                if (dt_ahrs < 0.002f) dt_ahrs = 0.002f;
                if (dt_ahrs > 0.25f)  dt_ahrs = 0.25f;
                s_last_10dof_ahrs_ms = (now_ms == 0) ? 1U : now_ms;

                if (s_beam_ahrs) {
                    ahrs_9dof_update(s_beam_ahrs,
                                     s_beam_state.accel_g,
                                     s_beam_state.gyro_dps,
                                     s_beam_state.mag_ut,
                                     s_beam_state.mag_valid,
                                     false,
                                     dt_ahrs,
                                     &s_beam_ahrs_out);
                    s_beam_ahrs_has_sample = true;

                    // Negate positive-up beam pitch for screen bubble Y so tipping up moves bubble UP (matching QMI8658)
                    float bx = s_beam_ahrs_out.roll_deg   - current_settings.pole_tilt_tare_roll_deg;
                    float by = -s_beam_ahrs_out.pitch_deg - current_settings.pole_tilt_tare_pitch_deg;
                    float tilt = sqrtf(bx * bx + by * by);
                    portENTER_CRITICAL(&s_imu_snapshot_mux);
                    s_snapshot_bubble_x  = bx;
                    s_snapshot_bubble_y  = by;
                    s_snapshot_pole_tilt = tilt;
                    portEXIT_CRITICAL(&s_imu_snapshot_mux);
                }

                float beam_gyr_mag = sqrtf(s_beam_state.gyro_dps[0] * s_beam_state.gyro_dps[0] +
                                           s_beam_state.gyro_dps[1] * s_beam_state.gyro_dps[1] +
                                           s_beam_state.gyro_dps[2] * s_beam_state.gyro_dps[2]);
                float beam_acc_delta = s_prev_10dof_valid
                    ? (fabsf(s_beam_state.accel_g[0] - s_prev_10dof_acc[0]) +
                       fabsf(s_beam_state.accel_g[1] - s_prev_10dof_acc[1]) +
                       fabsf(s_beam_state.accel_g[2] - s_prev_10dof_acc[2]))
                    : 0.0f;
                if (beam_gyr_mag >= 8.0f || beam_acc_delta >= 0.08f) {
                    s_last_imu_motion_ms = (now_ms == 0) ? 1U : now_ms;
                }
                s_prev_10dof_acc[0] = s_beam_state.accel_g[0];
                s_prev_10dof_acc[1] = s_beam_state.accel_g[1];
                s_prev_10dof_acc[2] = s_beam_state.accel_g[2];
                s_prev_10dof_valid  = true;
            } else if (!ok && ((now_ms - s_last_10dof_log_ms) >= 2000U)) {
                s_last_10dof_log_ms = now_ms;
                Serial.printf("[10DOF I2C%u] READ ERR (acc:%s mag:%s press:%s fails:%u)\n",
                              (unsigned)fic_sen0696_get_i2c_bus(&s_beam_ctx),
                              s_beam_state.accel_gyro_valid ? "OK" : "ERR",
                              s_beam_state.mag_valid        ? "OK" : "ERR",
                              s_beam_state.press_valid      ? "OK" : "ERR",
                              (unsigned)s_beam_state.consec_fails);
            }
        }
    }

    s_beam_10dof_active = fic_sen0696_is_present(&s_beam_ctx) && s_beam_state.present && s_beam_state.valid &&
                          s_beam_ahrs_has_sample && (fic_sen0696_get_consec_fails(&s_beam_ctx) < 15U);

    if (pending_tare_level) {
        pending_tare_level = false;
        if (s_beam_10dof_active) {
            current_settings.pole_tilt_tare_pitch_deg = -s_beam_ahrs_out.pitch_deg;
            current_settings.pole_tilt_tare_roll_deg  = s_beam_ahrs_out.roll_deg;
        } else {
            current_settings.pole_tilt_tare_pitch_deg = s_cached_imu_pitch;
            current_settings.pole_tilt_tare_roll_deg  = s_cached_imu_roll;
        }
        mark_settings_dirty();
        play_rover_beep(1800, 80);
    }

    // Update 360-deg CAL MAG ellipsoid wizard on active magnetometer stream
    {
        float cal_mag_raw[3] = {0.0f, 0.0f, 0.0f};
        bool  cal_mag_ok = false;
        if (s_beam_10dof_active && s_beam_state.mag_valid && s_beam_ahrs) {
            ahrs_9dof_transform_vec(s_beam_ahrs->cfg.orient_matrix, s_beam_state.mag_ut, cal_mag_raw);
            cal_mag_ok = true;
        } else if (s_cached_mag.present && s_cached_mag.valid) {
            memcpy(cal_mag_raw, s_cached_mag.mag_uT, sizeof(cal_mag_raw));
            cal_mag_ok = true;
        }
        mag_cal_update_wizard(&s_mag_cal_wiz, cal_mag_raw, cal_mag_ok, now_ms);
    }
}

extern "C" void rover_sensors_apply_to_ui_state(uint32_t now_ms, RoverUIData_t *ui_state)
{
    if (!ui_state) return;

    ui_state->imu_pitch_deg = s_cached_imu_pitch;
    ui_state->imu_roll_deg  = s_cached_imu_roll;
    ui_state->display_pose  = s_cached_pose;
    ui_state->imu_acc[0]    = s_cached_acc[0];
    ui_state->imu_acc[1]    = s_cached_acc[1];
    ui_state->imu_acc[2]    = s_cached_acc[2];
    ui_state->imu_gyr[0]    = s_cached_gyr[0];
    ui_state->imu_gyr[1]    = s_cached_gyr[1];
    ui_state->imu_gyr[2]    = s_cached_gyr[2];

    // Sensor Priority Hierarchy:
    // Tier A: Rigid-beam SEN0696 10DOF (I2C1 IO47/48, or I2C0 fallback) + 9-DOF AHRS
    // Tier B: Legacy MMC5983MA (0x30) + QMI8658 (0x6B)
    // Tier C: Onboard QMI8658 6-DOF
    NavHeadingSensorInput_t hdg_in = {};
    hdg_in.tip_lat                  = ui_state->tip_lat;
    hdg_in.tip_lon                  = ui_state->tip_lon;
    hdg_in.now_ms                   = now_ms;
    hdg_in.dual_antenna_valid       = (ui_state->heading_valid &&
                                       (ui_state->heading_source == HDG_SRC_DUAL_FIX ||
                                        ui_state->heading_source == HDG_SRC_DUAL_FLT));
    hdg_in.dual_antenna_fixed       = ui_state->heading_is_fixed;
    hdg_in.dual_heading_deg         = ui_state->true_heading_deg;
    hdg_in.dual_baseline_length_m   = ui_state->baseline_length_m;
    hdg_in.um982_baseline_pitch_deg = ui_state->baseline_pitch_deg;

    if (s_beam_10dof_active) {
        ui_state->pole_bubble_x_deg   = s_beam_ahrs_out.roll_deg   - current_settings.pole_tilt_tare_roll_deg;
        ui_state->pole_bubble_y_deg   = -s_beam_ahrs_out.pitch_deg - current_settings.pole_tilt_tare_pitch_deg;
        ui_state->mag_present         = true;
        ui_state->mag_uT[0]           = s_beam_ahrs_out.mag_cal_uT[0];
        ui_state->mag_uT[1]           = s_beam_ahrs_out.mag_cal_uT[1];
        ui_state->mag_uT[2]           = s_beam_ahrs_out.mag_cal_uT[2];
        ui_state->mag_total_uT        = s_beam_ahrs_out.mag_norm_uT;
        ui_state->mag_die_temp_c      = 0.0f;
        ui_state->mag_set_reset_count = 0U;

        hdg_in.yaw_rate_cw_dps       = s_beam_ahrs_out.world_yaw_rate_cw_dps;
        hdg_in.mag_present           = true;
        hdg_in.mag_valid             = s_beam_ahrs_out.mag_cal_valid;
        hdg_in.raw_mag_heading_deg   = s_beam_ahrs_out.tilt_comp_mag_heading_deg;
        hdg_in.mag_total_uT          = s_beam_ahrs_out.mag_norm_uT;
        hdg_in.mag_bz_world_uT       = s_beam_ahrs_out.mag_bz_world_uT;
        hdg_in.has_bz_world          = true;
        hdg_in.is_beam_10dof         = true;
        hdg_in.beam_pitch_deg        = s_beam_ahrs_out.pitch_deg;
    } else {
        ui_state->pole_bubble_x_deg   = s_cached_bubble_x;
        ui_state->pole_bubble_y_deg   = s_cached_bubble_y;
        ui_state->mag_present         = s_cached_mag.present;
        ui_state->mag_uT[0]           = s_cached_mag.mag_uT[0];
        ui_state->mag_uT[1]           = s_cached_mag.mag_uT[1];
        ui_state->mag_uT[2]           = s_cached_mag.mag_uT[2];
        ui_state->mag_total_uT        = s_cached_mag.total_uT;
        ui_state->mag_die_temp_c      = s_cached_mag.die_temp_c;
        ui_state->mag_set_reset_count = s_cached_mag.set_reset_count;

        hdg_in.yaw_rate_cw_dps       = s_cached_gyr[2];
        hdg_in.mag_present           = s_cached_mag.present;
        hdg_in.mag_valid             = s_cached_mag.valid;
        hdg_in.raw_mag_heading_deg   = s_cached_mag.raw_mag_heading_deg;
        hdg_in.mag_total_uT          = s_cached_mag.total_uT;
    }

    float pole_tilt_deg = sqrtf(ui_state->pole_bubble_x_deg * ui_state->pole_bubble_x_deg +
                                ui_state->pole_bubble_y_deg * ui_state->pole_bubble_y_deg);
    ui_state->pole_tilt_deg = pole_tilt_deg;
    hdg_in.pole_tilt_deg    = pole_tilt_deg;

    // Phase 5, Phase 8 & v0.9.0: Deep-Woods Heading Fallback Engine
    nav_heading_beam_update(&hdg_in,
                            &ui_state->heading_valid,
                            &ui_state->true_heading_deg,
                            &ui_state->heading_source,
                            &ui_state->displacement_dist_m,
                            &ui_state->mag_heading_deg,
                            &ui_state->mag_anomaly);
    ui_state->mag_cal_valid           = current_settings.mag_cal_valid;
    ui_state->mag_tare_offset_deg     = current_settings.mag_true_north_offset_deg;
    ui_state->mag_ellipsoid_cal_valid = current_settings.mag_ellipsoid_cal_valid;

    // Populate SEN0696 / BMP581 / CAL MAG / I2C Diagnostics
    ui_state->beam_10dof_present   = fic_sen0696_is_present(&s_beam_ctx);
    ui_state->beam_10dof_valid     = s_beam_10dof_active;
    ui_state->beam_10dof_bus       = fic_sen0696_get_i2c_bus(&s_beam_ctx);
    ui_state->beam_10dof_addr      = s_beam_state.i2c_addr;
    ui_state->beam_gyro_foc_locked = s_beam_ahrs_has_sample && s_beam_ahrs_out.gyro_foc_locked;
    ui_state->beam_pitch_deg       = s_beam_ahrs_out.pitch_deg;
    ui_state->beam_roll_deg        = s_beam_ahrs_out.roll_deg;
    ui_state->mag_cal_status       = (uint8_t)s_mag_cal_wiz.status;
    ui_state->mag_cal_progress_pct = s_mag_cal_wiz.progress_pct;
    ui_state->mag_cal_samples      = s_mag_cal_wiz.samples;
    ui_state->mag_cal_rms_pct      = s_mag_cal_wiz.rms_pct;
    ui_state->i2c0_err_count       = fic_i2c_get_bus_err_count(0U);
    ui_state->i2c1_err_count       = fic_i2c_get_bus_err_count(1U);

    if (s_beam_10dof_active && ((now_ms - s_last_10dof_log_ms) >= 1000U)) {
        s_last_10dof_log_ms = now_ms;
        const char *src_tag =
            (ui_state->heading_source == HDG_SRC_DUAL_FIX) ? "FIX" :
            (ui_state->heading_source == HDG_SRC_DUAL_FLT) ? "FLT" :
            (ui_state->heading_source == HDG_SRC_DISP)     ? "WLK" :
            (ui_state->heading_source == HDG_SRC_COMP)     ? "CMP" :
            (ui_state->heading_source == HDG_SRC_GYRO)     ? "GYR" :
            (ui_state->heading_source == HDG_SRC_COG)      ? "COG" :
            (ui_state->heading_source == HDG_SRC_HOLD)     ? "HLD" : "---";
        char g1_extra[56];
        if (ui_state->heading_source == HDG_SRC_DUAL_FIX || ui_state->heading_source == HDG_SRC_DUAL_FLT) {
            float dh = ui_state->mag_heading_deg - ui_state->true_heading_deg;
            while (dh > 180.0f)  dh -= 360.0f;
            while (dh < -180.0f) dh += 360.0f;
            float bp_tare = (isfinite(current_settings.beam_pitch_tare_deg) &&
                             fabsf(current_settings.beam_pitch_tare_deg) < 45.0f)
                            ? current_settings.beam_pitch_tare_deg : 0.0f;
            float dp = (s_beam_ahrs_out.pitch_deg - bp_tare) - ui_state->baseline_pitch_deg;
            snprintf(g1_extra, sizeof(g1_extra), " | dH:%+.1f dP:%+.1f | I2C:%lu/%lu",
                     dh, dp, (unsigned long)ui_state->i2c0_err_count, (unsigned long)ui_state->i2c1_err_count);
        } else {
            snprintf(g1_extra, sizeof(g1_extra), " | I2C:%lu/%lu",
                     (unsigned long)ui_state->i2c0_err_count, (unsigned long)ui_state->i2c1_err_count);
        }
        Serial.printf("[10DOF I2C%u] P/R:%+.1f/%+.1f deg | |B|:%.1fuT | Hdg:%.1f[%s] Mag:%.1f%s | FOC:%s | P:%.2fhPa%s\n",
                      (unsigned)fic_sen0696_get_i2c_bus(&s_beam_ctx),
                      s_beam_ahrs_out.pitch_deg,
                      s_beam_ahrs_out.roll_deg,
                      s_beam_ahrs_out.mag_cal_valid ? s_beam_ahrs_out.mag_norm_uT : s_beam_state.mag_norm_ut,
                      ui_state->heading_valid ? ui_state->true_heading_deg : 0.0f,
                      src_tag,
                      ui_state->mag_heading_deg,
                      ui_state->mag_anomaly ? " ANOM" : "",
                      s_beam_ahrs_out.gyro_foc_locked ? "OK" : "WAIT",
                      s_beam_state.pressure_hpa,
                      g1_extra);
    }

    if (fic_sen0696_is_present(&s_beam_ctx) && s_beam_state.press_valid) {
        ui_state->beam_pressure_hpa = s_beam_state.pressure_hpa;
        if (ui_state->fix_type == GNSS_FIX_RTK_FIXED && isfinite(ui_state->tip_height_m)) {
            float raw_diff = (float)ui_state->tip_height_m - s_beam_state.altitude_m;
            if (!s_baro_bias_valid) {
                s_baro_elev_bias_m = raw_diff;
                s_baro_bias_valid  = true;
            } else {
                s_baro_elev_bias_m = 0.98f * s_baro_elev_bias_m + 0.02f * raw_diff;
            }
        }
        ui_state->beam_baro_bias_valid = s_baro_bias_valid;
        ui_state->beam_baro_rel_elev_m = s_baro_bias_valid
                                         ? (s_beam_state.altitude_m + s_baro_elev_bias_m)
                                         : s_beam_state.altitude_m;
    } else {
        ui_state->beam_pressure_hpa    = 0.0f;
        ui_state->beam_baro_bias_valid = false;
        ui_state->beam_baro_rel_elev_m = 0.0f;
    }
}

extern "C" void rover_sensors_get_pole_tilt_snapshot(float *bubble_x, float *bubble_y, float *pole_tilt)
{
    portENTER_CRITICAL(&s_imu_snapshot_mux);
    if (bubble_x)  *bubble_x  = s_snapshot_bubble_x;
    if (bubble_y)  *bubble_y  = s_snapshot_bubble_y;
    if (pole_tilt) *pole_tilt = s_snapshot_pole_tilt;
    portEXIT_CRITICAL(&s_imu_snapshot_mux);
}

extern "C" bool rover_sensors_is_imu_motion_recent(uint32_t now_ms, uint32_t window_ms)
{
    return (s_last_imu_motion_ms != 0U) && ((now_ms - s_last_imu_motion_ms) < window_ms);
}

extern "C" bool rover_sensors_is_mag_cal_collecting(void)
{
    return s_mag_cal_wiz.status == MAG_CAL_COLLECTING;
}

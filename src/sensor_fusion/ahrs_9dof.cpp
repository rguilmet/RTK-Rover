#include "ahrs_9dof.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define AHRS_DEG_TO_RAD ((float)(M_PI / 180.0))
#define AHRS_RAD_TO_DEG ((float)(180.0 / M_PI))

static const int8_t kOrientTables[AHRS_ORIENT_COUNT][3][3] = {
    // 0: Face-Up 0 deg: (+X, +Y, +Z), det = +1
    {{ 1,  0,  0}, { 0,  1,  0}, { 0,  0,  1}},
    // 1: Face-Up 90 deg CCW: (-Y, +X, +Z), det = +1
    {{ 0, -1,  0}, { 1,  0,  0}, { 0,  0,  1}},
    // 2: Face-Up 180 deg: (-X, -Y, +Z), det = +1
    {{-1,  0,  0}, { 0, -1,  0}, { 0,  0,  1}},
    // 3: Face-Up 270 deg CCW: (+Y, -X, +Z), det = +1
    {{ 0,  1,  0}, {-1,  0,  0}, { 0,  0,  1}},
    // 4: Face-Down 0 deg (default SEN0696 beam mount, Acc Z = -1g): (+X, -Y, -Z), det = +1
    {{ 1,  0,  0}, { 0, -1,  0}, { 0,  0, -1}},
    // 5: Face-Down 90 deg: (+Y, +X, -Z), det = +1
    {{ 0,  1,  0}, { 1,  0,  0}, { 0,  0, -1}},
    // 6: Face-Down 180 deg: (-X, +Y, -Z), det = +1
    {{-1,  0,  0}, { 0,  1,  0}, { 0,  0, -1}},
    // 7: Face-Down 270 deg: (-Y, -X, -Z), det = +1
    {{ 0, -1,  0}, {-1,  0,  0}, { 0,  0, -1}},
};

static float inv_sqrt_f(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    return 1.0f / sqrtf(x);
}

static float wrap_360_f(float deg)
{
    while (deg < 0.0f) {
        deg += 360.0f;
    }
    while (deg >= 360.0f) {
        deg -= 360.0f;
    }
    return deg;
}

extern "C" void ahrs_9dof_get_orient_matrix(uint8_t mode, int8_t out_m[3][3])
{
    if (!out_m) {
        return;
    }
    uint8_t idx = (mode < AHRS_ORIENT_COUNT) ? mode : (uint8_t)AHRS_ORIENT_DOWN_0;
    memcpy(out_m, kOrientTables[idx], sizeof(kOrientTables[idx]));
}

extern "C" void ahrs_9dof_default_config(Ahrs9DofConfig_t *cfg, uint8_t orient_mode)
{
    if (!cfg) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    ahrs_9dof_get_orient_matrix(orient_mode, cfg->orient_matrix);
    cfg->beta                = 0.05f;
    cfg->accel_gate_g        = 0.12f;
    cfg->gyro_gate_dps       = 12.0f;
    cfg->foc_var_thresh      = 0.0015f;
    cfg->mag_ellipsoid_valid = false;
    cfg->mag_soft_iron[0][0] = 1.0f;
    cfg->mag_soft_iron[1][1] = 1.0f;
    cfg->mag_soft_iron[2][2] = 1.0f;
}

extern "C" void ahrs_9dof_init(Ahrs9DofState_t *st, const Ahrs9DofConfig_t *cfg)
{
    if (!st) {
        return;
    }
    memset(st, 0, sizeof(*st));
    if (cfg) {
        st->cfg = *cfg;
    } else {
        ahrs_9dof_default_config(&st->cfg, AHRS_ORIENT_DOWN_0);
    }
    st->q0          = 1.0f;
    st->q1          = 0.0f;
    st->q2          = 0.0f;
    st->q3          = 0.0f;
    st->initialized = false;
}

extern "C" void ahrs_9dof_set_orient_mode(Ahrs9DofState_t *st, uint8_t orient_mode)
{
    if (!st) {
        return;
    }
    ahrs_9dof_get_orient_matrix(orient_mode, st->cfg.orient_matrix);
    st->initialized  = false;
    st->hist_idx     = 0;
    st->hist_count   = 0;
    st->stable_ticks = 0;
}

extern "C" void ahrs_9dof_set_mag_cal(Ahrs9DofState_t *st,
                                      const float hard_iron_uT[3],
                                      const float soft_iron_3x3[9],
                                      bool valid)
{
    if (!st) {
        return;
    }
    if (hard_iron_uT) {
        st->cfg.mag_hard_iron_uT[0] = hard_iron_uT[0];
        st->cfg.mag_hard_iron_uT[1] = hard_iron_uT[1];
        st->cfg.mag_hard_iron_uT[2] = hard_iron_uT[2];
    } else {
        st->cfg.mag_hard_iron_uT[0] = 0.0f;
        st->cfg.mag_hard_iron_uT[1] = 0.0f;
        st->cfg.mag_hard_iron_uT[2] = 0.0f;
    }
    if (soft_iron_3x3) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                st->cfg.mag_soft_iron[i][j] = soft_iron_3x3[i * 3 + j];
            }
        }
    } else {
        memset(st->cfg.mag_soft_iron, 0, sizeof(st->cfg.mag_soft_iron));
        st->cfg.mag_soft_iron[0][0] = 1.0f;
        st->cfg.mag_soft_iron[1][1] = 1.0f;
        st->cfg.mag_soft_iron[2][2] = 1.0f;
    }
    st->cfg.mag_ellipsoid_valid = valid;
}

extern "C" void ahrs_9dof_transform_vec(const int8_t m[3][3], const float in_v[3], float out_v[3])
{
    if (!m || !in_v || !out_v) {
        return;
    }
    float v0 = in_v[0];
    float v1 = in_v[1];
    float v2 = in_v[2];
    for (int i = 0; i < 3; i++) {
        out_v[i] = (float)m[i][0] * v0 + (float)m[i][1] * v1 + (float)m[i][2] * v2;
    }
}

extern "C" void ahrs_9dof_madgwick_imu(Ahrs9DofState_t *st,
                                       float gx, float gy, float gz,
                                       float ax, float ay, float az,
                                       float dt_s)
{
    if (!st || dt_s <= 0.0f || dt_s > 0.5f) {
        return;
    }
    float q0 = st->q0, q1 = st->q1, q2 = st->q2, q3 = st->q3;
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    float acc_sq = ax * ax + ay * ay + az * az;
    if (acc_sq > 1e-6f) {
        float recipNorm = inv_sqrt_f(acc_sq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;
        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

        float s_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (s_sq > 1e-12f) {
            recipNorm = inv_sqrt_f(s_sq);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
            float beta = st->cfg.beta;
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }
    }

    q0 += qDot1 * dt_s;
    q1 += qDot2 * dt_s;
    q2 += qDot3 * dt_s;
    q3 += qDot4 * dt_s;

    float q_sq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
    float q_inv = inv_sqrt_f(q_sq);
    if (q_inv > 0.0f) {
        st->q0 = q0 * q_inv;
        st->q1 = q1 * q_inv;
        st->q2 = q2 * q_inv;
        st->q3 = q3 * q_inv;
    }
}

extern "C" void ahrs_9dof_madgwick_9dof(Ahrs9DofState_t *st,
                                        float gx, float gy, float gz,
                                        float ax, float ay, float az,
                                        float mx, float my, float mz,
                                        float dt_s)
{
    if (!st || dt_s <= 0.0f || dt_s > 0.5f) {
        return;
    }
    float mag_sq = mx * mx + my * my + mz * mz;
    if (mag_sq <= 1e-6f) {
        ahrs_9dof_madgwick_imu(st, gx, gy, gz, ax, ay, az, dt_s);
        return;
    }

    float q0 = st->q0, q1 = st->q1, q2 = st->q2, q3 = st->q3;
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    float acc_sq = ax * ax + ay * ay + az * az;
    if (acc_sq > 1e-6f) {
        float recipNorm = inv_sqrt_f(acc_sq);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        recipNorm = inv_sqrt_f(mag_sq);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        float _2q0mx = 2.0f * q0 * mx;
        float _2q0my = 2.0f * q0 * my;
        float _2q0mz = 2.0f * q0 * mz;
        float _2q1mx = 2.0f * q1 * mx;
        float _2q0   = 2.0f * q0;
        float _2q1   = 2.0f * q1;
        float _2q2   = 2.0f * q2;
        float _2q3   = 2.0f * q3;
        float _2q0q2 = 2.0f * q0 * q2;
        float _2q2q3 = 2.0f * q2 * q3;
        float q0q0 = q0 * q0;
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q3q3 = q3 * q3;

        float hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        float hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        float _2bx = sqrtf(hx * hx + hy * hy);
        float _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        float _4bx = 2.0f * _2bx;
        float _4bz = 2.0f * _2bz;

        float s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        float s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        float s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        float s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

        float s_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (s_sq > 1e-12f) {
            recipNorm = inv_sqrt_f(s_sq);
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
            float beta = st->cfg.beta;
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;
        }
    }

    q0 += qDot1 * dt_s;
    q1 += qDot2 * dt_s;
    q2 += qDot3 * dt_s;
    q3 += qDot4 * dt_s;

    float q_inv = inv_sqrt_f(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (q_inv > 0.0f) {
        st->q0 = q0 * q_inv;
        st->q1 = q1 * q_inv;
        st->q2 = q2 * q_inv;
        st->q3 = q3 * q_inv;
    }
}

static void ahrs_update_stationary_foc(Ahrs9DofState_t *st, const float acc[3], const float gyr[3])
{
    for (int i = 0; i < 3; i++) {
        st->acc_hist[st->hist_idx][i] = acc[i];
        st->gyr_hist[st->hist_idx][i] = gyr[i];
    }
    st->hist_idx = (st->hist_idx + 1) % AHRS_9DOF_FOC_WINDOW;
    if (st->hist_count < AHRS_9DOF_FOC_WINDOW) {
        st->hist_count++;
        return;
    }

    float mx = 0.0f, my = 0.0f, mz = 0.0f;
    for (int i = 0; i < AHRS_9DOF_FOC_WINDOW; i++) {
        mx += st->acc_hist[i][0];
        my += st->acc_hist[i][1];
        mz += st->acc_hist[i][2];
    }
    mx /= (float)AHRS_9DOF_FOC_WINDOW;
    my /= (float)AHRS_9DOF_FOC_WINDOW;
    mz /= (float)AHRS_9DOF_FOC_WINDOW;

    float var = 0.0f;
    for (int i = 0; i < AHRS_9DOF_FOC_WINDOW; i++) {
        float dx = st->acc_hist[i][0] - mx;
        float dy = st->acc_hist[i][1] - my;
        float dz = st->acc_hist[i][2] - mz;
        var += dx * dx + dy * dy + dz * dz;
    }
    var /= (float)AHRS_9DOF_FOC_WINDOW;

    if (var < st->cfg.foc_var_thresh) {
        st->stable_ticks++;
        if (st->stable_ticks >= AHRS_9DOF_FOC_WINDOW) {
            float gmx = 0.0f, gmy = 0.0f, gmz = 0.0f;
            for (int i = 0; i < AHRS_9DOF_FOC_WINDOW; i++) {
                gmx += st->gyr_hist[i][0];
                gmy += st->gyr_hist[i][1];
                gmz += st->gyr_hist[i][2];
            }
            float bx = gmx / (float)AHRS_9DOF_FOC_WINDOW;
            float by = gmy / (float)AHRS_9DOF_FOC_WINDOW;
            float bz = gmz / (float)AHRS_9DOF_FOC_WINDOW;
            // Reject implausible FOC captures during sustained constant turn rates
            if (fabsf(bx) < 8.0f && fabsf(by) < 8.0f && fabsf(bz) < 8.0f) {
                st->gyro_bias_dps[0] = bx;
                st->gyro_bias_dps[1] = by;
                st->gyro_bias_dps[2] = bz;
                st->gyro_foc_locked  = true;
                st->foc_update_count++;
            }
            st->stable_ticks = 40; // Re-evaluate every 10 stationary ticks without spike resets
        }
    } else {
        st->stable_ticks = 0;
    }
}

static void ahrs_seed_quat_from_gravity(Ahrs9DofState_t *st, const float acc[3])
{
    float r_yz = sqrtf(acc[1] * acc[1] + acc[2] * acc[2]);
    if (r_yz < 1e-4f) {
        return;
    }
    float roll_rad  = atan2f(acc[1], acc[2]);
    float pitch_rad = -atan2f(acc[0], r_yz);
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);

    st->q0 = cr * cp;
    st->q1 = sr * cp;
    st->q2 = cr * sp;
    st->q3 = -sr * sp;
    float inv_q = inv_sqrt_f(st->q0 * st->q0 + st->q1 * st->q1 + st->q2 * st->q2 + st->q3 * st->q3);
    if (inv_q > 0.0f) {
        st->q0 *= inv_q;
        st->q1 *= inv_q;
        st->q2 *= inv_q;
        st->q3 *= inv_q;
        st->initialized = true;
    }
}

extern "C" void ahrs_9dof_update(Ahrs9DofState_t *st,
                                 const float raw_acc_g[3],
                                 const float raw_gyr_dps[3],
                                 const float raw_mag_uT[3],
                                 bool mag_valid,
                                 bool use_mag_in_quat,
                                 float dt_s,
                                 Ahrs9DofOutput_t *out)
{
    if (!st || !raw_acc_g || !raw_gyr_dps || !out) {
        return;
    }
    memset(out, 0, sizeof(*out));

    ahrs_9dof_transform_vec(st->cfg.orient_matrix, raw_acc_g, out->acc_beam_g);
    float gyr_beam_raw[3];
    ahrs_9dof_transform_vec(st->cfg.orient_matrix, raw_gyr_dps, gyr_beam_raw);

    ahrs_update_stationary_foc(st, out->acc_beam_g, gyr_beam_raw);
    out->gyro_foc_locked = st->gyro_foc_locked;

    float gx = gyr_beam_raw[0] - st->gyro_bias_dps[0];
    float gy = gyr_beam_raw[1] - st->gyro_bias_dps[1];
    float gz = gyr_beam_raw[2] - st->gyro_bias_dps[2];
    out->gyr_beam_dps[0] = gx;
    out->gyr_beam_dps[1] = gy;
    out->gyr_beam_dps[2] = gz;

    float acc_norm = sqrtf(out->acc_beam_g[0] * out->acc_beam_g[0] +
                           out->acc_beam_g[1] * out->acc_beam_g[1] +
                           out->acc_beam_g[2] * out->acc_beam_g[2]);
    float gyr_norm = sqrtf(gx * gx + gy * gy + gz * gz);

    if (!st->initialized && acc_norm > 0.7f && acc_norm < 1.3f) {
        ahrs_seed_quat_from_gravity(st, out->acc_beam_g);
    }

    const bool accel_gated = (fabsf(acc_norm - 1.0f) > st->cfg.accel_gate_g) ||
                             (gyr_norm > st->cfg.gyro_gate_dps);
    out->accel_gated = accel_gated;

    float mag_beam[3] = {0.0f, 0.0f, 0.0f};
    if (mag_valid && raw_mag_uT) {
        ahrs_9dof_transform_vec(st->cfg.orient_matrix, raw_mag_uT, mag_beam);
        float dx = mag_beam[0] - st->cfg.mag_hard_iron_uT[0];
        float dy = mag_beam[1] - st->cfg.mag_hard_iron_uT[1];
        float dz = mag_beam[2] - st->cfg.mag_hard_iron_uT[2];
        out->mag_cal_uT[0] = st->cfg.mag_soft_iron[0][0] * dx + st->cfg.mag_soft_iron[0][1] * dy + st->cfg.mag_soft_iron[0][2] * dz;
        out->mag_cal_uT[1] = st->cfg.mag_soft_iron[1][0] * dx + st->cfg.mag_soft_iron[1][1] * dy + st->cfg.mag_soft_iron[1][2] * dz;
        out->mag_cal_uT[2] = st->cfg.mag_soft_iron[2][0] * dx + st->cfg.mag_soft_iron[2][1] * dy + st->cfg.mag_soft_iron[2][2] * dz;
        out->mag_norm_uT   = sqrtf(out->mag_cal_uT[0] * out->mag_cal_uT[0] +
                                   out->mag_cal_uT[1] * out->mag_cal_uT[1] +
                                   out->mag_cal_uT[2] * out->mag_cal_uT[2]);
        out->mag_cal_valid = (out->mag_norm_uT > 1.0f && isfinite(out->mag_norm_uT));
    }

    float ax_in = accel_gated ? 0.0f : out->acc_beam_g[0];
    float ay_in = accel_gated ? 0.0f : out->acc_beam_g[1];
    float az_in = accel_gated ? 0.0f : out->acc_beam_g[2];
    float gx_r = gx * AHRS_DEG_TO_RAD;
    float gy_r = gy * AHRS_DEG_TO_RAD;
    float gz_r = gz * AHRS_DEG_TO_RAD;

    if (use_mag_in_quat && out->mag_cal_valid && !accel_gated) {
        ahrs_9dof_madgwick_9dof(st, gx_r, gy_r, gz_r, ax_in, ay_in, az_in,
                                out->mag_cal_uT[0], out->mag_cal_uT[1], out->mag_cal_uT[2], dt_s);
    } else {
        ahrs_9dof_madgwick_imu(st, gx_r, gy_r, gz_r, ax_in, ay_in, az_in, dt_s);
    }

    // Body-frame World-Up unit vector from Madgwick quaternion
    float q0 = st->q0, q1 = st->q1, q2 = st->q2, q3 = st->q3;
    float ux = 2.0f * (q1 * q3 - q0 * q2);
    float uy = 2.0f * (q0 * q1 + q2 * q3);
    float uz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float u_norm_inv = inv_sqrt_f(ux * ux + uy * uy + uz * uz);
    if (u_norm_inv > 0.0f) {
        ux *= u_norm_inv;
        uy *= u_norm_inv;
        uz *= u_norm_inv;
    } else {
        ux = 0.0f; uy = 0.0f; uz = 1.0f;
    }

    float r_yz = sqrtf(uy * uy + uz * uz);
    out->roll_deg  = atan2f(uy, uz) * AHRS_RAD_TO_DEG;
    out->pitch_deg = (r_yz > 1e-4f) ? (atan2f(ux, r_yz) * AHRS_RAD_TO_DEG) : ((ux >= 0.0f) ? 90.0f : -90.0f);

    // Clockwise world-vertical yaw rate (+CW matches compass azimuth)
    out->world_yaw_rate_cw_dps = -(ux * gx + uy * gy + uz * gz);

    if (out->mag_cal_valid) {
        out->mag_bz_world_uT = out->mag_cal_uT[0] * ux + out->mag_cal_uT[1] * uy + out->mag_cal_uT[2] * uz;

        if (r_yz > 1e-4f) {
            float cos_phi   = uz / r_yz;
            float sin_phi   = uy / r_yz;
            float cos_theta = r_yz;
            float sin_theta = ux;

            float mx_p = out->mag_cal_uT[0];
            float my_p = out->mag_cal_uT[1] * cos_phi - out->mag_cal_uT[2] * sin_phi;
            float mz_p = out->mag_cal_uT[1] * sin_phi + out->mag_cal_uT[2] * cos_phi;

            float mx_h = mx_p * cos_theta - mz_p * sin_theta;
            float my_h = my_p;
            out->tilt_comp_mag_heading_deg = wrap_360_f(atan2f(my_h, mx_h) * AHRS_RAD_TO_DEG);
        } else {
            out->tilt_comp_mag_heading_deg = wrap_360_f(atan2f(out->mag_cal_uT[1], out->mag_cal_uT[0]) * AHRS_RAD_TO_DEG);
        }
    }
}

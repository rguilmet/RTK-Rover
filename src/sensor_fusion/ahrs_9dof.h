#ifndef AHRS_9DOF_H
#define AHRS_9DOF_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AHRS_9DOF_FOC_WINDOW 50

// Mounting orientations (all 8 proper right-handed orthogonal permutations, det = +1):
// Maps raw sensor axes into canonical Beam Frame (+X fwd, +Y left, +Z up = +1g at rest).
// Mode 4 is Face-Down 0 deg (+X, -Y, -Z) for the inverted beam SEN0696 mount (Acc Z = -1g).
typedef enum {
    AHRS_ORIENT_UP_0     = 0,
    AHRS_ORIENT_UP_90    = 1,
    AHRS_ORIENT_UP_180   = 2,
    AHRS_ORIENT_UP_270   = 3,
    AHRS_ORIENT_DOWN_0   = 4,
    AHRS_ORIENT_DOWN_90  = 5,
    AHRS_ORIENT_DOWN_180 = 6,
    AHRS_ORIENT_DOWN_270 = 7,
    AHRS_ORIENT_COUNT    = 8
} AhrsOrientMode_t;

typedef struct {
    int8_t  orient_matrix[3][3];
    float   beta;
    float   accel_gate_g;
    float   gyro_gate_dps;
    float   foc_var_thresh;
    float   mag_hard_iron_uT[3];
    float   mag_soft_iron[3][3];
    bool    mag_ellipsoid_valid;
} Ahrs9DofConfig_t;

typedef struct {
    Ahrs9DofConfig_t cfg;
    float    q0;
    float    q1;
    float    q2;
    float    q3;
    bool     initialized;
    float    gyro_bias_dps[3];
    bool     gyro_foc_locked;
    uint32_t foc_update_count;
    float    acc_hist[AHRS_9DOF_FOC_WINDOW][3];
    float    gyr_hist[AHRS_9DOF_FOC_WINDOW][3];
    int      hist_idx;
    int      hist_count;
    int      stable_ticks;
} Ahrs9DofState_t;

typedef struct {
    float acc_beam_g[3];
    float gyr_beam_dps[3];
    float mag_cal_uT[3];
    float mag_norm_uT;
    float mag_bz_world_uT;
    float roll_deg;
    float pitch_deg;
    float world_yaw_rate_cw_dps;
    float tilt_comp_mag_heading_deg;
    bool  accel_gated;
    bool  gyro_foc_locked;
    bool  mag_cal_valid;
} Ahrs9DofOutput_t;

// Populate a right-handed 3x3 signed axis permutation matrix for mode 0..7.
void ahrs_9dof_get_orient_matrix(uint8_t mode, int8_t out_m[3][3]);

// Populate default configuration parameters for a mounting mode (default 4 = Face-Down 0 deg).
void ahrs_9dof_default_config(Ahrs9DofConfig_t *cfg, uint8_t orient_mode);

// Initialize or reset an instance-safe AHRS filter state.
void ahrs_9dof_init(Ahrs9DofState_t *st, const Ahrs9DofConfig_t *cfg);

// Update mounting orientation and/or magnetometer hard/soft-iron calibration matrices cleanly.
void ahrs_9dof_set_orient_mode(Ahrs9DofState_t *st, uint8_t orient_mode);
void ahrs_9dof_set_mag_cal(Ahrs9DofState_t *st,
                           const float hard_iron_uT[3],
                           const float soft_iron_3x3[9],
                           bool valid);

// Transform a raw 3-axis vector into canonical Beam Frame via cfg.orient_matrix.
void ahrs_9dof_transform_vec(const int8_t m[3][3], const float in_v[3], float out_v[3]);

// Instance-safe Madgwick filter primitives (pure quaternion math, zero globals).
void ahrs_9dof_madgwick_imu(Ahrs9DofState_t *st,
                            float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt_s);

void ahrs_9dof_madgwick_9dof(Ahrs9DofState_t *st,
                             float gx, float gy, float gz,
                             float ax, float ay, float az,
                             float mx, float my, float mz,
                             float dt_s);

// Execute one full beam-frame 9-DOF AHRS + stationary gyro FOC + gated tilt + mag step.
void ahrs_9dof_update(Ahrs9DofState_t *st,
                      const float raw_acc_g[3],
                      const float raw_gyr_dps[3],
                      const float raw_mag_uT[3],
                      bool mag_valid,
                      bool use_mag_in_quat,
                      float dt_s,
                      Ahrs9DofOutput_t *out);

#ifdef __cplusplus
}
#endif

#endif // AHRS_9DOF_H

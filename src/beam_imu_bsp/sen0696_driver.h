#ifndef SEN0696_DRIVER_H
#define SEN0696_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Default DFRobot Gravity SEN0696 (BMI323 + BMM350 + BMP581) I2C address range (A1/A0 DIP)
#define SEN0696_I2C_ADDR_DEFAULT     0x4AU
#define SEN0696_I2C_ADDR_MAX         0x4DU

// Verified DFRobot_Multi_DOF_IMU I2C identity constants
#define SEN0696_EXPECTED_VID         0x3343U
#define SEN0696_MIN_10DOF_PID        0x0010U

/**
 * @brief Persistent state and latest physical readings for DFRobot SEN0696 (10DOF IMU).
 */
typedef struct {
    bool     present;          // True if detected on I2C1 or I2C0 and verified PID >= 0x0010 at boot
    bool     id_valid;         // True if REG_I2C_PID >= 0x0010
    bool     configured;       // True if Normal mode + +/-2g + +/-250dps writes succeeded
    bool     valid;            // True if the latest sample read cleanly
    bool     accel_gyro_valid; // True if latest 12-byte IMU block read passed sanity checks
    bool     mag_valid;        // True if latest 12-byte magnetometer block passed sanity checks
    bool     press_valid;      // True if latest 4-byte barometer block passed sanity checks
    uint8_t  i2c_bus;          // Active I2C peripheral port (1 = isolated I2C1, 0 = shared I2C0 fallback)
    uint8_t  i2c_addr;         // Detected 7-bit I2C address (0 if not found)
    uint16_t vid;              // REG_I2C_VID (0x0000), expected 0x3343 (DFRobot)
    uint16_t pid;              // REG_I2C_PID (0x0001), expected >= 0x0010 (10DOF)
    uint16_t fw_version;       // REG_I2C_FW_VERSION (0x0002)
    float    accel_g[3];       // Accelerometer X, Y, Z in g (+/-2g scale)
    float    gyro_dps[3];      // Gyroscope X, Y, Z in deg/s (+/-250dps scale)
    float    mag_ut[3];        // Magnetometer X, Y, Z in uT (0.01 uT/LSB)
    float    mag_norm_ut;      // Magnetic field magnitude |B| in uT
    float    pressure_pa;      // Barometric pressure in Pa (1 Pa/LSB)
    float    pressure_hpa;     // Barometric pressure in hPa
    float    altitude_m;       // Standard-atmosphere altitude estimate (P0 = 101325 Pa)
    uint8_t  consec_fails;     // Consecutive poll errors since last valid sample
    uint32_t last_sample_ms;   // FreeRTOS tick timestamp (ms) of last valid sample
} sen0696_state_t;

/**
 * @brief Instance context for an attached DFRobot SEN0696 10-DOF sensor.
 */
typedef struct {
    sen0696_state_t          state;
    i2c_master_dev_handle_t  owned_dev_handle;
    i2c_master_dev_handle_t *dev_handle_ptr;
    bool                     link_suppressed;
    bool                     owns_global_beam_handle;
} FicSen0696_t;

/**
 * @brief Quietly probes [addr_min..addr_max] on a specific I2C port (0 or 1) without tripping recovery.
 * @return First responding 7-bit I2C address, or 0 if no SEN0696 responded.
 */
uint8_t sen0696_detect_i2c_addr_on_bus(uint8_t bus_port, uint8_t addr_min, uint8_t addr_max);

/**
 * @brief Quietly probes [addr_min..addr_max] preferring isolated I2C1, falling back to I2C0.
 * @return First responding 7-bit I2C address, or 0 if no SEN0696 responded on either bus.
 */
uint8_t sen0696_detect_i2c_addr(uint8_t addr_min, uint8_t addr_max);

void fic_sen0696_reset(FicSen0696_t *ctx);
bool fic_sen0696_init_with_device(FicSen0696_t *ctx,
                                  uint8_t i2c_bus,
                                  uint8_t i2c_addr,
                                  i2c_master_dev_handle_t dev_handle,
                                  sen0696_state_t *out_state);
bool fic_sen0696_init(FicSen0696_t *ctx, uint8_t addr_min, uint8_t addr_max, sen0696_state_t *out_state);
bool fic_sen0696_is_present(const FicSen0696_t *ctx);
uint8_t fic_sen0696_get_i2c_bus(const FicSen0696_t *ctx);
uint8_t fic_sen0696_get_consec_fails(const FicSen0696_t *ctx);
bool fic_sen0696_poll_step(FicSen0696_t *ctx, uint8_t step, sen0696_state_t *out_state);
bool fic_sen0696_poll_fast(FicSen0696_t *ctx, bool read_baro, sen0696_state_t *out_state);
bool fic_sen0696_poll(FicSen0696_t *ctx, sen0696_state_t *out_state);

#ifdef __cplusplus
}
#endif

#endif // SEN0696_DRIVER_H

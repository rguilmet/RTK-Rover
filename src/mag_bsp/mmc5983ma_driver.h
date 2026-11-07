#ifndef MMC5983MA_DRIVER_H
#define MMC5983MA_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================================
// MEMSIC MMC5983MA 3-Axis 18-Bit AMR Magnetometer Driver (0x30 on I2C0)
// SparkFun Qwiic Micro Magnetometer (SEN-19921)
// Features:
//  - Boot-time auto-detection (probes 0x30 and verifies Reg 0x2F == 0x30)
//  - Hardware SET/RESET degaussing & exact bridge-offset cancellation
//  - 18-bit resolution (16,384 LSB/Gauss = 163.84 LSB/uT, 0.4 mG noise floor)
//  - Tilt-compensated horizontal azimuth using QMI8658 pitch & roll
// =========================================================================

typedef struct {
    bool     present;            // True if MMC5983MA detected at 0x30 on boot
    bool     valid;              // True if latest 18-bit reading is valid
    float    mag_uT[3];          // Bridge-offset-cancelled Bx, By, Bz in microteslas (uT)
    float    total_uT;           // Total magnetic field magnitude ||B|| (uT)
    float    raw_mag_heading_deg;// Tilt-compensated magnetic heading (0..360 deg, before True-North tare)
    float    die_temp_c;         // On-chip die temperature (deg C)
    float    bridge_offset_lsb[3];// Measured SET/RESET null-field bridge offset (nominal ~131072 LSB)
    uint32_t set_reset_count;    // Total hardware SET/RESET calibrations completed
} Mmc5983maState_t;

// Auto-detects MMC5983MA on I2C0 (0x30), runs SW_RST, SET/RESET bridge calibration,
// and configures 50 Hz 18-bit continuous mode. Returns true if present, false if absent.
bool mmc5983ma_init(void);

// Returns true if MMC5983MA was detected at boot
bool mmc5983ma_is_present(void);

// Executes hardware SET/RESET pulse pair to cancel AMR bridge offset & thermal drift
bool mmc5983ma_calibrate_bridge_offset(void);

// Polls 18-bit magnetic field (X, Y, Z), computes tilt-compensated magnetic heading
// using pole pitch/roll (deg), and triggers periodic/thermal SET/RESET recalibration.
bool mmc5983ma_poll(float pitch_deg, float roll_deg, Mmc5983maState_t *state_out);

// Copies latest cached magnetometer state without performing I2C transactions
void mmc5983ma_get_state(Mmc5983maState_t *state_out);

#ifdef __cplusplus
}
#endif

#endif // MMC5983MA_DRIVER_H

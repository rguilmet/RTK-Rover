#ifndef I2C_BSP_H
#define I2C_BSP_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

extern i2c_master_bus_handle_t user_i2c_port0_handle;
extern i2c_master_dev_handle_t disp_touch_dev_handle;
extern i2c_master_dev_handle_t rtc_dev_handle;
extern i2c_master_dev_handle_t imu_dev_handle;
extern i2c_master_dev_handle_t axp2101_dev_handle;

void i2c_bsp_gpio_bus_clear(void);
void i2c_master_Init(void);
void i2c_bsp_reinit_bus(void);
void i2c_bsp_recover_bus(void);
bool i2c_bsp_lock(uint32_t timeout_ms);
void i2c_bsp_unlock(void);
bool i2c_bsp_probe_addr(uint8_t addr_7bit);
uint8_t i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);
uint8_t i2c_master_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen);
uint8_t i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // I2C_BSP_H

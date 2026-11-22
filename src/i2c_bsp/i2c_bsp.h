#ifndef I2C_BSP_H
#define I2C_BSP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*i2c_bsp_bus_hook_t)(i2c_master_bus_handle_t bus_handle, void *ctx);

extern i2c_master_bus_handle_t user_i2c_port0_handle;
extern i2c_master_bus_handle_t user_i2c_port1_handle;
extern i2c_master_dev_handle_t disp_touch_dev_handle;
extern i2c_master_dev_handle_t rtc_dev_handle;
extern i2c_master_dev_handle_t imu_dev_handle;
extern i2c_master_dev_handle_t axp2101_dev_handle;
extern i2c_master_dev_handle_t mag_dev_handle;
extern i2c_master_dev_handle_t beam_imu_dev_handle;

void i2c_bsp_gpio_bus_clear(void);
void i2c_bsp_i2c1_gpio_bus_clear(void);
void i2c_master_Init(void);
void i2c_bsp_i2c1_init(void);
void i2c_bsp_reinit_bus(void);
void i2c_bsp_i2c1_reinit_bus(void);
void i2c_bsp_recover_bus(void);
bool i2c_bsp_attach_beam_imu(uint8_t addr_7bit);
bool i2c_bsp_attach_beam_imu_port(uint8_t port, uint8_t addr_7bit);
void i2c_bsp_detach_beam_imu(void);
uint8_t i2c_bsp_get_beam_imu_bus(void);
uint32_t i2c_bsp_get_bus_err_count(uint8_t port);
bool i2c_bsp_register_bus_hooks(i2c_bsp_bus_hook_t detach_fn, i2c_bsp_bus_hook_t reattach_fn, void *ctx);
void i2c_bsp_unregister_bus_hooks(void *ctx);
bool i2c_bsp_has_sentinel_run(const uint8_t *buf, size_t len, size_t min_run);
bool i2c_bsp_has_sentinel_tail(const uint8_t *buf, size_t len, size_t tail_len);
void i2c_bsp_report_status(i2c_master_dev_handle_t dev_handle, const char *action, int err_code);
void i2c_bsp_report_status_port(uint8_t port, i2c_master_dev_handle_t dev_handle, const char *action, int err_code);
bool i2c_bsp_lock(uint32_t timeout_ms);
void i2c_bsp_unlock(void);
bool i2c_bsp_lock_port(uint8_t port, uint32_t timeout_ms);
void i2c_bsp_unlock_port(uint8_t port);
bool i2c_bsp_probe_addr(uint8_t addr_7bit);
bool i2c_bsp_probe_optional_addr(uint8_t addr_7bit);
bool i2c_bsp_probe_optional_addr_port(uint8_t port, uint8_t addr_7bit);
uint8_t i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);
uint8_t i2c_master_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen);
uint8_t i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif // I2C_BSP_H

#ifndef AXP2101_BSP_H
#define AXP2101_BSP_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool    online;
    float   battery_voltage_v;
    uint8_t battery_percent;
    bool    is_charging;
    bool    vbus_good;
    float   vbus_voltage_v;
} Axp2101Status_t;

void axp2101_bsp_init(void);
void axp2101_bsp_read(Axp2101Status_t *status);
void axp2101_bsp_power_off(void);

#ifdef __cplusplus
}
#endif

#endif // AXP2101_BSP_H

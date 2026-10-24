#ifndef RTC_BSP_H
#define RTC_BSP_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  week;
} RtcDateTime_t;

#ifdef __cplusplus
extern "C" {
#endif

void rtc_bsp_init(void);
void i2c_rtc_setTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
RtcDateTime_t i2c_rtc_get(void);
void get_local_datetime(RtcDateTime_t *local_dt);
void get_formatted_timestamp(char *buffer, size_t max_len, bool filename_safe);

#ifdef __cplusplus
}
#endif

#endif // RTC_BSP_H

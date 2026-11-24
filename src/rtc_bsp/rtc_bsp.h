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

typedef int16_t (*rtc_utc_offset_cb_t)(void);
typedef bool    (*rtc_allow_sys_clock_sync_cb_t)(void);

void rtc_bsp_set_utc_offset_minutes(int16_t offset_min);
int16_t rtc_bsp_get_utc_offset_minutes(void);
void rtc_bsp_set_utc_offset_provider(rtc_utc_offset_cb_t provider_cb);
void rtc_bsp_set_sys_clock_sync_guard(rtc_allow_sys_clock_sync_cb_t allow_cb);

void rtc_bsp_init(void);
void i2c_rtc_setTime_ex(uint16_t year, uint8_t month, uint8_t day,
                        uint8_t hour, uint8_t minute, uint8_t second,
                        bool sync_sys_clock);
void i2c_rtc_setTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);
RtcDateTime_t i2c_rtc_get(void);
void get_local_datetime(RtcDateTime_t *local_dt);
void get_formatted_timestamp(char *buffer, size_t max_len, bool filename_safe);

#ifdef __cplusplus
}
#endif

#endif // RTC_BSP_H

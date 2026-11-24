#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Wire.h>
#include <sys/time.h>
#include <time.h>
#include "RtcDrv.hpp"
#include "rtc_bsp.h"
#include "../i2c_bsp/i2c_bsp.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "rtc_bsp";

static SensorPCF85063 rtc;
static const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static RtcDateTime_t cached_rtc_time = {2026, 1, 1, 0, 0, 0, 0};
static uint32_t      last_rtc_read_ms = 0;

static int16_t                       s_utc_offset_min    = -300;
static rtc_utc_offset_cb_t           s_utc_offset_cb     = NULL;
static rtc_allow_sys_clock_sync_cb_t s_allow_sys_sync_cb = NULL;

static bool isLeapYear(int y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

static bool rtc_i2c_custom_cb(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len, bool writeReg, bool isWrite)
{
    (void)addr;
    int reg_addr = writeReg ? reg : -1;
    if (isWrite) {
        return i2c_write_buff(rtc_dev_handle, reg_addr, buf, (uint8_t)len) == ESP_OK;
    } else {
        return i2c_read_buff(rtc_dev_handle, reg_addr, buf, (uint8_t)len) == ESP_OK;
    }
}

extern "C" void rtc_bsp_set_utc_offset_minutes(int16_t offset_min)
{
    s_utc_offset_min = offset_min;
}

extern "C" int16_t rtc_bsp_get_utc_offset_minutes(void)
{
    if (s_utc_offset_cb != NULL) {
        return s_utc_offset_cb();
    }
    return s_utc_offset_min;
}

extern "C" void rtc_bsp_set_utc_offset_provider(rtc_utc_offset_cb_t provider_cb)
{
    s_utc_offset_cb = provider_cb;
}

extern "C" void rtc_bsp_set_sys_clock_sync_guard(rtc_allow_sys_clock_sync_cb_t allow_cb)
{
    s_allow_sys_sync_cb = allow_cb;
}

extern "C" void rtc_bsp_init(void)
{
    if (rtc_dev_handle == NULL) {
        ESP_LOGW(TAG, "RTC I2C Handle not initialized yet!");
        return;
    }

    if (!rtc.begin(rtc_i2c_custom_cb)) {
        ESP_LOGW(TAG, "Failed to find PCF85063 - check your wiring!");
    } else {
        ESP_LOGI(TAG, "PCF85063 RTC initialized successfully.");
    }
}

static void sync_system_clock(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
    if (year >= 2020 && month >= 1 && month <= 12 && day >= 1 && day <= 31 &&
        hour <= 23 && minute <= 59 && second <= 59) {
        struct tm tm_buf;
        memset(&tm_buf, 0, sizeof(tm_buf));
        tm_buf.tm_year = year - 1900;
        tm_buf.tm_mon  = month - 1;
        tm_buf.tm_mday = day;
        tm_buf.tm_hour = hour;
        tm_buf.tm_min  = minute;
        tm_buf.tm_sec  = second;
        time_t epoch = mktime(&tm_buf);
        if (epoch > 0) {
            struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
            settimeofday(&tv, NULL);
        }
    }
}

extern "C" void i2c_rtc_setTime_ex(uint16_t year, uint8_t month, uint8_t day,
                                   uint8_t hour, uint8_t minute, uint8_t second,
                                   bool sync_sys_clock)
{
    rtc.setDateTime(year, month, day, hour, minute, second);
    cached_rtc_time.year   = year;
    cached_rtc_time.month  = month;
    cached_rtc_time.day    = day;
    cached_rtc_time.hour   = hour;
    cached_rtc_time.minute = minute;
    cached_rtc_time.second = second;
    last_rtc_read_ms       = millis();
    if (sync_sys_clock) {
        sync_system_clock(year, month, day, hour, minute, second);
    }
}

extern "C" void i2c_rtc_setTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    bool sync_sys = (s_allow_sys_sync_cb != NULL) ? s_allow_sys_sync_cb() : true;
    i2c_rtc_setTime_ex(year, month, day, hour, minute, second, sync_sys);
}

extern "C" RtcDateTime_t i2c_rtc_get(void)
{
    uint32_t now = millis();
    // Rate-limit physical I2C reads of PCF85063 (0x51) to 1 Hz (every 1000 ms)
    if (last_rtc_read_ms == 0 || (now - last_rtc_read_ms >= 1000)) {
        last_rtc_read_ms = (now == 0) ? 1 : now;
        RTC_DateTime datetime = rtc.getDateTime();
        uint16_t y = datetime.getYear();
        uint8_t  m = datetime.getMonth();
        uint8_t  d = datetime.getDay();
        if (m >= 1 && m <= 12 && d >= 1 && d <= 31 && y >= 2020) {
            cached_rtc_time.year   = y;
            cached_rtc_time.month  = m;
            cached_rtc_time.day    = d;
            cached_rtc_time.hour   = datetime.getHour();
            cached_rtc_time.minute = datetime.getMinute();
            cached_rtc_time.second = datetime.getSecond();
            cached_rtc_time.week   = datetime.getWeek();

            // Sync system time if system clock is uninitialized (e.g. at boot)
            time_t sys_now = time(NULL);
            if (sys_now < 946684800) { // Jan 1, 2000
                sync_system_clock(y, m, d, cached_rtc_time.hour, cached_rtc_time.minute, cached_rtc_time.second);
            }
        }
    }
    return cached_rtc_time;
}

extern "C" void get_local_datetime(RtcDateTime_t *local_dt)
{
    if (!local_dt) return;
    RtcDateTime_t t = i2c_rtc_get();
    int16_t utc_off = rtc_bsp_get_utc_offset_minutes();

    int total_minutes = (int)t.hour * 60 + (int)t.minute + (int)utc_off;
    int day_offset = 0;

    if (total_minutes < 0) {
        int days_back = (abs(total_minutes) + 1439) / 1440;
        day_offset = -days_back;
        total_minutes = (total_minutes % 1440) + 1440;
    } else if (total_minutes >= 1440) {
        day_offset = total_minutes / 1440;
        total_minutes = total_minutes % 1440;
    }

    int new_hour  = total_minutes / 60;
    int new_min   = total_minutes % 60;
    int new_year  = t.year;
    int new_month = t.month;
    int new_day   = t.day;

    if (day_offset != 0) {
        new_day += day_offset;
        while (new_day < 1) {
            new_month--;
            if (new_month < 1) {
                new_month = 12;
                new_year--;
            }
            uint8_t dim = daysInMonth[new_month - 1];
            if (new_month == 2 && isLeapYear(new_year)) dim = 29;
            new_day += dim;
        }
        while (true) {
            uint8_t dim = daysInMonth[new_month - 1];
            if (new_month == 2 && isLeapYear(new_year)) dim = 29;
            if (new_day <= dim) break;
            new_day -= dim;
            new_month++;
            if (new_month > 12) {
                new_month = 1;
                new_year++;
            }
        }
    }

    local_dt->year   = (uint16_t)new_year;
    local_dt->month  = (uint8_t)new_month;
    local_dt->day    = (uint8_t)new_day;
    local_dt->hour   = (uint8_t)new_hour;
    local_dt->minute = (uint8_t)new_min;
    local_dt->second = t.second;
    local_dt->week   = t.week;
}

extern "C" void get_formatted_timestamp(char *buffer, size_t max_len, bool filename_safe)
{
    RtcDateTime_t lt;
    get_local_datetime(&lt);

    if (filename_safe) {
        int16_t utc_off = rtc_bsp_get_utc_offset_minutes();
        char sign = (utc_off >= 0) ? '+' : '-';
        int abs_offset = abs((int)utc_off);
        snprintf(buffer, max_len, "%04u-%02u-%02u_%02u-%02u-%02u%c%d",
                 lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, sign, abs_offset);
    } else {
        snprintf(buffer, max_len, "%04u-%02u-%02uT%02u:%02u:%02u",
                 lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second);
    }
}

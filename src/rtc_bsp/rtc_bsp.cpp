#include <stdio.h>
#include <Wire.h>
#include "rtc_bsp.h"
#include "../i2c_bsp/i2c_bsp.h"
#include "../settings_manager/settings_manager.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static SensorPCF85063 rtc;
static const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

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

extern "C" void rtc_init(void)
{
    if (rtc_dev_handle == NULL) {
        Serial.println("RTC I2C Handle not initialized yet!");
        return;
    }

    if (!rtc.begin(rtc_i2c_custom_cb)) {
        Serial.println("Failed to find PCF85063 - check your wiring!");
    } else {
        Serial.println("PCF85063 RTC initialized successfully.");
    }
}

extern "C" void i2c_rtc_setTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second)
{
    rtc.setDateTime(year, month, day, hour, minute, second);
}

extern "C" RtcDateTime_t i2c_rtc_get(void)
{
    RtcDateTime_t time = {2026, 1, 1, 0, 0, 0, 0};
    RTC_DateTime datetime = rtc.getDateTime();
    time.year   = datetime.getYear();
    time.month  = datetime.getMonth();
    time.day    = datetime.getDay();
    time.hour   = datetime.getHour();
    time.minute = datetime.getMinute();
    time.second = datetime.getSecond();
    time.week   = datetime.getWeek();
    if (time.month < 1 || time.month > 12) time.month = 1;
    if (time.day < 1 || time.day > 31) time.day = 1;
    return time;
}

extern "C" void get_local_datetime(RtcDateTime_t *local_dt)
{
    if (!local_dt) return;
    RtcDateTime_t t = i2c_rtc_get();

    int total_minutes = (int)t.hour * 60 + (int)t.minute + (int)current_settings.minutesOffsetToUTC;
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
        char sign = (current_settings.minutesOffsetToUTC >= 0) ? '+' : '-';
        int abs_offset = abs(current_settings.minutesOffsetToUTC);
        snprintf(buffer, max_len, "%04u-%02u-%02u_%02u-%02u-%02u%c%d",
                 lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, sign, abs_offset);
    } else {
        snprintf(buffer, max_len, "%04u-%02u-%02uT%02u:%02u:%02u",
                 lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second);
    }
}

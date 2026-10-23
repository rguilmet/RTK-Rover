#include "settings_manager.h"
#include <ArduinoJson.h>
#include <SD.h>
#include <FFat.h>

SemaphoreHandle_t fs_mux = NULL;

fs::FS& get_active_fs(void) {
    if (sd_card_mounted) {
        return SD;
    }
    return FFat;
}

struct SystemSettings current_settings = {
    -240,             // minutesOffsetToUTC (EDT default)
    "EDT",            // timezone_label
    false,            // use_imperial_units (false = SI m/cm, true = Imperial ft/in)
    1.800f,           // antenna_height_m (pole height from ground tip to antenna plane)
    0.555f,           // antenna_spacing_m (Master to Slave separation, pole at center)
    0.0f,             // baseline_mount_offset_deg
    true,             // apply_lever_arm
    0.0f,             // pole_tilt_tare_pitch_deg
    0.0f,             // pole_tilt_tare_roll_deg
    57600,            // e220_uart_baud (57600 paces UART input so 1400B bursts don't overrun E220 1000B FIFO)
    55,               // e220_channel (CH 55 = 905.125 MHz)
    38.4f,            // e220_air_rate_kbps (2.4, 4.8, 9.6, 19.2, 38.4, 62.5)
    21,               // e220_tx_power_dbm (30, 27, 24, 21)
    128,              // e220_subpacket_bytes (200, 128, 64, 32)
    false,            // e220_lbt_enable
    5,                // survey_avg_seconds (1, 5, or 10s)
    "PIN_",           // default_label_prefix
    "/waypoints.csv", // active_waypoint_file
    85,               // brightness (%)
    30,               // volume (%)
    false,            // is_muted
    "Your_SSID",      // wifi_ssid
    "Your_PASSWORD",  // wifi_password
    false             // enable_serial_logging
};

volatile bool settings_dirty = false;
volatile uint32_t last_settings_change_ms = 0;

extern "C" void mark_settings_dirty(void) {
    settings_dirty = true;
    last_settings_change_ms = millis();
}

extern "C" void init_settings_manager(void) {
    if (fs_mux == NULL) {
        fs_mux = xSemaphoreCreateMutex();
    }
    load_settings();
}

extern "C" void load_settings(void) {
    File file = get_active_fs().open("/settings.json", "r");
    if (!file && sd_card_mounted) {
        file = FFat.open("/settings.json", "r");
        if (file) {
            Serial.println("Loaded settings.json from FFat fallback.");
        }
    }

    if (!file) {
        Serial.println("No settings.json found, saving default settings.json.");
        save_settings();
        return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (!error) {
        if (!doc["minutesOffsetToUTC"].isNull()) current_settings.minutesOffsetToUTC = doc["minutesOffsetToUTC"];
        if (!doc["timezone_label"].isNull()) strncpy(current_settings.timezone_label, doc["timezone_label"], sizeof(current_settings.timezone_label) - 1);
        else if (!doc["timezonelabel"].isNull()) strncpy(current_settings.timezone_label, doc["timezonelabel"], sizeof(current_settings.timezone_label) - 1);

        if (!doc["use_imperial_units"].isNull()) current_settings.use_imperial_units = doc["use_imperial_units"];
        if (!doc["antenna_height_m"].isNull()) current_settings.antenna_height_m = doc["antenna_height_m"];
        if (!doc["antenna_spacing_m"].isNull()) current_settings.antenna_spacing_m = doc["antenna_spacing_m"];
        if (!doc["baseline_mount_offset_deg"].isNull()) current_settings.baseline_mount_offset_deg = doc["baseline_mount_offset_deg"];
        if (!doc["apply_lever_arm"].isNull()) current_settings.apply_lever_arm = doc["apply_lever_arm"];
        if (!doc["pole_tilt_tare_pitch_deg"].isNull()) current_settings.pole_tilt_tare_pitch_deg = doc["pole_tilt_tare_pitch_deg"];
        if (!doc["pole_tilt_tare_roll_deg"].isNull()) current_settings.pole_tilt_tare_roll_deg = doc["pole_tilt_tare_roll_deg"];
        if (!doc["e220_uart_baud"].isNull()) current_settings.e220_uart_baud = doc["e220_uart_baud"];
        if (!doc["e220_channel"].isNull()) current_settings.e220_channel = doc["e220_channel"];
        if (!doc["e220_air_rate_kbps"].isNull()) current_settings.e220_air_rate_kbps = doc["e220_air_rate_kbps"];
        if (!doc["e220_tx_power_dbm"].isNull()) current_settings.e220_tx_power_dbm = doc["e220_tx_power_dbm"];
        if (!doc["e220_subpacket_bytes"].isNull()) current_settings.e220_subpacket_bytes = doc["e220_subpacket_bytes"];
        if (!doc["e220_lbt_enable"].isNull()) current_settings.e220_lbt_enable = doc["e220_lbt_enable"];
        if (!doc["survey_avg_seconds"].isNull()) current_settings.survey_avg_seconds = doc["survey_avg_seconds"];
        if (!doc["default_label_prefix"].isNull()) strncpy(current_settings.default_label_prefix, doc["default_label_prefix"], sizeof(current_settings.default_label_prefix) - 1);
        if (!doc["active_waypoint_file"].isNull()) strncpy(current_settings.active_waypoint_file, doc["active_waypoint_file"], sizeof(current_settings.active_waypoint_file) - 1);
        if (!doc["brightness"].isNull()) current_settings.brightness = doc["brightness"];
        if (!doc["volume"].isNull()) current_settings.volume = doc["volume"];
        if (!doc["is_muted"].isNull()) current_settings.is_muted = doc["is_muted"];
        if (!doc["wifi_ssid"].isNull()) strncpy(current_settings.wifi_ssid, doc["wifi_ssid"], sizeof(current_settings.wifi_ssid) - 1);
        if (!doc["wifi_password"].isNull()) strncpy(current_settings.wifi_password, doc["wifi_password"], sizeof(current_settings.wifi_password) - 1);
        if (!doc["enable_serial_logging"].isNull()) current_settings.enable_serial_logging = doc["enable_serial_logging"];
    }
}

extern "C" void save_settings(void) {
    JsonDocument doc;
    doc["minutesOffsetToUTC"]        = current_settings.minutesOffsetToUTC;
    doc["timezone_label"]            = current_settings.timezone_label;
    doc["use_imperial_units"]        = current_settings.use_imperial_units;
    doc["antenna_height_m"]          = current_settings.antenna_height_m;
    doc["antenna_spacing_m"]         = current_settings.antenna_spacing_m;
    doc["baseline_mount_offset_deg"] = current_settings.baseline_mount_offset_deg;
    doc["apply_lever_arm"]           = current_settings.apply_lever_arm;
    doc["pole_tilt_tare_pitch_deg"]  = current_settings.pole_tilt_tare_pitch_deg;
    doc["pole_tilt_tare_roll_deg"]   = current_settings.pole_tilt_tare_roll_deg;
    doc["e220_uart_baud"]            = current_settings.e220_uart_baud;
    doc["e220_channel"]              = current_settings.e220_channel;
    doc["e220_air_rate_kbps"]        = current_settings.e220_air_rate_kbps;
    doc["e220_tx_power_dbm"]         = current_settings.e220_tx_power_dbm;
    doc["e220_subpacket_bytes"]      = current_settings.e220_subpacket_bytes;
    doc["e220_lbt_enable"]           = current_settings.e220_lbt_enable;
    doc["survey_avg_seconds"]        = current_settings.survey_avg_seconds;
    doc["default_label_prefix"]      = current_settings.default_label_prefix;
    doc["active_waypoint_file"]      = current_settings.active_waypoint_file;
    doc["brightness"]                = current_settings.brightness;
    doc["volume"]                    = current_settings.volume;
    doc["is_muted"]                  = current_settings.is_muted;
    doc["wifi_ssid"]                 = current_settings.wifi_ssid;
    doc["wifi_password"]             = current_settings.wifi_password;
    doc["enable_serial_logging"]     = current_settings.enable_serial_logging;

    String output;
    serializeJsonPretty(doc, output);

    File file = get_active_fs().open("/settings.json", "w");
    if (file) {
        file.print(output);
        file.close();
    }

    if (sd_card_mounted) {
        File ffat_file = FFat.open("/settings.json", "w");
        if (ffat_file) {
            ffat_file.print(output);
            ffat_file.close();
        }
        Serial.println("Settings saved to SD Card & FFat mirror.");
    } else {
        Serial.println("Settings saved to FFat.");
    }
}

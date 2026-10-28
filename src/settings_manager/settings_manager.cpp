#include "settings_manager.h"
#include <ArduinoJson.h>
#include <SD_MMC.h>
#include <SD.h>
#include <FFat.h>
#include <string.h>

extern bool sd_card_use_spi;
SemaphoreHandle_t fs_mux = NULL;

fs::FS& get_active_fs(void) {
    if (sd_card_mounted) {
        if (sd_card_use_spi) {
            return SD;
        }
        return SD_MMC;
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
    62.5f,            // e220_air_rate_kbps (2.4, 4.8, 9.6, 19.2, 38.4, 62.5)
    21,               // e220_tx_power_dbm (30, 27, 24, 21)
    128,              // e220_subpacket_bytes (200, 128, 64, 32)
    false,            // e220_lbt_enable
    5,                // survey_avg_seconds (1, 5, or 10s)
    "PIN_",           // default_label_prefix
    "/waypoints.csv", // active_waypoint_file (default)
    0,                // active_waypoint_index (default: 0)
    80,               // brightness (%)
    40,               // volume (%)
    false,            // is_muted
    "Your_SSID",      // wifi_ssid
    "Your_PASSWORD",  // wifi_password
    true,             // enable_serial_logging (master switch for serial monitor output)
    false,            // enable_imu_logging (periodic 1 Hz IMU pose and raw accelerometer logs)
    true,             // enable_telemetry_logging (periodic 5s [TELEM] diagnostic heartbeat)
    true              // enable_touch_logging (log touch events and coordinates to Serial Monitor)
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

// -----------------------------------------------------------------------------
// Internal Safeguards & Helpers
// -----------------------------------------------------------------------------

static bool is_placeholder_ssid(const char *ssid) {
    if (!ssid || ssid[0] == '\0') return true;
    if (strcasecmp(ssid, "Your_SSID") == 0) return true;
    if (strcasecmp(ssid, "Your-SSID-Here") == 0) return true;
    if (strcasecmp(ssid, "Your-SSID") == 0) return true;
    if (strcasecmp(ssid, "SSID") == 0) return true;
    return false;
}

static void backup_file(fs::FS &fs, const char *src_path, const char *bak_path) {
    if (!fs.exists(src_path)) return;
    File src = fs.open(src_path, "r");
    if (!src) return;
    if (src.size() == 0) {
        src.close();
        return;
    }
    File dst = fs.open(bak_path, "w");
    if (!dst) {
        src.close();
        return;
    }
    uint8_t buf[256];
    while (src.available()) {
        size_t n = src.read(buf, sizeof(buf));
        if (n > 0) dst.write(buf, n);
    }
    dst.flush();
    dst.close();
    src.close();
}

static bool read_wifi_credentials_from_file(fs::FS &fs, const char *path, char *ssid_out, char *pass_out) {
    if (!fs.exists(path)) return false;
    File f = fs.open(path, "r");
    if (!f || f.size() == 0) {
        if (f) f.close();
        return false;
    }
    JsonDocument d;
    DeserializationError err = deserializeJson(d, f);
    f.close();
    if (err) return false;

    const char *raw_ssid = NULL;
    if (!d["wifi_ssid"].isNull()) raw_ssid = d["wifi_ssid"];
    else if (!d["ssid"].isNull()) raw_ssid = d["ssid"];
    else if (!d["WIFI_SSID"].isNull()) raw_ssid = d["WIFI_SSID"];
    else if (!d["SSID"].isNull()) raw_ssid = d["SSID"];

    if (!raw_ssid) return false;
    while (*raw_ssid == ' ' || *raw_ssid == '\t' || *raw_ssid == '\r' || *raw_ssid == '\n') raw_ssid++;
    if (is_placeholder_ssid(raw_ssid)) return false;

    strncpy(ssid_out, raw_ssid, 32);
    ssid_out[32] = '\0';
    int len = strlen(ssid_out);
    while (len > 0 && (ssid_out[len - 1] == ' ' || ssid_out[len - 1] == '\r' || ssid_out[len - 1] == '\n' || ssid_out[len - 1] == '\t')) {
        ssid_out[--len] = '\0';
    }

    const char *raw_pass = NULL;
    if (!d["wifi_password"].isNull()) raw_pass = d["wifi_password"];
    else if (!d["password"].isNull()) raw_pass = d["password"];
    else if (!d["wifi_pass"].isNull()) raw_pass = d["wifi_pass"];
    else if (!d["WIFI_PASSWORD"].isNull()) raw_pass = d["WIFI_PASSWORD"];
    else if (!d["PASSWORD"].isNull()) raw_pass = d["PASSWORD"];

    if (raw_pass) {
        while (*raw_pass == ' ' || *raw_pass == '\t' || *raw_pass == '\r' || *raw_pass == '\n') raw_pass++;
        strncpy(pass_out, raw_pass, 64);
        pass_out[64] = '\0';
        int plen = strlen(pass_out);
        while (plen > 0 && (pass_out[plen - 1] == ' ' || pass_out[plen - 1] == '\r' || pass_out[plen - 1] == '\n' || pass_out[plen - 1] == '\t')) {
            pass_out[--plen] = '\0';
        }
    } else {
        pass_out[0] = '\0';
    }

    return true;
}

// -----------------------------------------------------------------------------
// Load Settings
// -----------------------------------------------------------------------------

extern "C" void load_settings(void) {
    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);

    File file = get_active_fs().open("/settings.json", "r");
    if (!file && sd_card_mounted) {
        // SD card might have just mounted; brief retry
        delay(50);
        file = get_active_fs().open("/settings.json", "r");
    }

    // Try .bak fallback on active FS if primary failed
    if (!file) {
        file = get_active_fs().open("/settings.json.bak", "r");
        if (file) {
            Serial.println("[SETTINGS] Primary /settings.json unavailable; loaded /settings.json.bak from active storage.");
        }
    }

    // Fall back to FFat internal flash mirror
    if (!file && sd_card_mounted) {
        file = FFat.open("/settings.json", "r");
        if (file) {
            Serial.println("[SETTINGS] Loaded /settings.json from FFat internal mirror.");
        } else {
            file = FFat.open("/settings.json.bak", "r");
            if (file) {
                Serial.println("[SETTINGS] Loaded /settings.json.bak from FFat internal mirror.");
            }
        }
    }

    // CRITICAL SAFEGUARD: Never overwrite disk with default settings on read failure!
    if (!file) {
        Serial.println("[SETTINGS] Notice: No /settings.json found on SD or FFat; using memory defaults without modifying storage.");
        if (fs_mux) xSemaphoreGive(fs_mux);
        return;
    }

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
        Serial.printf("[SETTINGS] Deserialization error in /settings.json: %s\n", error.c_str());
        // Attempt recovery from .bak
        File bak_file = get_active_fs().open("/settings.json.bak", "r");
        if (!bak_file && sd_card_mounted) {
            bak_file = FFat.open("/settings.json.bak", "r");
        }
        if (bak_file) {
            error = deserializeJson(doc, bak_file);
            bak_file.close();
            if (!error) {
                Serial.println("[SETTINGS] Recovered valid settings from .bak file!");
            }
        }
        if (error) {
            Serial.println("[SETTINGS] Notice: Parse and backup recovery failed; using memory defaults without modifying storage.");
            if (fs_mux) xSemaphoreGive(fs_mux);
            return;
        }
    }

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
    if (!doc["active_waypoint_index"].isNull()) current_settings.active_waypoint_index = doc["active_waypoint_index"];
    if (!doc["brightness"].isNull()) current_settings.brightness = doc["brightness"];
    if (!doc["volume"].isNull()) current_settings.volume = doc["volume"];
    if (!doc["is_muted"].isNull()) current_settings.is_muted = doc["is_muted"];

    // Robust WiFi SSID loading with trim and key aliases
    const char *raw_ssid = NULL;
    if (!doc["wifi_ssid"].isNull()) raw_ssid = doc["wifi_ssid"];
    else if (!doc["ssid"].isNull()) raw_ssid = doc["ssid"];
    else if (!doc["WIFI_SSID"].isNull()) raw_ssid = doc["WIFI_SSID"];
    else if (!doc["SSID"].isNull()) raw_ssid = doc["SSID"];

    if (raw_ssid) {
        while (*raw_ssid == ' ' || *raw_ssid == '\t' || *raw_ssid == '\r' || *raw_ssid == '\n') raw_ssid++;
        strncpy(current_settings.wifi_ssid, raw_ssid, sizeof(current_settings.wifi_ssid) - 1);
        current_settings.wifi_ssid[sizeof(current_settings.wifi_ssid) - 1] = '\0';
        int len = strlen(current_settings.wifi_ssid);
        while (len > 0 && (current_settings.wifi_ssid[len - 1] == ' ' || current_settings.wifi_ssid[len - 1] == '\r' || current_settings.wifi_ssid[len - 1] == '\n' || current_settings.wifi_ssid[len - 1] == '\t')) {
            current_settings.wifi_ssid[--len] = '\0';
        }
    }

    // Robust WiFi password loading with trim and key aliases
    const char *raw_pass = NULL;
    if (!doc["wifi_password"].isNull()) raw_pass = doc["wifi_password"];
    else if (!doc["password"].isNull()) raw_pass = doc["password"];
    else if (!doc["wifi_pass"].isNull()) raw_pass = doc["wifi_pass"];
    else if (!doc["WIFI_PASSWORD"].isNull()) raw_pass = doc["WIFI_PASSWORD"];
    else if (!doc["PASSWORD"].isNull()) raw_pass = doc["PASSWORD"];

    if (raw_pass) {
        while (*raw_pass == ' ' || *raw_pass == '\t' || *raw_pass == '\r' || *raw_pass == '\n') raw_pass++;
        strncpy(current_settings.wifi_password, raw_pass, sizeof(current_settings.wifi_password) - 1);
        current_settings.wifi_password[sizeof(current_settings.wifi_password) - 1] = '\0';
        int len = strlen(current_settings.wifi_password);
        while (len > 0 && (current_settings.wifi_password[len - 1] == ' ' || current_settings.wifi_password[len - 1] == '\r' || current_settings.wifi_password[len - 1] == '\n' || current_settings.wifi_password[len - 1] == '\t')) {
            current_settings.wifi_password[--len] = '\0';
        }
    }

    // If loaded SSID is a placeholder, check if any backup has valid real credentials
    if (is_placeholder_ssid(current_settings.wifi_ssid)) {
        char disk_ssid[33] = {0};
        char disk_pass[65] = {0};
        if (read_wifi_credentials_from_file(get_active_fs(), "/settings.json.bak", disk_ssid, disk_pass) ||
            read_wifi_credentials_from_file(FFat, "/settings.json", disk_ssid, disk_pass) ||
            read_wifi_credentials_from_file(FFat, "/settings.json.bak", disk_ssid, disk_pass)) {
            if (!is_placeholder_ssid(disk_ssid)) {
                strncpy(current_settings.wifi_ssid, disk_ssid, sizeof(current_settings.wifi_ssid) - 1);
                current_settings.wifi_ssid[sizeof(current_settings.wifi_ssid) - 1] = '\0';
                strncpy(current_settings.wifi_password, disk_pass, sizeof(current_settings.wifi_password) - 1);
                current_settings.wifi_password[sizeof(current_settings.wifi_password) - 1] = '\0';
                Serial.printf("[SETTINGS] Automatically recovered valid Wi-Fi credentials from backup: SSID='%s'\n", current_settings.wifi_ssid);
            }
        }
    }

    Serial.printf("[SETTINGS] WiFi Config: SSID='%s' (len=%d), Pass len=%d\n",
                  current_settings.wifi_ssid,
                  (int)strlen(current_settings.wifi_ssid),
                  (int)strlen(current_settings.wifi_password));

    if (!doc["enable_serial_logging"].isNull()) current_settings.enable_serial_logging = doc["enable_serial_logging"];
    if (!doc["enable_imu_logging"].isNull()) current_settings.enable_imu_logging = doc["enable_imu_logging"];
    else if (!doc["log_imu_periodic"].isNull()) current_settings.enable_imu_logging = doc["log_imu_periodic"];
    if (!doc["enable_telemetry_logging"].isNull()) current_settings.enable_telemetry_logging = doc["enable_telemetry_logging"];
    else if (!doc["log_telemetry_periodic"].isNull()) current_settings.enable_telemetry_logging = doc["log_telemetry_periodic"];
    else if (!doc["enable_telem_logging"].isNull()) current_settings.enable_telemetry_logging = doc["enable_telem_logging"];
    if (!doc["enable_touch_logging"].isNull()) current_settings.enable_touch_logging = doc["enable_touch_logging"];
    else if (!doc["log_touch_events"].isNull()) current_settings.enable_touch_logging = doc["log_touch_events"];

    // If SD card is mounted, ensure FFat internal flash is an exact mirror of the SD Card settings
    if (sd_card_mounted) {
        backup_file(FFat, "/settings.json", "/settings.json.bak");
        File ffat_file = FFat.open("/settings.json", "w");
        if (ffat_file) {
            serializeJsonPretty(doc, ffat_file);
            ffat_file.flush();
            ffat_file.close();
            Serial.println("[SETTINGS] Mirrored SD Card settings.json to internal FFat.");
        }
    }

    if (fs_mux) xSemaphoreGive(fs_mux);
}

// -----------------------------------------------------------------------------
// Save Settings
// -----------------------------------------------------------------------------

extern "C" void save_settings(void) {
    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);

    // Safeguard 1: Never overwrite real disk Wi-Fi credentials with placeholder values
    if (is_placeholder_ssid(current_settings.wifi_ssid)) {
        char disk_ssid[33] = {0};
        char disk_pass[65] = {0};
        if (read_wifi_credentials_from_file(get_active_fs(), "/settings.json", disk_ssid, disk_pass) ||
            read_wifi_credentials_from_file(get_active_fs(), "/settings.json.bak", disk_ssid, disk_pass) ||
            read_wifi_credentials_from_file(FFat, "/settings.json", disk_ssid, disk_pass) ||
            read_wifi_credentials_from_file(FFat, "/settings.json.bak", disk_ssid, disk_pass)) {
            if (!is_placeholder_ssid(disk_ssid)) {
                strncpy(current_settings.wifi_ssid, disk_ssid, sizeof(current_settings.wifi_ssid) - 1);
                current_settings.wifi_ssid[sizeof(current_settings.wifi_ssid) - 1] = '\0';
                strncpy(current_settings.wifi_password, disk_pass, sizeof(current_settings.wifi_password) - 1);
                current_settings.wifi_password[sizeof(current_settings.wifi_password) - 1] = '\0';
                Serial.printf("[SETTINGS] Preserved existing Wi-Fi credentials: SSID='%s'\n", current_settings.wifi_ssid);
            }
        }
    }

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
    doc["active_waypoint_index"]     = current_settings.active_waypoint_index;
    doc["brightness"]                = current_settings.brightness;
    doc["volume"]                    = current_settings.volume;
    doc["is_muted"]                  = current_settings.is_muted;
    doc["wifi_ssid"]                 = current_settings.wifi_ssid;
    doc["wifi_password"]             = current_settings.wifi_password;
    doc["enable_serial_logging"]     = current_settings.enable_serial_logging;
    doc["enable_imu_logging"]        = current_settings.enable_imu_logging;
    doc["enable_telemetry_logging"]  = current_settings.enable_telemetry_logging;
    doc["enable_touch_logging"]      = current_settings.enable_touch_logging;

    String output;
    serializeJsonPretty(doc, output);

    // Safeguard 2: Backup existing active file before writing
    backup_file(get_active_fs(), "/settings.json", "/settings.json.bak");

    File file = get_active_fs().open("/settings.json", "w");
    if (file) {
        file.print(output);
        file.flush();
        file.close();
    }

    if (sd_card_mounted) {
        backup_file(FFat, "/settings.json", "/settings.json.bak");
        File ffat_file = FFat.open("/settings.json", "w");
        if (ffat_file) {
            ffat_file.print(output);
            ffat_file.flush();
            ffat_file.close();
        }
        Serial.println("[SETTINGS] Settings saved to SD Card & FFat mirror (with .bak backups).");
    } else {
        Serial.println("[SETTINGS] Settings saved to FFat (with .bak backup).");
    }

    if (fs_mux) xSemaphoreGive(fs_mux);
}

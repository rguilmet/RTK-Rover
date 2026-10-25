#include "web_server.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include "../waypoint_manager/waypoint_manager.h"
#include <WiFi.h>
#include <WebServer.h>
#include <SD_MMC.h>
#include <SD.h>
#include <FFat.h>
#include <dirent.h>
#include <sys/stat.h>

extern bool sd_card_mounted;
extern bool sd_card_use_spi;

static bool wifi_active = false;
static char s_wifi_ip[32] = {0};
static WebServer server(80);

extern "C" void update_wifi_ip_label(const char* ip);

extern "C" bool is_wifi_active(void) {
    return wifi_active;
}

extern "C" const char* get_wifi_ip(void) {
    return s_wifi_ip;
}

static fs::FS* get_target_fs(const String& fs_param) {
    if (fs_param == "sd" && sd_card_mounted) {
        if (sd_card_use_spi) return &SD;
        return &SD_MMC;
    }
    return &FFat;
}

static void list_directory_recursive(String& out, const char* fs_param, const char* vfs_base, const char* rel_sub) {
    String full_dir = String(vfs_base) + String(rel_sub);
    DIR *dir = opendir(full_dir.c_str());
    if (!dir) return;

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        String filename = String(ent->d_name);
        if (filename == "." || filename == "..") continue;

        String item_vfs = full_dir + "/" + filename;
        String item_rel = String(rel_sub) + "/" + filename;
        if (!item_rel.startsWith("/")) item_rel = "/" + item_rel;

        struct stat st;
        if (stat(item_vfs.c_str(), &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                // Recurse into subdirectories (e.g. /routes, /nmea)
                list_directory_recursive(out, fs_param, vfs_base, item_rel.c_str());
            } else if (S_ISREG(st.st_mode)) {
                size_t size = st.st_size;
                String link_attrs;
                String endpoint = "/download";
                if (filename.endsWith(".bmp")) {
                    link_attrs = "target=\"_blank\"";
                    endpoint = "/view";
                } else {
                    link_attrs = "download=\"" + filename + "\"";
                }
                out += "<li><a href=\"" + endpoint + "?fs=" + String(fs_param) + "&file=" + item_rel + "\" " + link_attrs + ">" + item_rel + "</a> (" + String(size) + " bytes) - <a href=\"/delete?fs=" + String(fs_param) + "&file=" + item_rel + "\" onclick=\"return confirm('Delete " + filename + "?');\">[Delete]</a></li>";
            }
        }
    }
    closedir(dir);
}

static void handleRoot() {
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><title>RTK Rover Portal " + String(ROVER_FIRMWARE_VERSION) + "</title><style>body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;margin:20px;background:#12181b;color:#e0e6ed;} h1{color:#00ffaa;} h2{color:#00d8ff;border-bottom:1px solid #334;padding-bottom:5px;} a{color:#38bdf8;text-decoration:none;} a:hover{text-decoration:underline;} li{margin:8px 0;font-family:monospace;font-size:14px;} .up{margin-bottom:20px;padding:15px;background:#1e293b;border-radius:8px;} input[type=submit],button{background:#00ffaa;color:#000;font-weight:bold;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;} .badge{background:#0284c7;padding:3px 8px;border-radius:4px;font-size:12px;}</style></head><body><h1>RTK Rover Portal " + String(ROVER_FIRMWARE_VERSION) + "</h1>";
    
    auto upload_html = [](const char* fs_param, const char* title) -> String {
        return "<div class='up'><h3>Upload to " + String(title) + "</h3><form method='POST' action='/upload?fs=" + String(fs_param) + "' enctype='multipart/form-data'><input type='file' name='f' style='color:#fff'> <input type='submit' value='Upload'></form></div>";
    };

    auto generate_dir_html = [](const char* title, const char* fs_param, const char* vfs_path) -> String {
        String out = "<h2>" + String(title) + "</h2><ul>";
        String list_items;
        list_directory_recursive(list_items, fs_param, vfs_path, "");
        if (list_items.length() == 0) {
            out += "<li>No files found.</li>";
        } else {
            out += list_items;
        }
        out += "</ul>";
        return out;
    };

    if (sd_card_mounted) {
        html += upload_html("sd", "MicroSD Card");
        html += generate_dir_html("MicroSD Card (/sdcard)", "sd", "/sdcard");
    }
    
    html += upload_html("ffat", "Internal Flash (FFat)");
    html += generate_dir_html("Internal Flash (/ffat)", "ffat", "/ffat");
    
    html += "</body></html>";
    server.send(200, "text/html", html);
}

static void handleSetTime() {
    if (server.hasArg("y") && server.hasArg("m") && server.hasArg("d") && server.hasArg("h") && server.hasArg("min") && server.hasArg("s")) {
        uint16_t y = server.arg("y").toInt();
        uint8_t m = server.arg("m").toInt();
        uint8_t d = server.arg("d").toInt();
        uint8_t h = server.arg("h").toInt();
        uint8_t min = server.arg("min").toInt();
        uint8_t s = server.arg("s").toInt();
        i2c_rtc_setTime(y, m, d, h, min, s);
    }
    server.send(200, "text/plain", "OK");
}

static void handleDownload() {
    if (!server.hasArg("file")) {
        server.send(400, "text/plain", "Missing file parameter");
        return;
    }
    String path = server.arg("file");
    String fs_type = server.hasArg("fs") ? server.arg("fs") : "ffat";
    fs::FS* target_fs = get_target_fs(fs_type);

    if (!path.startsWith("/")) path = "/" + path;
    File file = target_fs->open(path, "r");
    if (!file) {
        server.send(404, "text/plain", "File not found: " + path);
        return;
    }
    
    int slashIndex = path.lastIndexOf('/');
    String filename = (slashIndex >= 0) ? path.substring(slashIndex + 1) : path;
    
    String mimeType = "application/octet-stream";
    String disposition = "attachment";
    
    if (filename.endsWith(".csv")) {
        mimeType = "text/csv";
    } else if (filename.endsWith(".bmp")) {
        mimeType = "image/bmp";
        disposition = "inline";
    } else if (filename.endsWith(".json")) {
        mimeType = "application/json";
    } else if (filename.endsWith(".log") || filename.endsWith(".txt")) {
        mimeType = "text/plain";
    }
    
    server.sendHeader("Content-Disposition", disposition + "; filename=\"" + filename + "\"");
    server.sendHeader("Content-Length", String(file.size()));
    server.sendHeader("Connection", "close");
    server.streamFile(file, mimeType);
    file.close();
}

static void handleView() {
    if (!server.hasArg("file")) {
        server.send(400, "text/plain", "Missing file parameter");
        return;
    }
    String path = server.arg("file");
    String fs_type = server.hasArg("fs") ? server.arg("fs") : "ffat";
    
    int slashIndex = path.lastIndexOf('/');
    String filename = (slashIndex >= 0) ? path.substring(slashIndex + 1) : path;
    
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'><title>View Image</title></head><body style='background:#12181b;color:white;text-align:center;font-family:sans-serif;padding:20px;'>";
    html += "<h2 style='color:#00ffaa;'>" + filename + "</h2>";
    html += "<button onclick='dl()' style='padding:12px 24px;font-size:16px;background:#00d8ff;color:#000;font-weight:bold;border:none;border-radius:6px;cursor:pointer;'>Save Securely as PNG</button>";
    html += "<p style='color:#94a3b8;font-size:13px;margin-top:10px;margin-bottom:20px;'>Converts internal BMP to PNG via client browser to bypass HTTP download warnings.</p>";
    html += "<img id='img' src='/download?fs=" + fs_type + "&file=" + path + "' style='max-width:100%; border:2px solid #334; border-radius:8px;'><br><br>";
    html += "<script>function dl(){";
    html += "var img = document.getElementById('img');";
    html += "var c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;";
    html += "c.getContext('2d').drawImage(img, 0, 0);";
    html += "var a = document.createElement('a'); a.download = '" + filename.substring(0, filename.length()-4) + ".png';";
    html += "a.href = c.toDataURL('image/png'); a.click();";
    html += "}</script></body></html>";
    
    server.send(200, "text/html", html);
}

static void handleDelete() {
    if (!server.hasArg("file")) {
        server.send(400, "text/plain", "Missing file parameter");
        return;
    }
    String path = server.arg("file");
    String fs_type = server.hasArg("fs") ? server.arg("fs") : "ffat";
    fs::FS* target_fs = get_target_fs(fs_type);

    if (!path.startsWith("/")) path = "/" + path;
    target_fs->remove(path);
    server.sendHeader("Location", "/");
    server.send(303);
}

static File uploadFile;
static void handleUpload() {
    HTTPUpload& upload = server.upload();
    String fs_type = server.hasArg("fs") ? server.arg("fs") : "ffat";
    fs::FS* target_fs = get_target_fs(fs_type);

    if (upload.status == UPLOAD_FILE_START) {
        String filename = upload.filename;
        if (!filename.startsWith("/")) filename = "/" + filename;
        uploadFile = target_fs->open(filename, "w");
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) uploadFile.close();
        if (upload.filename.endsWith("waypoints.csv")) {
            waypoint_load_csv(current_settings.active_waypoint_file);
            Serial.println("[WEB] Reloaded waypoints.csv after upload!");
        } else if (upload.filename.endsWith("settings.json")) {
            load_settings();
            Serial.println("[WEB] Reloaded settings.json after upload!");
        }
    }
}

static void setup_web_routes(void) {
    server.on("/", handleRoot);
    server.on("/set_time", handleSetTime);
    server.on("/download", handleDownload);
    server.on("/view", handleView);
    server.on("/delete", handleDelete);
    server.on("/upload", HTTP_POST, []() {
        server.sendHeader("Location", "/");
        server.send(303);
    }, handleUpload);
}

extern "C" void start_wifi(void) {
    if (wifi_active) return;
    
    update_wifi_ip_label("Connecting...");
    Serial.printf("[WIFI] Attempting connection to SSID: '%s' (configured in settings.json)...\n", current_settings.wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(current_settings.wifi_ssid, current_settings.wifi_password);
    
    // Non-blocking connection task so UI stays responsive
    xTaskCreatePinnedToCore([](void* arg){
        int attempts = 0;
        // Wait up to 10 seconds for STA connection
        while(WiFi.status() != WL_CONNECTED && attempts < 20) {
            vTaskDelay(pdMS_TO_TICKS(500));
            attempts++;
        }
        if (WiFi.status() == WL_CONNECTED) {
            wifi_active = true;
            String ip = WiFi.localIP().toString();
            strncpy(s_wifi_ip, ip.c_str(), sizeof(s_wifi_ip) - 1);
            update_wifi_ip_label(s_wifi_ip);
            
            // Auto-sync NTP to RTC
            configTime(0, 0, "pool.ntp.org");
            struct tm timeinfo;
            if (getLocalTime(&timeinfo, 10000)) {
                i2c_rtc_setTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                Serial.println("[WIFI] RTC synced via NTP!");
            }
            
            setup_web_routes();
            server.begin();
            Serial.printf("[WIFI] Connected to %s! Web Portal: http://%s/\n", current_settings.wifi_ssid, s_wifi_ip);
        } else {
            // STA failed or unconfigured in field: Launch local SoftAP hotspot fallback
            Serial.printf("[WIFI] STA connection to '%s' failed. Starting SoftAP 'RTK-Rover' fallback...\n", current_settings.wifi_ssid);
            update_wifi_ip_label("Starting AP...");
            WiFi.disconnect();
            WiFi.mode(WIFI_AP);
            WiFi.softAP("RTK-Rover", "12345678");
            vTaskDelay(pdMS_TO_TICKS(150));
            
            IPAddress apIP = WiFi.softAPIP();
            String ip = apIP.toString();
            strncpy(s_wifi_ip, ip.c_str(), sizeof(s_wifi_ip) - 1);
            wifi_active = true;
            update_wifi_ip_label(s_wifi_ip);
            
            setup_web_routes();
            server.begin();
            Serial.printf("[WIFI] SoftAP Active! Connect phone to SSID 'RTK-Rover' (pwd: 12345678) -> http://%s/\n", s_wifi_ip);
        }
        vTaskDelete(NULL);
    }, "wifi_connect", 4096, NULL, 1, NULL, 0);
}

extern "C" void stop_wifi(void) {
    if (!wifi_active) return;
    server.stop();
    WiFi.disconnect(true);
    wifi_active = false;
    s_wifi_ip[0] = '\0';
    update_wifi_ip_label("Wi-Fi Off");
    Serial.println("[WIFI] Web Portal Stopped.");
}

extern "C" void handle_wifi_server(void) {
    if (wifi_active) {
        server.handleClient();
    }
}

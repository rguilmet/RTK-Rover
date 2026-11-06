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
#include <vector>
#include <algorithm>
#include <ctype.h>
#include <time.h>

extern bool sd_card_mounted;
extern bool sd_card_use_spi;

static bool wifi_active = false;
static bool s_wifi_starting = false;
static char s_wifi_ip[32] = {0};
static WebServer server(80);

extern "C" void update_wifi_ip_label(const char* ip);

extern "C" bool is_wifi_active(void) {
    return wifi_active;
}

extern "C" bool is_wifi_softap(void) {
    return wifi_active && ((WiFi.getMode() & WIFI_AP) != 0);
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

struct WebFileEntry {
    String filename;
    String item_rel;
    size_t size;
    time_t mtime;
    String dt_str;
    bool is_bmp;
};

// Extract YYYY-MM-DD_HH-MM-SS from filename (e.g. nema-log_2026-10-29_09-34-27-240.txt)
static bool parse_filename_datetime(const String& name, time_t& out_time, String& out_str) {
    const char* str = name.c_str();
    int len = name.length();
    for (int i = 0; i <= len - 19; i++) {
        if (isdigit((unsigned char)str[i]) && isdigit((unsigned char)str[i+1]) &&
            isdigit((unsigned char)str[i+2]) && isdigit((unsigned char)str[i+3]) &&
            str[i+4] == '-' &&
            isdigit((unsigned char)str[i+5]) && isdigit((unsigned char)str[i+6]) &&
            str[i+7] == '-' &&
            isdigit((unsigned char)str[i+8]) && isdigit((unsigned char)str[i+9]) &&
            str[i+10] == '_' &&
            isdigit((unsigned char)str[i+11]) && isdigit((unsigned char)str[i+12]) &&
            str[i+13] == '-' &&
            isdigit((unsigned char)str[i+14]) && isdigit((unsigned char)str[i+15]) &&
            str[i+16] == '-' &&
            isdigit((unsigned char)str[i+17]) && isdigit((unsigned char)str[i+18])) {
            
            int y   = atoi(name.substring(i, i+4).c_str());
            int m   = atoi(name.substring(i+5, i+7).c_str());
            int d   = atoi(name.substring(i+8, i+10).c_str());
            int h   = atoi(name.substring(i+11, i+13).c_str());
            int min = atoi(name.substring(i+14, i+16).c_str());
            int s   = atoi(name.substring(i+17, i+19).c_str());

            if (y >= 2020 && m >= 1 && m <= 12 && d >= 1 && d <= 31 &&
                h >= 0 && h <= 23 && min >= 0 && min <= 59 && s >= 0 && s <= 59) {
                struct tm tm_buf;
                memset(&tm_buf, 0, sizeof(tm_buf));
                tm_buf.tm_year = y - 1900;
                tm_buf.tm_mon  = m - 1;
                tm_buf.tm_mday = d;
                tm_buf.tm_hour = h;
                tm_buf.tm_min  = min;
                tm_buf.tm_sec  = s;
                out_time = mktime(&tm_buf);

                char buf[32];
                snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", y, m, d, h, min, s);
                out_str = String(buf);
                return true;
            }
        }
    }
    return false;
}

static String format_file_size(size_t bytes) {
    if (bytes < 1024) {
        return String(bytes) + " B";
    } else if (bytes < 1024 * 1024) {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.1f KB", (float)bytes / 1024.0f);
        return String(buf);
    } else {
        char buf[32];
        snprintf(buf, sizeof(buf), "%.2f MB", (float)bytes / (1024.0f * 1024.0f));
        return String(buf);
    }
}

static void collect_files_recursive(std::vector<WebFileEntry>& files, const char* vfs_base, const char* rel_sub) {
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
                collect_files_recursive(files, vfs_base, item_rel.c_str());
            } else if (S_ISREG(st.st_mode)) {
                WebFileEntry entry;
                entry.filename = filename;
                entry.item_rel = item_rel;
                entry.size = st.st_size;
                entry.is_bmp = filename.endsWith(".bmp");

                time_t parsed_time = 0;
                String parsed_dt = "";
                if (parse_filename_datetime(filename, parsed_time, parsed_dt)) {
                    entry.mtime = parsed_time;
                    entry.dt_str = parsed_dt;
                } else if (st.st_mtime > 946684800) {
                    entry.mtime = st.st_mtime;
                    struct tm *tm_info = localtime(&st.st_mtime);
                    char buf[32];
                    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_info);
                    entry.dt_str = String(buf);
                } else {
                    entry.mtime = 0;
                    entry.dt_str = "-";
                }

                files.push_back(entry);
            }
        }
    }
    closedir(dir);
}

static void sort_web_files(std::vector<WebFileEntry>& files) {
    // Primary: newest date/time first (descending)
    // Secondary: alphabetical descending by path
    std::sort(files.begin(), files.end(), [](const WebFileEntry& a, const WebFileEntry& b) {
        if (a.mtime != b.mtime) {
            return a.mtime > b.mtime;
        }
        return a.item_rel > b.item_rel;
    });
}

static String generate_dir_table_html(const char* title, const char* fs_param, const char* vfs_path, const char* table_id) {
    std::vector<WebFileEntry> files;
    collect_files_recursive(files, vfs_path, "");
    sort_web_files(files);

    String out;
    out.reserve(files.size() * 260 + 512);

    out += "<h2>" + String(title) + " <span class='badge'>" + String(files.size()) + " files</span></h2>";

    if (files.empty()) {
        out += "<p style='color:#94a3b8;font-style:italic;margin:10px 0;'>No files found in storage.</p>";
        return out;
    }

    out += "<div class='sort-bar'>";
    out += "<span style='color:#94a3b8;font-size:12px;margin-right:4px;'>Quick Sort:</span>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 1, true, 'desc')\">Newest First</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 1, true, 'asc')\">Oldest First</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 0, false, 'asc')\">Name (A-Z)</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 0, false, 'desc')\">Name (Z-A)</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 2, true, 'desc')\">Largest First</button>";
    out += "</div>";

    out += "<div class='tbl-wrap'><table id='" + String(table_id) + "'>";
    out += "<thead><tr>";
    out += "<th onclick=\"toggleSort('" + String(table_id) + "', 0, false)\" title=\"Click to sort by Filename\">File Name / Path <span class='sort-icon' id='" + String(table_id) + "_i0'></span></th>";
    out += "<th onclick=\"toggleSort('" + String(table_id) + "', 1, true)\" title=\"Click to sort by Date & Time\">Date &amp; Time <span class='sort-icon' id='" + String(table_id) + "_i1'> [v]</span></th>";
    out += "<th onclick=\"toggleSort('" + String(table_id) + "', 2, true)\" title=\"Click to sort by Size\">Size <span class='sort-icon' id='" + String(table_id) + "_i2'></span></th>";
    out += "<th>Actions</th>";
    out += "</tr></thead><tbody>";

    for (const auto& f : files) {
        out += "<tr>";
        out += "<td data-val=\"" + f.item_rel + "\">";
        if (f.is_bmp) {
            out += "<a href=\"/view?fs=" + String(fs_param) + "&file=" + f.item_rel + "\" target=\"_blank\">" + f.item_rel + "</a>";
        } else {
            out += "<a href=\"#\" onclick=\"dlFile('/download?fs=" + String(fs_param) + "&file=" + f.item_rel + "', '" + f.filename + "'); return false;\" title=\"Download securely via in-memory Blob\">" + f.item_rel + "</a>";
        }
        out += "</td>";

        out += "<td data-val=\"" + String((uint32_t)f.mtime) + "\">" + f.dt_str + "</td>";

        out += "<td data-val=\"" + String(f.size) + "\" title=\"" + String(f.size) + " bytes\">" + format_file_size(f.size) + "</td>";

        out += "<td class='actions'>";
        if (f.is_bmp) {
            out += "<a href=\"/view?fs=" + String(fs_param) + "&file=" + f.item_rel + "\" target=\"_blank\">[View]</a>";
        } else {
            out += "<a href=\"/download?fs=" + String(fs_param) + "&file=" + f.item_rel + "&raw=1\" target=\"_blank\" title=\"View raw text in browser\">[View]</a>";
            out += "<a href=\"#\" onclick=\"dlFile('/download?fs=" + String(fs_param) + "&file=" + f.item_rel + "', '" + f.filename + "'); return false;\" title=\"Download file\">[Download]</a>";
        }
        out += "<a href=\"/delete?fs=" + String(fs_param) + "&file=" + f.item_rel + "\" class=\"del-btn\" onclick=\"return confirm('Delete " + f.filename + "?');\">[Delete]</a>";
        out += "</td>";

        out += "</tr>";
    }

    out += "</tbody></table></div>";
    return out;
}

static void handleRoot() {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>RTK Rover Portal " + String(ROVER_FIRMWARE_VERSION) + "</title><style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;margin:20px;background:#12181b;color:#e0e6ed;}"
        "h1{color:#00ffaa;margin-bottom:6px;}"
        "h2{color:#00d8ff;border-bottom:1px solid #334;padding-bottom:6px;margin-top:28px;}"
        "a{color:#38bdf8;text-decoration:none;}"
        "a:hover{text-decoration:underline;}"
        ".badge{background:#0284c7;color:#fff;padding:2px 8px;border-radius:4px;font-size:12px;font-weight:normal;margin-left:8px;}"
        ".up{margin-bottom:20px;padding:15px;background:#1e293b;border-radius:8px;}"
        "input[type=submit],button{background:#00ffaa;color:#000;font-weight:bold;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;}"
        ".sort-bar{margin:10px 0;display:flex;flex-wrap:wrap;gap:6px;align-items:center;}"
        ".btn-sort{background:#243447;color:#38bdf8;font-size:12px;font-weight:600;padding:5px 10px;border-radius:4px;border:1px solid #3b4f68;}"
        ".btn-sort:hover{background:#334e68;color:#00ffaa;}"
        ".tbl-wrap{overflow-x:auto;margin-top:8px;border-radius:8px;border:1px solid #233142;}"
        "table{width:100%;border-collapse:collapse;font-family:monospace;font-size:13px;background:#18222d;}"
        "th,td{padding:9px 12px;text-align:left;border-bottom:1px solid #233142;white-space:nowrap;}"
        "th{background:#1e293b;color:#00d8ff;cursor:pointer;user-select:none;}"
        "th:hover{color:#00ffaa;background:#26354a;}"
        "tr:hover{background:#1f2c3a;}"
        ".sort-icon{font-size:11px;margin-left:4px;color:#94a3b8;}"
        ".actions a{margin-right:8px;}"
        ".del-btn{color:#f87171 !important;}"
        ".del-btn:hover{color:#ef4444 !important;}"
        "</style><script>"
        "function dlFile(url, name){"
        "  fetch(url).then(function(r){if(!r.ok) throw new Error('Download failed: ' + r.status); return r.blob();})"
        "  .then(function(blob){"
        "    var mime = name.endsWith('.csv') ? 'text/csv;charset=utf-8' : (name.endsWith('.json') ? 'application/json;charset=utf-8' : 'text/plain;charset=utf-8');"
        "    var safeBlob = new Blob([blob], {type: mime});"
        "    var a = document.createElement('a'); a.href = URL.createObjectURL(safeBlob); a.download = name;"
        "    document.body.appendChild(a); a.click();"
        "    setTimeout(function(){URL.revokeObjectURL(a.href); a.remove();}, 2000);"
        "  }).catch(function(err){window.location.href = url;});"
        "}"
        "function toggleSort(tableId, colIdx, isNum) {"
        "  var tbl = document.getElementById(tableId);"
        "  if (!tbl) return;"
        "  var curDir = tbl.getAttribute('data-dir-' + colIdx);"
        "  var nextDir = (curDir === 'asc') ? 'desc' : 'asc';"
        "  sortTable(tableId, colIdx, isNum, nextDir);"
        "}"
        "function sortTable(tableId, colIdx, isNum, forceDir) {"
        "  var tbl = document.getElementById(tableId);"
        "  if (!tbl) return;"
        "  var tbody = tbl.querySelector('tbody');"
        "  if (!tbody) return;"
        "  var rows = Array.from(tbody.querySelectorAll('tr'));"
        "  var dir = forceDir || ((tbl.getAttribute('data-dir-' + colIdx) === 'asc') ? 'desc' : 'asc');"
        "  tbl.setAttribute('data-dir-' + colIdx, dir);"
        "  for (var c = 0; c < 3; c++) {"
        "    var icon = document.getElementById(tableId + '_i' + c);"
        "    if (icon) icon.innerText = (c === colIdx) ? (dir === 'asc' ? ' [^]' : ' [v]') : '';"
        "  }"
        "  rows.sort(function(a, b) {"
        "    var ca = a.cells[colIdx], cb = b.cells[colIdx];"
        "    var va = ca ? (ca.getAttribute('data-val') || ca.innerText.trim()) : '';"
        "    var vb = cb ? (cb.getAttribute('data-val') || cb.innerText.trim()) : '';"
        "    if (isNum) {"
        "      var na = parseFloat(va) || 0, nb = parseFloat(vb) || 0;"
        "      return (dir === 'asc') ? (na - nb) : (nb - na);"
        "    }"
        "    return (dir === 'asc') ? va.localeCompare(vb) : vb.localeCompare(va);"
        "  });"
        "  rows.forEach(function(r) { tbody.appendChild(r); });"
        "}"
        "</script></head><body><h1>RTK Rover Portal " + String(ROVER_FIRMWARE_VERSION) + "</h1>";

    auto upload_html = [](const char* fs_param, const char* title) -> String {
        return "<div class='up'><h3>Upload to " + String(title) + "</h3><form method='POST' action='/upload?fs=" + String(fs_param) + "' enctype='multipart/form-data'><input type='file' name='f' style='color:#fff'> <input type='submit' value='Upload'></form></div>";
    };

    if (sd_card_mounted) {
        html += upload_html("sd", "MicroSD Card");
        html += generate_dir_table_html("MicroSD Card (/sdcard)", "sd", "/sdcard", "tbl_sd");
    }

    html += upload_html("ffat", "Internal Flash (FFat)");
    html += generate_dir_table_html("Internal Flash (/ffat)", "ffat", "/ffat", "tbl_ffat");

    html += "</body></html>";
    server.send(200, "text/html; charset=utf-8", html);
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
    String disposition = "inline";
    
    if (filename.endsWith(".csv")) {
        mimeType = "text/csv; charset=utf-8";
    } else if (filename.endsWith(".bmp")) {
        mimeType = "image/bmp";
        disposition = "inline";
    } else if (filename.endsWith(".json")) {
        mimeType = "application/json; charset=utf-8";
    } else if (filename.endsWith(".log") || filename.endsWith(".txt")) {
        mimeType = "text/plain; charset=utf-8";
    }
    
    if (server.hasArg("raw")) {
        server.sendHeader("Content-Type", mimeType);
        server.sendHeader("X-Content-Type-Options", "nosniff");
        server.sendHeader("Content-Length", String(file.size()));
        server.sendHeader("Connection", "close");
        server.streamFile(file, mimeType);
        file.close();
        return;
    }

    server.sendHeader("Content-Disposition", disposition + "; filename=\"" + filename + "\"");
    server.sendHeader("X-Content-Type-Options", "nosniff");
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
    
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>View Image</title></head><body style='background:#12181b;color:white;text-align:center;font-family:sans-serif;padding:20px;'>";
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
    
    server.send(200, "text/html; charset=utf-8", html);
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
static File uploadMirrorFile;
static void handleUpload() {
    HTTPUpload& upload = server.upload();
    String fs_type = server.hasArg("fs") ? server.arg("fs") : "ffat";
    fs::FS* target_fs = get_target_fs(fs_type);

    if (upload.status == UPLOAD_FILE_START) {
        String filename = upload.filename;
        if (!filename.startsWith("/")) filename = "/" + filename;
        uploadFile = target_fs->open(filename, "w");
        if (upload.filename.endsWith("settings.json") && sd_card_mounted) {
            fs::FS* other_fs = (target_fs == &FFat) ? get_target_fs("sd") : &FFat;
            if (other_fs) {
                uploadMirrorFile = other_fs->open(filename, "w");
            }
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) uploadFile.write(upload.buf, upload.currentSize);
        if (uploadMirrorFile) uploadMirrorFile.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) uploadFile.close();
        if (uploadMirrorFile) uploadMirrorFile.close();
        if (upload.filename.endsWith("waypoints.csv")) {
            waypoint_load_csv(current_settings.active_waypoint_file);
            Serial.println("[WEB] Reloaded waypoints.csv after upload!");
        } else if (upload.filename.endsWith("settings.json")) {
            load_settings();
            Serial.println("[WEB] Reloaded settings.json after dual-FS sync upload!");
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
    if (wifi_active || s_wifi_starting) return;
    s_wifi_starting = true;
    
    update_wifi_ip_label("Connecting...");
    
    // Non-blocking connection task so UI stays responsive
    xTaskCreatePinnedToCore([](void* arg){
        WiFi.persistent(false);

        static wifi_event_id_t s_wifi_event_id = 0;
        if (s_wifi_event_id == 0) {
            s_wifi_event_id = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
                if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
                    uint8_t reason = info.wifi_sta_disconnected.reason;
                    Serial.printf("[WIFI EVENT] STA Disconnected (Reason: %u - %s)\n",
                                  reason, WiFi.disconnectReasonName((wifi_err_reason_t)reason));
                } else if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
                    Serial.println("[WIFI EVENT] STA Associated with AP!");
                } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
                    Serial.printf("[WIFI EVENT] STA Got IP: %s\n",
                                  IPAddress(info.got_ip.ip_info.ip.addr).toString().c_str());
                } else if (event == ARDUINO_EVENT_WIFI_AP_START) {
                    Serial.println("[WIFI EVENT] SoftAP Started!");
                }
            });
        }

        // Check if SSID is a placeholder or blank
        bool has_valid_ssid = (current_settings.wifi_ssid[0] != '\0' &&
                               strcasecmp(current_settings.wifi_ssid, "Your_SSID") != 0 &&
                               strcasecmp(current_settings.wifi_ssid, "Your-SSID-Here") != 0 &&
                               strcasecmp(current_settings.wifi_ssid, "Your-SSID") != 0 &&
                               strcasecmp(current_settings.wifi_ssid, "SSID") != 0);

        bool sta_connected = false;

        if (has_valid_ssid) {
            Serial.printf("[WIFI] Connecting to '%s' (password len=%d)...\n",
                          current_settings.wifi_ssid, (int)strlen(current_settings.wifi_password));
            
            // Clean reset of Wi-Fi subsystem before entering STA mode
            WiFi.disconnect(true);
            vTaskDelay(pdMS_TO_TICKS(100));
            WiFi.mode(WIFI_STA);
            WiFi.setSleep(false);
            WiFi.setAutoReconnect(true);

            // Use standard Fast Scan & native security negotiation (supports WPA, WPA2, WPA3)
            WiFi.begin(current_settings.wifi_ssid, current_settings.wifi_password);

            int attempts = 0;
            const int max_attempts = 40; // 20 seconds timeout (40 * 500ms)
            wl_status_t last_status = WL_IDLE_STATUS;

            while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
                vTaskDelay(pdMS_TO_TICKS(500));
                attempts++;
                wl_status_t cur_status = WiFi.status();
                if (cur_status != last_status || (attempts % 4 == 0)) {
                    last_status = cur_status;
                    const char* st_name = (cur_status == WL_IDLE_STATUS)     ? "IDLE" :
                                          (cur_status == WL_NO_SSID_AVAIL)   ? "NO_SSID_AVAIL" :
                                          (cur_status == WL_SCAN_COMPLETED)  ? "SCAN_COMPLETED" :
                                          (cur_status == WL_CONNECTED)       ? "CONNECTED" :
                                          (cur_status == WL_CONNECT_FAILED)  ? "CONNECT_FAILED" :
                                          (cur_status == WL_CONNECTION_LOST) ? "CONNECTION_LOST" :
                                          (cur_status == WL_DISCONNECTED)    ? "DISCONNECTED" : "UNKNOWN";
                    Serial.printf("[WIFI] Status: %s (%d) | %d/%d (%.1fs)\n",
                                  st_name, (int)cur_status, attempts, max_attempts, attempts * 0.5f);
                    char prog[32];
                    snprintf(prog, sizeof(prog), "Connecting (%ds)...", (int)(attempts * 0.5f));
                    update_wifi_ip_label(prog);
                }
                
                // If stuck disconnected after 10 attempts (5s), actively re-trigger connection
                if (cur_status == WL_DISCONNECTED && (attempts == 10 || attempts == 20 || attempts == 30)) {
                    Serial.printf("[WIFI] Re-triggering connection attempt (%d/%d)...\n", attempts, max_attempts);
                    WiFi.reconnect();
                }
            }
            sta_connected = (WiFi.status() == WL_CONNECTED);
        } else {
            Serial.println("[WIFI] No valid STA credentials configured in settings.json. Launching SoftAP hotspot immediately...");
        }

        if (sta_connected) {
            wifi_active = true;
            s_wifi_starting = false;
            String ip = WiFi.localIP().toString();
            strncpy(s_wifi_ip, ip.c_str(), sizeof(s_wifi_ip) - 1);
            s_wifi_ip[sizeof(s_wifi_ip) - 1] = '\0';
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
            Serial.printf("[WIFI] Connected to '%s'! Web Portal: http://%s/\n", current_settings.wifi_ssid, s_wifi_ip);
        } else {
            // STA failed or unconfigured in field: Launch local SoftAP hotspot fallback
            if (has_valid_ssid) {
                Serial.printf("[WIFI] STA connection to '%s' failed (Status: %d). Starting SoftAP 'RTK-Rover' fallback...\n",
                              current_settings.wifi_ssid, (int)WiFi.status());
            }
            update_wifi_ip_label("Starting AP...");
            WiFi.disconnect(true);
            vTaskDelay(pdMS_TO_TICKS(100));
            WiFi.mode(WIFI_AP);
            
            IPAddress ap_local_ip(192, 168, 4, 1);
            IPAddress ap_gateway(192, 168, 4, 1);
            IPAddress ap_subnet(255, 255, 255, 0);
            WiFi.softAPConfig(ap_local_ip, ap_gateway, ap_subnet);
            WiFi.softAP("RTK-Rover", "12345678");
            
            // Wait up to 1 second for AP netif to bring up IP
            int ap_wait = 0;
            while (WiFi.softAPIP() == IPAddress(0, 0, 0, 0) && ap_wait < 10) {
                vTaskDelay(pdMS_TO_TICKS(100));
                ap_wait++;
            }
            
            IPAddress apIP = WiFi.softAPIP();
            if (apIP == IPAddress(0, 0, 0, 0)) {
                apIP = ap_local_ip; // Guaranteed fallback: 192.168.4.1
            }
            String ip = apIP.toString();
            strncpy(s_wifi_ip, ip.c_str(), sizeof(s_wifi_ip) - 1);
            s_wifi_ip[sizeof(s_wifi_ip) - 1] = '\0';
            wifi_active = true;
            s_wifi_starting = false;
            update_wifi_ip_label(s_wifi_ip);
            
            setup_web_routes();
            server.begin();
            Serial.printf("[WIFI] SoftAP Active! Connect phone to SSID 'RTK-Rover' (pwd: 12345678) -> http://%s/\n", s_wifi_ip);
        }
        s_wifi_starting = false;
        vTaskDelete(NULL);
    }, "wifi_connect", 8192, NULL, 1, NULL, 0);
}

extern "C" void stop_wifi(void) {
    s_wifi_starting = false;
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

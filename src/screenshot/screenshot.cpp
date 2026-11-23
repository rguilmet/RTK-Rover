#include "screenshot.h"
#include "../../user_config.h"
#include <SD_MMC.h>
#include <SD.h>
#include <FFat.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_heap_caps.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include "../lvgl_port/lvgl_port.h"

extern bool sd_card_use_spi;
extern SemaphoreHandle_t fs_mux;

static uint64_t get_fs_free_kb(const fs::FS *fs_ptr)
{
    uint64_t tot = 0;
    uint64_t use = 0;
    if (fs_ptr == &FFat) {
        tot = FFat.totalBytes();
        use = FFat.usedBytes();
    } else if (sd_card_use_spi) {
        tot = SD.totalBytes();
        use = SD.usedBytes();
    } else {
        tot = SD_MMC.totalBytes();
        use = SD_MMC.usedBytes();
    }
    return (tot > use) ? ((tot - use) / 1024ULL) : 0;
}

static File open_screenshot_file(fs::FS &fs, const char *fs_name, const char *ts,
                                 char *out_path, size_t out_path_sz,
                                 bool *out_mkdir_failed)
{
    (void)fs_name;
    *out_mkdir_failed = false;
    if (!fs.exists("/screenshots")) {
        if (!fs.mkdir("/screenshots")) {
            *out_mkdir_failed = true;
        }
    }

    snprintf(out_path, out_path_sz, "/screenshots/screenshot_%s.bmp", ts);
    File file = fs.open(out_path, FILE_WRITE);
    if (!file) {
        // Fallback to flat root filename if /screenshots directory is not supported
        snprintf(out_path, out_path_sz, "/screenshot_%s.bmp", ts);
        file = fs.open(out_path, FILE_WRITE);
    }
    return file;
}

extern "C" bool take_screenshot_to_sd(void)
{
    uint32_t t0_ms = millis();

    const uint32_t w = ROVER_LCD_H_RES;
    const uint32_t h = ROVER_LCD_V_RES;
    const uint32_t fb_bytes = w * h * 2U;
    const uint32_t row_size = ((w * 3U + 3U) & ~3U);
    const uint32_t image_size = row_size * h;
    const uint32_t file_size = 54U + image_size;

    uint8_t *fb_copy = (uint8_t *)heap_caps_malloc((size_t)fb_bytes,
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!fb_copy) {
        Serial.printf("[SCREENSHOT ERR] PSRAM fb alloc failed (%lu B, PSRAM free=%u B)\n",
                      (unsigned long)fb_bytes,
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return false;
    }

    if (!rover_lvgl_capture_framebuffer(fb_copy, (size_t)fb_bytes)) {
        Serial.println("[SCREENSHOT ERR] Framebuffer capture timed out waiting for LVGL render cycle");
        heap_caps_free(fb_copy);
        return false;
    }

    // Use small internal DMA-capable bounce buffer (0 B static DRAM0) so SDMMC/FFat VFS
    // writes never DMA directly from Octal PSRAM.
    uint32_t chunk_rows = 2U;
    uint8_t *chunk_buf = (uint8_t *)heap_caps_malloc((size_t)row_size * chunk_rows,
                                                     MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    if (!chunk_buf) {
        chunk_rows = 1U;
        chunk_buf = (uint8_t *)heap_caps_malloc((size_t)row_size,
                                                MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    if (!chunk_buf) {
        Serial.printf("[SCREENSHOT ERR] Internal DMA chunk alloc failed (%lu B, DMA free=%u B)\n",
                      (unsigned long)row_size,
                      (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));
        heap_caps_free(fb_copy);
        return false;
    }

    char filename[64];
    char ts[32];
    ts[0] = '\0';
    get_formatted_timestamp(ts, sizeof(ts), true);
    if (ts[0] == '\0') {
        snprintf(ts, sizeof(ts), "uptime_%lu", (unsigned long)millis());
    }

    if (fs_mux && xSemaphoreTake(fs_mux, pdMS_TO_TICKS(5000)) != pdTRUE) {
        Serial.println("[SCREENSHOT ERR] Timed out waiting for fs_mux");
        heap_caps_free(chunk_buf);
        heap_caps_free(fb_copy);
        return false;
    }

    fs::FS *primary_fs = &get_active_fs();
    const char *primary_name = sd_card_mounted ? (sd_card_use_spi ? "SD_SPI" : "SD_MMC") : "FFat";
    fs::FS *target_fs = primary_fs;
    const char *target_name = primary_name;

    bool primary_mkdir_failed = false;
    bool fallback_mkdir_failed = false;
    bool used_ffat_fallback = false;
    int saved_errno = 0;

    errno = 0;
    File file = open_screenshot_file(*primary_fs, primary_name, ts,
                                     filename, sizeof(filename), &primary_mkdir_failed);
    if (!file) {
        saved_errno = errno;
        if (sd_card_mounted) {
            used_ffat_fallback = true;
            target_fs = &FFat;
            target_name = "FFat";
            errno = 0;
            file = open_screenshot_file(FFat, target_name, ts,
                                        filename, sizeof(filename), &fallback_mkdir_failed);
            if (!file) {
                saved_errno = errno;
            }
        }
    }

    if (!file) {
        uint64_t sd_free_kb = sd_card_mounted ? get_fs_free_kb(primary_fs) : 0;
        uint64_t ff_free_kb = get_fs_free_kb(&FFat);

        if (fs_mux) xSemaphoreGive(fs_mux);
        if (primary_mkdir_failed) {
            Serial.printf("[SCREENSHOT WARN] mkdir /screenshots failed on %s\n", primary_name);
        }
        if (used_ffat_fallback) {
            Serial.printf("[SCREENSHOT WARN] Open failed on %s -- falling back to FFat\n", primary_name);
            if (fallback_mkdir_failed) {
                Serial.println("[SCREENSHOT WARN] mkdir /screenshots failed on FFat");
            }
        }
        Serial.printf("[SCREENSHOT ERR] Open failed on %s (errno=%d, sd_mounted=%d, sd_free=%lluKB, ffat_free=%lluKB)\n",
                      target_name, saved_errno, sd_card_mounted ? 1 : 0,
                      (unsigned long long)sd_free_kb, (unsigned long long)ff_free_kb);
        heap_caps_free(chunk_buf);
        heap_caps_free(fb_copy);
        return false;
    }

    const int32_t neg_h = -(int32_t)h;
    const uint32_t neg_h_u = (uint32_t)neg_h;

    uint8_t header[54] = {
        'B', 'M',
        (uint8_t)(file_size), (uint8_t)(file_size >> 8), (uint8_t)(file_size >> 16), (uint8_t)(file_size >> 24),
        0, 0, 0, 0,
        54, 0, 0, 0,
        40, 0, 0, 0,
        (uint8_t)(w), (uint8_t)(w >> 8), (uint8_t)(w >> 16), (uint8_t)(w >> 24),
        (uint8_t)(neg_h_u), (uint8_t)(neg_h_u >> 8), (uint8_t)(neg_h_u >> 16), (uint8_t)(neg_h_u >> 24),
        1, 0,
        24, 0,
        0, 0, 0, 0,
        (uint8_t)(image_size), (uint8_t)(image_size >> 8), (uint8_t)(image_size >> 16), (uint8_t)(image_size >> 24),
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 0
    };

    uint32_t bytes_written = 0;
    errno = 0;
    bool write_ok = (file.write(header, 54) == 54);
    if (write_ok) {
        bytes_written += 54;
    } else {
        saved_errno = errno;
    }

    const uint8_t *p565 = fb_copy;
    for (uint32_t y_base = 0; write_ok && y_base < h; y_base += chunk_rows) {
        uint32_t rows_in_chunk = h - y_base;
        if (rows_in_chunk > chunk_rows) {
            rows_in_chunk = chunk_rows;
        }

        uint32_t pos = 0;
        for (uint32_t r_idx = 0; r_idx < rows_in_chunk; r_idx++) {
            uint32_t row_start = pos;
            for (uint32_t x = 0; x < w; x++) {
                // rover_lvgl_flush_cb() byte-swaps RGB565 in place before DMA (BE bytes)
                uint16_t color = ((uint16_t)p565[0] << 8) | (uint16_t)p565[1];
                p565 += 2;

                uint8_t r = (uint8_t)((color >> 11) & 0x1F);
                uint8_t g = (uint8_t)((color >> 5) & 0x3F);
                uint8_t b = (uint8_t)(color & 0x1F);

                chunk_buf[pos++] = (uint8_t)((b * 255U + 15U) / 31U);
                chunk_buf[pos++] = (uint8_t)((g * 255U + 31U) / 63U);
                chunk_buf[pos++] = (uint8_t)((r * 255U + 15U) / 31U);
            }
            while ((pos - row_start) < row_size) {
                chunk_buf[pos++] = 0;
            }
        }

        errno = 0;
        size_t written = file.write(chunk_buf, pos);
        bytes_written += (uint32_t)written;
        if (written != pos) {
            saved_errno = errno;
            write_ok = false;
        }
    }

    if (write_ok) {
        file.flush();
    }
    file.close();

    size_t verified_size = 0;
    if (write_ok) {
        errno = 0;
        File verify = target_fs->open(filename, FILE_READ);
        verified_size = verify ? verify.size() : 0;
        if (verify) {
            verify.close();
        }
        if (verified_size != (size_t)file_size) {
            saved_errno = errno;
            write_ok = false;
            target_fs->remove(filename);
        }
    } else {
        target_fs->remove(filename);
    }

    uint64_t free_kb = (!write_ok) ? get_fs_free_kb(target_fs) : 0;

    if (fs_mux) xSemaphoreGive(fs_mux);

    heap_caps_free(chunk_buf);
    heap_caps_free(fb_copy);

    if (primary_mkdir_failed) {
        Serial.printf("[SCREENSHOT WARN] mkdir /screenshots failed on %s\n", primary_name);
    }
    if (used_ffat_fallback) {
        Serial.printf("[SCREENSHOT WARN] Open failed on %s -- falling back to FFat\n", primary_name);
        if (fallback_mkdir_failed) {
            Serial.println("[SCREENSHOT WARN] mkdir /screenshots failed on FFat");
        }
    }

    uint32_t elapsed_ms = millis() - t0_ms;
    if (!write_ok) {
        Serial.printf("[SCREENSHOT ERR] Write/verify failed on %s (%s) wrote=%lu/%lu verified=%lu errno=%d free=%lluKB\n",
                      target_name, filename,
                      (unsigned long)bytes_written,
                      (unsigned long)file_size,
                      (unsigned long)verified_size,
                      saved_errno,
                      (unsigned long long)free_kb);
        return false;
    }

    Serial.printf("[SCREENSHOT] Saved %s on %s (%lu bytes verified, %lu ms)\n",
                  filename, target_name,
                  (unsigned long)verified_size,
                  (unsigned long)elapsed_ms);
    return true;
}

#ifndef RTCM_ARBITER_H
#define RTCM_ARBITER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RTCM_ARBITER_LORA_FAILOVER_MS   3000U
#define RTCM_ARBITER_LORA_RECOVER_COUNT 3U
#define RTCM_ARBITER_STALE_MS           10000U
#define RTCM_ARBITER_NTRIP_TRUNC_MS     600U

void rtcm_arbiter_init(void);
void rtcm_arbiter_poll(void);
void rtcm_arbiter_submit_ble_stream(const uint8_t *data, size_t len);
bool rtcm_arbiter_submit_frame(RtcmSourceId_t src,
                               const uint8_t *frame,
                               size_t len,
                               uint16_t msg_type,
                               uint32_t now_ms);

void rtcm_arbiter_set_mode(RtcmSourceMode_t mode);
bool rtcm_arbiter_set_mode_from_str(const char *mode_str);
RtcmSourceMode_t rtcm_arbiter_get_mode(void);
RtcmSourceId_t rtcm_arbiter_get_active_source(void);
const char *rtcm_arbiter_mode_to_str(RtcmSourceMode_t mode);
const char *rtcm_arbiter_source_to_str(RtcmSourceId_t src);
void rtcm_arbiter_get_stats(RtcmArbiterStats_t *out);

#ifdef __cplusplus
}
#endif

#endif // RTCM_ARBITER_H

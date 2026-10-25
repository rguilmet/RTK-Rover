#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void start_wifi(void);
void stop_wifi(void);
void handle_wifi_server(void);
bool is_wifi_active(void);
const char* get_wifi_ip(void);

#ifdef __cplusplus
}
#endif

#ifndef SERIAL_LOGGER_H
#define SERIAL_LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
#include <Arduino.h>
#include <Stream.h>

class SerialLoggerClass : public Stream {
public:
    SerialLoggerClass();
    void begin(unsigned long baud = 115200);
    void end();
    void setTxTimeoutMs(uint32_t timeout);

    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    int available() override;
    int read() override;
    int peek() override;
    void flush() override;

    operator bool() const { return (bool)::Serial; }

    bool start();
    void stop();
    bool isActive() const;
    void flushBuffer();
    const char* getCurrentFilename() const;
    uint32_t getBytesLogged() const;
};

extern SerialLoggerClass SerialLogger;

extern "C" {
#endif

bool serial_logger_start(void);
void serial_logger_stop(void);
bool serial_logger_is_active(void);
void serial_logger_flush(void);
const char* serial_logger_get_current_filename(void);
uint32_t serial_logger_get_bytes_logged(void);
int rover_printf(const char *format, ...);

#ifdef __cplusplus
}

#ifndef RTK_ROVER_NO_SERIAL_REDIRECT
#define Serial SerialLogger
#endif

#endif

#endif // SERIAL_LOGGER_H

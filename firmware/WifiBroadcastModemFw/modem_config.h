#ifndef MODEM_CONFIG_H_
#define MODEM_CONFIG_H_

//#define LOG_DEBUG(...) Serial.printf(__VA_ARGS__)
//#define LOG_ERROR(...) Serial.printf(__VA_ARGS__)

int log_printf(const char *format, ... );

#define NEWLINE     "\n"

#define LOG_DEBUG(...)
//#define LOG_ERROR(message, args...) Serial.printf(message NEWLINE, ## args)
#define LOG_ERROR(message, args...) log_printf(message NEWLINE, ## args)

#define MODEM_SERIAL_ENABLED

#endif

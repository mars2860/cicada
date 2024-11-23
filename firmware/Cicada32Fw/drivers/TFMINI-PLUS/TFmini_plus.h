#ifndef TF_MINI_PLUS_H
#define TF_MINI_PLUS_H

#include <Arduino.h>
#include <Wire.h>

///////////////////////////////////////////////////////////////////////////////

const uint8_t TFMINI_PLUS_FRAME_START = 0x5A;
const uint8_t TFMINI_PLUS_RESPONSE_FRAME_HEADER = 0x59;
const uint8_t TFMINI_PLUS_MINIMUM_PACKET_SIZE = 4;

const float TFMINI_PLUS_P00 = 0.9758;
const float TFMINI_PLUS_P01 = 1.175;
const float TFMINI_PLUS_P10 = -0.6072;
const float TFMINI_PLUS_P20 = 0.09501;
const float TFMINI_PLUS_P11 = -0.2904;

typedef struct {
    uint16_t distance;
    uint16_t strength;
    float temperature;
} tfminiplus_data_t;

typedef union {
    uint8_t raw[3];
    struct {
        uint8_t major;
        uint8_t minor;
        uint8_t revision;
    };
} tfminiplus_version_t;

///////////////////////////////////////////////////////////////////////////////

typedef enum TFMINI_PLUS_COMMANDS {
    TFMINI_PLUS_GET_DATA = 0,
    TFMINI_PLUS_GET_VERSION = 1,
    TFMINI_PLUS_SYSTEM_RESET = 2,
    TFMINI_PLUS_SET_FRAME_RATE = 3,
    TFMINI_PLUS_TRIGGER_DETECTION = 4,
    TFMINI_PLUS_SET_OUTPUT_FORMAT = 5,
    TFMINI_PLUS_SET_BAUD_RATE = 6,
    TFMINI_PLUS_ENABLE_DATA_OUTPUT = 7,
    TFMINI_PLUS_SET_COMMUNICATION_INTERFACE = 0x0A,
    TFMINI_PLUS_SET_I2C_ADDRESS = 0x0B,
    TFMINI_PLUS_SET_IO_MODE = 0x3B,
    TFMINI_PLUS_RESTORE_FACTORY_SETTINGS = 0x10,
    TFMINI_PLUS_SAVE_SETTINGS = 0x11
} tfminiplus_command_t;

enum TFMINI_PLUS_PACKET_LENGTHS {
    TFMINI_PLUS_PACK_LENGTH_SYSTEM_RESET_RESPONSE = 5,
    TFMINI_PLUS_PACK_LENGTH_RESTORE_FACTORY_SETTINGS_RESPONSE = 5,
    TFMINI_PLUS_PACK_LENGTH_SAVE_SETTINGS_RESPONSE = 5,
    TFMINI_PLUS_PACK_LENGTH_GET_DATA = 5,
    TFMINI_PLUS_PACK_LENGTH_SET_FRAME_RATE = 6,
    TFMINI_PLUS_PACK_LENGTH_SET_OUTPUT_FORMAT = 5,
    TFMINI_PLUS_PACK_LENGTH_SET_BAUD_RATE = 8,
    TFMINI_PLUS_PACK_LENGTH_ENABLE_DATA_OUTPUT = 5,
    TFMINI_PLUS_PACK_LENGTH_SET_COMMUNICATION_INTERFACE = 5,
    TFMINI_PLUS_PACK_LENGTH_SET_I2C_ADDRESS = 5,
    TFMINI_PLUS_PACK_LENGTH_SET_IO_MODE = 9,
    TFMINI_PLUS_PACK_LENGTH_DATA_RESPONSE = 9,
    TFMINI_PLUS_PACK_LENGTH_VERSION_RESPONSE = 7
};

enum TFMINI_PLUS_PACKET_POSITIONS {
    TFMINI_PLUS_PACKET_POS_START = 0,
    TFMINI_PLUS_PACKET_POS_LENGTH = 1,
    TFMINI_PLUS_PACKET_POS_COMMAND = 2,
};

typedef enum TFMINI_PLUS_IO_MODE {
    STANDARD = 0,
    IO_NEAR_HIGH_FAR_LOW = 1,
    IO_NEAR_LOW_FAR_HIGH = 2,
} tfminiplus_mode_t;

typedef enum TFMINI_PLUS_FRAMERATE {
    TFMINI_PLUS_FRAMERATE_0HZ = 0,
    TFMINI_PLUS_FRAMERATE_1HZ = 1,
    TFMINI_PLUS_FRAMERATE_2HZ = 2,
    TFMINI_PLUS_FRAMERATE_5HZ = 5,
    TFMINI_PLUS_FRAMERATE_10HZ = 10,
    TFMINI_PLUS_FRAMERATE_20HZ = 20,
    TFMINI_PLUS_FRAMERATE_25HZ = 25,
    TFMINI_PLUS_FRAMERATE_50HZ = 50,
    TFMINI_PLUS_FRAMERATE_100HZ = 100,
    TFMINI_PLUS_FRAMERATE_200HZ = 200,
    TFMINI_PLUS_FRAMERATE_250HZ = 250,
    TFMINI_PLUS_FRAMERATE_500HZ = 500,
    TFMINI_PLUS_FRAMERATE_1000HZ = 1000,
} tfminiplus_framerate_t;

typedef enum TFMINI_PLUS_BAUDRATE {
    TFMINI_PLUS_BAUDRATE_9600 = 9600,
    TFMINI_PLUS_BAUDRATE_19200 = 19200,
    TFMINI_PLUS_BAUDRATE_38400 = 38400,
    TFMINI_PLUS_BAUDRATE_57600 = 57600,
    TFMINI_PLUS_BAUDRATE_115200 = 115200
} tfminiplus_baudrate_t;

typedef enum TFMINI_PLUS_OUTPUT_FORMAT {
    TFMINI_PLUS_OUTPUT_CM = 1,
    TFMINI_PLUS_OUTPUT_PIXHAWK = 2,
    TFMINI_PLUS_OUTPUT_MM = 6
} tfminiplus_output_format_t;

typedef enum TFMINI_PLUS_COMMUNICATION_MODE {
    TFMINI_PLUS_UART = 0,
    TFMINI_PLUS_I2C = 1,
} tfminiplus_communication_mode_t;

///////////////////////////////////////////////////////////////////////////////

class TFminiPlus {
   public:
    void begin(uint8_t address = 0x10);
    void begin(Stream *stream);

    bool set_communication_interface(tfminiplus_communication_mode_t mode);
    bool set_i2c_address(uint8_t address);

    bool save_settings();
    bool reset_system();
    bool factory_reset();
    tfminiplus_version_t get_version();

    bool enable_output(bool output_enabled);
    bool set_framerate(tfminiplus_framerate_t framerate);
    bool set_baudrate(tfminiplus_baudrate_t baudrate);
    bool set_output_format(tfminiplus_output_format_t format);
    bool set_io_mode(tfminiplus_mode_t mode, uint16_t critical_distance = 0, uint16_t hysteresis = 0);

    void trigger_manual_reading();
    bool read_manual_reading(tfminiplus_data_t &data);
    tfminiplus_data_t get_manual_reading();
    uint16_t get_manual_distance();

    bool read_data(tfminiplus_data_t &data, bool in_mm_format = true);
    tfminiplus_data_t get_data(bool in_mm_format = true);
    uint16_t get_distance(bool in_mm_format = true);

    float get_effective_accuracy(uint16_t strength, uint16_t frequency);

    void dump_serial_cache();

   private:
    uint8_t _address;
    uint8_t _communications_mode;
    Stream *_stream;

    void do_i2c_wait();

    bool send(uint8_t *input, uint8_t size);
    bool send_uart(uint8_t *input, uint8_t size);
    bool send_i2c(uint8_t *input, uint8_t size);
    bool send_command(tfminiplus_command_t command, uint8_t *arguments, uint8_t size);
    bool send_command(tfminiplus_command_t command);

    bool receive(uint8_t *output, uint8_t size);
    uint8_t receive_uart(uint8_t *output, uint8_t size, unsigned long timeout = 10);
    uint8_t receive_i2c(uint8_t *output, uint8_t size);
    bool uart_receive_data(uint8_t *output, uint8_t size, unsigned long timeout = 10);
    bool read_data_response(tfminiplus_data_t &data);

    uint8_t calculate_checksum(uint8_t *data, uint8_t size);
    bool compare_checksum(uint8_t *data, uint8_t size);
};

#endif

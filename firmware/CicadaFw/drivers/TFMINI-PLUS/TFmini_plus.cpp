#include <TFmini_plus.h>

///////////////////////////////////////////////////////////////////////////////

/**
 * Send a packet to the lidar.
 * Data is sent by either UART or I2C; whichever is actively in use.
 *
 * @param input: Array containing headers, payload, and checksum.
 * @param size: Number of bytes to send.
 * @return: True for successful transmission.
 */
bool TFminiPlus::send(uint8_t *input, uint8_t size) {
    bool result = false;
    if (_communications_mode == TFMINI_PLUS_UART) result = send_uart(input, size);
    if (_communications_mode == TFMINI_PLUS_I2C) result = send_i2c(input, size);

    return result;
}

/**
 * Send the packet over UART.
 * Both software and hardware implementations are allowed.
 * @param input: Data packet to send.
 * @param size: Number of bytes to send.
 * @return: True if the correct number of bytes were sent.
 */
bool TFminiPlus::send_uart(uint8_t *input, uint8_t size) {
    // Burn any other bytes waiting in the receive buffer
    dump_serial_cache();
    dump_serial_cache();

    uint8_t bytes_sent = _stream->write(input, size) == size;

    return bytes_sent == size;
}

/**
 * Send the packet over I2C.
 * Both software and hardware implementations are allowed.
 * @param input: Data packet to send.
 * @param size: Number of bytes to send.
 * @return: True if the correct number of bytes were sent.
 */
bool TFminiPlus::send_i2c(uint8_t *input, uint8_t size) {
    Wire.beginTransmission(_address);
    uint8_t bytes_sent = Wire.write(input, size);
    if (bytes_sent != size) Wire.write(0);
    uint8_t error = Wire.endTransmission(true);
    return (bytes_sent == size and not error);
}

/**
 * Send a command packet to the lidar.
 * Command packets use the following structure:
 * [0] 0x5A - Command header
 * [1] Length of packet
 * [2] Command code
 * [3-n] Command arguments (Not all commands have arguments)
 * [n] Checksum
 *
 * @param command: 8-bit command to send; see TFMINI_PLUS_COMMANDS.
 * @param arguments: Container containing the command arguments in little-endian format.
 * @param size: Total number of bytes to send, including header and checksum.
 * @return: True if the transmission was successful.
 */
bool TFminiPlus::send_command(tfminiplus_command_t command, uint8_t *arguments, uint8_t size) {
    bool result;
    uint8_t packet[size];
    packet[0] = TFMINI_PLUS_FRAME_START;
    packet[1] = size;
    packet[2] = command;

    // Put arguments into the packet as-is, assuming the endianess has been handled elsewhere
    if (size > TFMINI_PLUS_MINIMUM_PACKET_SIZE) {
        for (uint8_t i = 0; i < size - TFMINI_PLUS_MINIMUM_PACKET_SIZE; i++) {
            packet[TFMINI_PLUS_MINIMUM_PACKET_SIZE - 1 + i] = arguments[i];
        }
    }

    // Slap on the checksum and run
    packet[size - 1] = calculate_checksum(packet, size - 1);
    result = send(packet, size);
    return result;
}

/**
 * Send a command packet to the lidar.
 * Only to be used with commands that do not take arguments.
 *
 * @param command: 8-bit command to send; see TFMINI_PLUS_COMMANDS.
 * @return: True if the transmission was successful.
 */
bool TFminiPlus::send_command(tfminiplus_command_t command) { return send_command(command, 0, TFMINI_PLUS_MINIMUM_PACKET_SIZE); }

///////////////////////////////////////////////////////////////////////////////

/**
 * Get a packet from the lidar.
 * Data is received by either UART or I2c; whichever is actively in use.
 *
 * @param output: Container for received data, headers, and checksum.
 * @param size: Number of bytes expected to receive.
 * @return: True if the received length and checksum is valid
 */
bool TFminiPlus::receive(uint8_t *output, uint8_t size) {
    bool result = false;
    uint8_t bytes_received = 0;

    // Grab data from stream
    if (_communications_mode == TFMINI_PLUS_UART) bytes_received = receive_uart(output, size);
    if (_communications_mode == TFMINI_PLUS_I2C) bytes_received = receive_i2c(output, size);
    result = (bytes_received == size);

    // Data is valid if the packet length matches the received length value and if the checksum matches
    result &= compare_checksum(output, size);
    return result;
}

/**
 * Receive data from the UART.
 *
 * @param output: Container for data to read into.
 * @param size: Number of bytes expected to be read.
 * @return: True if the expected number of bytes was read.
 */
uint8_t TFminiPlus::receive_uart(uint8_t *output, uint8_t size, unsigned long timeout) {
    bool packet_start_found = false;
    unsigned long start_time = millis();
    uint8_t bytes_read = 0;

    // Discard data until a valid packet header is found (0x5a)
    while ((millis() - start_time) < timeout and not packet_start_found) {
        if (_stream->available() >= size) {
            if (_stream->read() == TFMINI_PLUS_FRAME_START) {
                if (_stream->read() == size) {
                    packet_start_found = true;
                    output[0] = TFMINI_PLUS_FRAME_START;
                    output[1] = size;
                    bytes_read = _stream->readBytes(&output[2], size - 2) + 2;
                }
            }
        }
    }

    return bytes_read;
}

/**
 * Receive data from the I2C bus.
 *
 * @param output: Container for data to read into.
 * @param size: Number of bytes expected to be read.
 * @return: True if the expected number of bytes was read.
 */
uint8_t TFminiPlus::receive_i2c(uint8_t *output, uint8_t size) {
    Wire.requestFrom(_address, size, true);

    uint8_t bytes_read = 0;
    for (size_t i = 0; (i < size) and Wire.available(); i++) {
        uint8_t c = Wire.read();
        output[i] = c;
        bytes_read++;
    }

    return bytes_read;
}

/**
 * Get a data packet from UART.
 * Data packets can be received at any time, so a synchronised packet start is not guaranteed.
 * This function discards received data until a valid packet is found.
 * Only the first valid packet is read. More packets may be available in the stream's buffer.
 *
 * @param output: Container to read data into.
 * @param size: Number of bytes that are expected to be received
 * @return: True if the expected number of bytes were received.
 */
bool TFminiPlus::uart_receive_data(uint8_t *output, uint8_t size, unsigned long timeout) {
    bool packet_start_found = false;
    unsigned long start_time = millis();

    // Discard data until a valid packet header is found (0x5959)
    while ((millis() - start_time) < timeout and not packet_start_found) {
        if (_stream->available() >= 9) {
            if (_stream->read() == TFMINI_PLUS_RESPONSE_FRAME_HEADER and _stream->read() == TFMINI_PLUS_RESPONSE_FRAME_HEADER) {
                output[0] = TFMINI_PLUS_RESPONSE_FRAME_HEADER;
                output[1] = TFMINI_PLUS_RESPONSE_FRAME_HEADER;
                packet_start_found = true;
            }
        }
    }

    // Read in the rest of the packet
    uint8_t bytes_read = _stream->readBytes(&output[2], size - 2) + 2;
    return bytes_read == size;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Check if the calculate checksum of a data packet matches the sent checksum byte.
 *
 * @param data: Container of received packet.
 * @param size: Number of bytes contained in the packet, including headers and checksum.
 * @return: True if the checksums match.
 */
bool TFminiPlus::compare_checksum(uint8_t *data, uint8_t size) {
    uint8_t checksum = calculate_checksum(data, size - 1);
    bool checksums_match = checksum == data[size - 1];

    return checksums_match;
}

/**
 * Calculate the checksum for a container.
 *
 * @param data: Container with data to calculate checksum.
 * @param size: Number of bytes in container.
 * @return: Checksum of data in the container.
 */
uint8_t TFminiPlus::calculate_checksum(uint8_t *data, uint8_t size) {
    uint8_t checksum = 0;

    for (size_t i = 0; i < size; i++) {
        checksum += data[i];
    }
    return checksum;
}

/**
 * Wait for the lidar to process a command.
 * This wait is only required when using the I2C bus.
 * This wait is also not required when requesting a data packet on either communication mode.
 */
void TFminiPlus::do_i2c_wait() {
    if (_communications_mode == TFMINI_PLUS_I2C) delay(150);
}

///////////////////////////////////////////////////////////////////////////////

/**
 * Start communication with the lidar in I2C mode.
 * @param address: I2C address of the lidar. Defaults to 0x10.
 */
void TFminiPlus::begin(uint8_t address) {
    _communications_mode = TFMINI_PLUS_I2C;
    _address = address & 0x7F;
}

/**
 * Start communication with the lidar in UART mode.
 * @param stream: Pointer to a stream object for communication (eg. HardwareSerial or SoftwareSerial)
 */
void TFminiPlus::begin(Stream *stream) {
    _communications_mode = TFMINI_PLUS_UART;
    _stream = stream;
    _stream->flush();
}

/**
 * Set the I2C address of the lidar.
 * This will change the slave address of the lidar so the device is mapped to a separate logical location.
 * Changes will take effect immediately upon confirmation that the address has been received correctly.
 *
 * @param address: I2C slave address to change the lidar to. (0x01 - 0x7F)
 * @param return: True if the address change was successful.
 *
 */
bool TFminiPlus::set_i2c_address(uint8_t address) {
    bool result = false;

    send_command(TFMINI_PLUS_SET_I2C_ADDRESS, &address, TFMINI_PLUS_PACK_LENGTH_SET_I2C_ADDRESS);
    do_i2c_wait();

    // Only commit the changes if the correct address is echoed back
    uint8_t response[TFMINI_PLUS_PACK_LENGTH_SET_I2C_ADDRESS];
    if (receive(response, sizeof(response)) and response[3] == address) {
        result = save_settings();
        if (result) {
            _address = address;
        }
    }

    return result;
}

/**
 * Get the firmware version of the lidar.
 *
 * @return: Version information of lidar firmware. [major, minor, revision]
 */
tfminiplus_version_t TFminiPlus::get_version() {
    tfminiplus_version_t version;

    version.revision = 0;
    version.minor = 0;
    version.major = 0;

    send_command(TFMINI_PLUS_GET_VERSION);
    do_i2c_wait();

    uint8_t response[TFMINI_PLUS_PACK_LENGTH_VERSION_RESPONSE];
    if (receive(response, sizeof(response)) and response[TFMINI_PLUS_PACKET_POS_COMMAND] == TFMINI_PLUS_GET_VERSION) {
        version.revision = response[3];
        version.minor = response[4];
        version.major = response[5];
    }

    return version;
}

/**
 * Set the framerate of the lidar.
 * The datasheet recommends that the refresh rate stay below 100Hz when using I2C mode.
 * Changes will not take effect until settings have been saved.
 *
 * @param framerate: Framerate to set the lidar to in Hz.
 * @return: True if the framerate change was successfully received.
 */
bool TFminiPlus::set_framerate(tfminiplus_framerate_t framerate) {
    bool result = false;
    uint8_t argument[2];
    argument[0] = framerate & 0xFF;
    argument[1] = framerate >> 8;
    send_command(TFMINI_PLUS_SET_FRAME_RATE, argument, TFMINI_PLUS_PACK_LENGTH_SET_FRAME_RATE);
    do_i2c_wait();

    uint8_t response[TFMINI_PLUS_PACK_LENGTH_SET_FRAME_RATE];
    if (receive(response, sizeof(response))) {
        if (argument[0] == response[3] and argument[1] == response[4]) result = true;
    }

    return result;
}

/**
 * Set the baudrate of the lidar.
 * Changes will not take effect until settings have been saved.
 *
 * @param baudrate: Baudrate to set UART communications to.
 * @return: True if the baudrate change was successfully received.
 */
bool TFminiPlus::set_baudrate(tfminiplus_baudrate_t baudrate) {
    bool result = false;
    uint8_t argument[4];
    argument[0] = baudrate & 0xFF;
    argument[1] = baudrate >> 8;
    argument[2] = baudrate >> 16;
    argument[3] = baudrate >> 24;

    send_command(TFMINI_PLUS_SET_BAUD_RATE, argument, TFMINI_PLUS_PACK_LENGTH_SET_BAUD_RATE);
    do_i2c_wait();

    // Verify the echoed baudrate
    uint8_t response[TFMINI_PLUS_PACK_LENGTH_SET_BAUD_RATE];
    if (receive(response, sizeof(response))) {
        if (argument[0] == response[3] and argument[1] == response[4] and argument[2] == response[5] and argument[3] == response[6]) result = true;
    }
    return result;
}

/**
 * Set the output format of the lidar.
 * The output format changes the output units or enables a pixhawk-compatible stream.
 * Changes must be saved to take effect.
 *
 * @param format: Format option to change the lidar output to.
 * @result: True if the format change was received succesfully.
 */
bool TFminiPlus::set_output_format(tfminiplus_output_format_t format) {
    bool result = false;

    send_command(TFMINI_PLUS_SET_OUTPUT_FORMAT, (uint8_t *)&format, TFMINI_PLUS_PACK_LENGTH_SET_OUTPUT_FORMAT);
    do_i2c_wait();

    uint8_t response[TFMINI_PLUS_PACK_LENGTH_SET_OUTPUT_FORMAT];
    if (receive(response, sizeof(response))) {
        if (format == response[3]) result = true;
    }

    return result;
}

/**
 * Take a manual reading of the lidar.
 * This method is useful when the framerate has been changed to 0 Hz
 * and readings are taken on an on-demand basis.
 *
 * @param data: Data container to read the lidar output into.
 * @return: True if a data output frame was received successfully.
 */
bool TFminiPlus::read_manual_reading(tfminiplus_data_t &data) {
    trigger_manual_reading();
    return read_data(data);
}

/**
 * Request the lidar to do a manual read.
 * Useful for on-demand measurements.
 */
void TFminiPlus::trigger_manual_reading() { send_command(TFMINI_PLUS_TRIGGER_DETECTION); }

/**
 * Take a manual reading of the lidar.
 * This method is useful when the framerate has been changed to 0 Hz
 * and readings are taken on an on-demand basis.
 *
 * @return: Data container that includes distance, strength, and temperature measurements.
 */
tfminiplus_data_t TFminiPlus::get_manual_reading() {
    tfminiplus_data_t data;
    read_manual_reading(data);
    return data;
}

/**
 * Take a manual reading of the lidar.
 * This method is useful when the framerate has been changed to 0 Hz
 * and readings are taken on an on-demand basis.
 *
 * @return: Distance in cm/mm (depending on configuration).
 */
uint16_t TFminiPlus::get_manual_distance() { return get_manual_reading().distance; }

/**
 * Read a data frame from the lidar.
 * If using the UART interface, frames are continually sent and do not need to be specifically requested.
 *
 * @param data: Data container to read output frame into.
 * @param in_mm_format: True to request the data frame in mm units (I2C only).
 * @return: True if the data frame was received successfully.
 */
bool TFminiPlus::read_data(tfminiplus_data_t &data, bool in_mm_format) {
    uint8_t command = 1 + 5 * in_mm_format;
    if (_communications_mode == TFMINI_PLUS_I2C) send_command(TFMINI_PLUS_GET_DATA, &command, TFMINI_PLUS_PACK_LENGTH_GET_DATA);
    // TODO do_i2c_wait(); // it is redundant
    return read_data_response(data);
}

/**
 * Read a data frame from the lidar.
 * If using the UART interface, frames are continually sent and do not need to be specifically requested.
 *
 * @param in_mm_format: True to request the data frame in mm units (I2C only).
 * @return: Data container that includes distance, strength, and temperature measurements.
 */
tfminiplus_data_t TFminiPlus::get_data(bool in_mm_format) {
    tfminiplus_data_t data;
    read_data(data, in_mm_format);
    return data;
}

/**
 * Read a data frame from the lidar.
 * If using the UART interface, frames are continually sent and do not need to be specifically requested.
 *
 * @param in_mm_format: True to request the data frame in mm units (I2C only).
 * @return: Distance in cm/mm (depending on configuration).
 */
uint16_t TFminiPlus::get_distance(bool in_mm_format) { return get_data(in_mm_format).distance; }

/**
 * Read in a response frame from the lidar.
 * Data frames are read in normally with I2C mode after a request.
 * In UART mode, frames are sent in when available.
 *
 * @param data: Data frame to read the lidar output into.
 * @return: True if the response was received correctly.
 */
bool TFminiPlus::read_data_response(tfminiplus_data_t &data) {
    bool result = false;

    // Grab the data
    uint8_t response[TFMINI_PLUS_PACK_LENGTH_DATA_RESPONSE];
    if (_communications_mode == TFMINI_PLUS_UART) {
        result = uart_receive_data(response, sizeof(response));
    } else {
        result = receive(response, sizeof(response));
    }

    // Put the data into the proper format

    data.distance = response[2] + (response[3] << 8);
    result &= data.distance > 0;

    data.strength = response[4] + (response[5] << 8);
    result &= data.strength > 0 and data.strength != 65535;

    data.temperature = (response[6] + (response[7] << 8)) / 8.0 - 256;
    result &= data.temperature < 100;

    return result;
}

/**
 * Set the IO mode of the lidar.
 * Changes will not occur until settings have been saved.
 * Modes:
 *  0 - Standard: Output is transmitted over UART or I2C.
 *  1 - IO LN/HF - Output pin is high if the measured distance is above the defined distance; otherwise low.
 *  2 - IO HN/LF - Output pin is low if the measured distance is above the defined distance; otherwise high.
 *
 * @param mode: Output mode of the lidar [0-2].
 * @param critical_distance: Threshold distance in cm for the near/far zones when using IO modes.
 * @param hysteresis: Hysteresis in cm used when switching between near/far zones when using IO modes.
 * @return: True if the mode change was received successfully.
 */
bool TFminiPlus::set_io_mode(tfminiplus_mode_t mode, uint16_t critical_distance, uint16_t hysteresis) {
    uint8_t arguments[5];
    arguments[0] = mode;
    arguments[1] = uint8_t(critical_distance);
    arguments[2] = critical_distance >> 8;
    arguments[3] = uint8_t(hysteresis);
    arguments[4] = hysteresis >> 8;

    return send_command(TFMINI_PLUS_SET_IO_MODE, arguments, TFMINI_PLUS_PACK_LENGTH_SET_IO_MODE);
}

/**
 * Set the communication mode for the lidar.
 * Changes are applied immediately.
 * The library will switch to use the mode specified if the change was successfully applied.
 *
 * @param mode: Communication mode. [UART, I2C]
 * @return: True if the settings were saved correctly.
 */
bool TFminiPlus::set_communication_interface(tfminiplus_communication_mode_t mode) {
    bool result;
    send_command(TFMINI_PLUS_SET_COMMUNICATION_INTERFACE, (uint8_t *)&mode, TFMINI_PLUS_PACK_LENGTH_SET_COMMUNICATION_INTERFACE);

    result = save_settings();
    if (result) {
        _communications_mode = mode;
    }
    return result;
}

/**
 * Enable or disable the lidar output.
 * The datasheet is unclear as to whether the lidar will still respond to communication or data requests if output
 * is disabled.
 * Changes must be saved to be applied.
 *
 * @param output_enabled: True to enable output; False to disable.
 * @return: True if the command was received successfully.
 */
bool TFminiPlus::enable_output(bool output_enabled) {
    bool result = false;

    send_command(TFMINI_PLUS_ENABLE_DATA_OUTPUT, (uint8_t *)&output_enabled, TFMINI_PLUS_PACK_LENGTH_ENABLE_DATA_OUTPUT);
    do_i2c_wait();

    uint8_t response[TFMINI_PLUS_PACK_LENGTH_ENABLE_DATA_OUTPUT];
    if (receive(response, sizeof(response))) {
        if (output_enabled == response[3]) result = true;
    }
    return result;
}

/**
 * Apply written settings to the lidar.
 * The datasheet is unclear as to whether saved settings are persistent across power cycles.
 *
 * @return: True is settings were save successfully.
 */
bool TFminiPlus::save_settings() {
    bool result = false;

    send_command(TFMINI_PLUS_SAVE_SETTINGS);
    do_i2c_wait();

    uint8_t response[TFMINI_PLUS_PACK_LENGTH_SAVE_SETTINGS_RESPONSE];
    if (receive(response, sizeof(response))) {
        if (response[3] == 0) {
            result = true;
        }
    }
    return result;
}

/**
 * Reset the lidar.
 * Not sure what this does exactly...
 *
 * @return: True if reset occurred?
 */
bool TFminiPlus::reset_system() {
    bool result = false;

    send_command(TFMINI_PLUS_SYSTEM_RESET);
    do_i2c_wait();

    uint8_t response[TFMINI_PLUS_PACK_LENGTH_SYSTEM_RESET_RESPONSE];
    if (receive(response, sizeof(response))) {
        if (response[3] == 0) result = true;
    }

    return result;
}

/**
 * Reset the lidar's settings back to their factory defaults.
 * The factory reset does not affect the communication mode.
 *
 * @return: True if the factory reset was successful.
 */
bool TFminiPlus::factory_reset() {
    bool result = false;

    send_command(TFMINI_PLUS_RESTORE_FACTORY_SETTINGS);
    do_i2c_wait();

    uint8_t response[TFMINI_PLUS_PACK_LENGTH_RESTORE_FACTORY_SETTINGS_RESPONSE];
    if (receive(response, sizeof(response))) {
        if (response[3] == 0) result = true;
    }
    return result;
}

/**
 * Calculate the effective accuracy of the lidar.
 *
 * @param strength: Strength of the last reading.
 * @param frequency: Framerate of the lidar.
 * @return: Effective accuracy of the lidar in cm.
 */
float TFminiPlus::get_effective_accuracy(uint16_t strength, uint16_t frequency) {
    float x = log10(strength);
    float y = log10(frequency);

    float ranging_accuracy = TFMINI_PLUS_P00 + TFMINI_PLUS_P10 * x + TFMINI_PLUS_P01 * y + TFMINI_PLUS_P20 * x * x + TFMINI_PLUS_P11 * x * y;
    return ranging_accuracy;
}

void TFminiPlus::dump_serial_cache() {
    while (_stream->available()) {
        _stream->read();
    }
    _stream->flush();
}

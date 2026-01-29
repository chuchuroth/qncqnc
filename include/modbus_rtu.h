/* modbus_rtu.h
 *
 * Copyright (C) 2025 NeuraSync, all rights reserved.
 *
 * Direct Modbus RTU (serial) implementation for QNC gripper communication.
 * No external dependencies - uses POSIX termios for serial communication.
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#ifndef QNC_MODBUS_RTU_H
#define QNC_MODBUS_RTU_H

#include <cstring>
#include <stdint.h>
#include <string>
#include <vector>

// POSIX serial port
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <errno.h>

#ifdef ENABLE_MODBUS_RTU_LOGGING
#include <cstdio>
#define RTU_LOG(fmt, ...) printf("[modbus_rtu] " fmt "\n", ##__VA_ARGS__)
#else
#define RTU_LOG(...) (void)0
#endif

// Modbus RTU Function Codes
#define RTU_READ_COILS 0x01
#define RTU_READ_DISCRETE_INPUTS 0x02
#define RTU_READ_HOLDING_REGISTERS 0x03
#define RTU_READ_INPUT_REGISTERS 0x04
#define RTU_WRITE_SINGLE_COIL 0x05
#define RTU_WRITE_SINGLE_REGISTER 0x06
#define RTU_WRITE_MULTIPLE_COILS 0x0F
#define RTU_WRITE_MULTIPLE_REGISTERS 0x10

// Modbus RTU Exception Codes
#define RTU_EX_ILLEGAL_FUNCTION 0x01
#define RTU_EX_ILLEGAL_ADDRESS 0x02
#define RTU_EX_ILLEGAL_VALUE 0x03
#define RTU_EX_SLAVE_DEVICE_FAILURE 0x04
#define RTU_EX_ACKNOWLEDGE 0x05
#define RTU_EX_SLAVE_BUSY 0x06
#define RTU_EX_NEGATIVE_ACK 0x07
#define RTU_EX_MEMORY_PARITY 0x08
#define RTU_EX_GATEWAY_PATH 0x0A
#define RTU_EX_GATEWAY_TARGET 0x0B

// Error codes
#define RTU_SUCCESS 0
#define RTU_ERROR_CONNECTION -1
#define RTU_ERROR_TIMEOUT -2
#define RTU_ERROR_CRC -3
#define RTU_ERROR_EXCEPTION -4
#define RTU_ERROR_INVALID_RESPONSE -5

#define RTU_MAX_FRAME_SIZE 256

/**
 * Modbus RTU Serial Communication Class
 *
 * Direct serial port communication with Modbus RTU devices.
 * Supports all standard Modbus functions for gripper control.
 */
class modbus_rtu
{
public:
    bool err{};
    int err_no{};
    std::string error_msg;

    // Constructor
    modbus_rtu(const std::string& port,
               uint32_t baudrate = 115200,
               uint8_t data_bits = 8,
               uint8_t stop_bits = 1,
               char parity = 'N');
    ~modbus_rtu();

    // Connection management
    bool connect();
    void close();
    bool is_connected() const { return _connected; }

    // Slave ID setter
    void set_slave_id(uint8_t id);
    uint8_t get_slave_id() const { return _slave_id; }

    // Read functions
    int read_coils(uint16_t address, uint16_t count, bool* buffer);
    int read_discrete_inputs(uint16_t address, uint16_t count, bool* buffer);
    int read_holding_registers(uint16_t address, uint16_t count, uint16_t* buffer);
    int read_input_registers(uint16_t address, uint16_t count, uint16_t* buffer);

    // Write functions
    int write_coil(uint16_t address, bool value);
    int write_register(uint16_t address, uint16_t value);
    int write_coils(uint16_t address, uint16_t count, const bool* values);
    int write_registers(uint16_t address, uint16_t count, const uint16_t* values);

    // Configuration
    void set_response_timeout_ms(uint32_t timeout_ms);
    void set_inter_frame_delay_ms(uint32_t delay_ms);

private:
    bool _connected{};
    int _fd{-1};
    std::string _port;
    uint32_t _baudrate;
    uint8_t _data_bits;
    uint8_t _stop_bits;
    char _parity;
    uint8_t _slave_id{1};
    uint32_t _response_timeout_ms{500};
    uint32_t _inter_frame_delay_ms{50};

    struct termios _old_tio{};

    // CRC calculation
    static uint16_t crc16(const uint8_t* buffer, size_t length);

    // Frame building
    size_t build_request_frame(uint8_t* frame, uint8_t function_code,
                               uint16_t address, uint16_t count_or_value);
    size_t build_write_multiple_frame(uint8_t* frame, uint8_t function_code,
                                      uint16_t address, uint16_t count,
                                      const uint8_t* data, size_t data_len);

    // Communication
    int send_frame(const uint8_t* frame, size_t length);
    int receive_frame(uint8_t* frame, size_t max_length);
    int wait_for_data(uint32_t timeout_ms);

    // Response parsing
    int parse_read_response(const uint8_t* frame, size_t length,
                           uint8_t expected_function, uint16_t expected_count,
                           uint8_t* data_out, size_t* data_len_out);
    int parse_write_response(const uint8_t* frame, size_t length,
                            uint8_t expected_function);

    // Error handling
    void set_error(int code, const std::string& msg);
    void clear_error();
    void handle_exception(uint8_t exception_code);
};

/**
 * Constructor
 */
inline modbus_rtu::modbus_rtu(const std::string& port,
                              uint32_t baudrate,
                              uint8_t data_bits,
                              uint8_t stop_bits,
                              char parity)
    : _port(port), _baudrate(baudrate), _data_bits(data_bits),
      _stop_bits(stop_bits), _parity(parity)
{
    err = false;
    err_no = 0;
    error_msg = "";
    _connected = false;
}

/**
 * Destructor
 */
inline modbus_rtu::~modbus_rtu()
{
    if (_connected)
    {
        close();
    }
}

/**
 * Set slave ID for Modbus communication
 */
inline void modbus_rtu::set_slave_id(uint8_t id)
{
    _slave_id = id;
}

/**
 * Set response timeout in milliseconds
 */
inline void modbus_rtu::set_response_timeout_ms(uint32_t timeout_ms)
{
    _response_timeout_ms = timeout_ms;
}

/**
 * Set inter-frame delay in milliseconds
 */
inline void modbus_rtu::set_inter_frame_delay_ms(uint32_t delay_ms)
{
    _inter_frame_delay_ms = delay_ms;
}

/**
 * Open serial port and configure for Modbus RTU
 */
inline bool modbus_rtu::connect()
{
    // Open serial port
    _fd = ::open(_port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (_fd < 0)
    {
        set_error(RTU_ERROR_CONNECTION, "Failed to open " + _port + ": " + strerror(errno));
        return false;
    }

    // Save old terminal settings
    if (tcgetattr(_fd, &_old_tio) != 0)
    {
        set_error(RTU_ERROR_CONNECTION, "Failed to get terminal attributes");
        ::close(_fd);
        _fd = -1;
        return false;
    }

    // Configure new terminal settings
    struct termios tio = {};

    // Control modes
    tio.c_cflag = CS8 | CLOCAL | CREAD; // 8 data bits, ignore modem controls, enable receiver

    // Set data bits
    tio.c_cflag &= ~CSIZE;
    switch (_data_bits)
    {
        case 5: tio.c_cflag |= CS5; break;
        case 6: tio.c_cflag |= CS6; break;
        case 7: tio.c_cflag |= CS7; break;
        case 8: default: tio.c_cflag |= CS8; break;
    }

    // Set stop bits
    if (_stop_bits == 2)
        tio.c_cflag |= CSTOPB;

    // Set parity
    switch (_parity)
    {
        case 'E': // Even parity
        case 'e':
            tio.c_cflag |= PARENB;
            break;
        case 'O': // Odd parity
        case 'o':
            tio.c_cflag |= (PARENB | PARODD);
            break;
        case 'N': // No parity
        case 'n':
        default:
            break;
    }

    // Set baud rate
    speed_t speed;
    switch (_baudrate)
    {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        default:
            set_error(RTU_ERROR_CONNECTION, "Unsupported baud rate");
            ::close(_fd);
            _fd = -1;
            return false;
    }
    cfsetospeed(&tio, speed);
    cfsetispeed(&tio, speed);

    // Input modes - turn off input processing
    tio.c_iflag = 0;
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

    // Output modes - raw output
    tio.c_oflag = 0;

    // Local modes - raw mode
    tio.c_lflag = 0;

    // Control characters
    tio.c_cc[VMIN] = 0;  // Non-blocking read
    tio.c_cc[VTIME] = 0; // No timeout

    // Apply settings
    if (tcsetattr(_fd, TCSANOW, &tio) != 0)
    {
        set_error(RTU_ERROR_CONNECTION, "Failed to set terminal attributes");
        ::close(_fd);
        _fd = -1;
        return false;
    }

    // Flush any existing data
    tcflush(_fd, TCIOFLUSH);

    _connected = true;
    clear_error();
    RTU_LOG("Connected to %s at %u baud", _port.c_str(), _baudrate);
    return true;
}

/**
 * Close serial port
 */
inline void modbus_rtu::close()
{
    if (_fd >= 0)
    {
        // Restore old terminal settings
        tcsetattr(_fd, TCSANOW, &_old_tio);
        ::close(_fd);
        _fd = -1;
    }
    _connected = false;
    RTU_LOG("Disconnected from %s", _port.c_str());
}

/**
 * CRC-16 (Modbus) calculation
 */
inline uint16_t modbus_rtu::crc16(const uint8_t* buffer, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * Build standard Modbus RTU request frame
 */
inline size_t modbus_rtu::build_request_frame(uint8_t* frame, uint8_t function_code,
                                              uint16_t address, uint16_t count_or_value)
{
    frame[0] = _slave_id;
    frame[1] = function_code;
    frame[2] = (address >> 8) & 0xFF;   // Address high byte
    frame[3] = address & 0xFF;          // Address low byte
    frame[4] = (count_or_value >> 8) & 0xFF;  // Count/Value high byte
    frame[5] = count_or_value & 0xFF;         // Count/Value low byte

    // Calculate and append CRC
    uint16_t crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;        // CRC low byte
    frame[7] = (crc >> 8) & 0xFF; // CRC high byte

    return 8;
}

/**
 * Build write multiple registers/coils frame
 */
inline size_t modbus_rtu::build_write_multiple_frame(uint8_t* frame, uint8_t function_code,
                                                     uint16_t address, uint16_t count,
                                                     const uint8_t* data, size_t data_len)
{
    frame[0] = _slave_id;
    frame[1] = function_code;
    frame[2] = (address >> 8) & 0xFF;
    frame[3] = address & 0xFF;
    frame[4] = (count >> 8) & 0xFF;
    frame[5] = count & 0xFF;
    frame[6] = data_len;  // Byte count

    // Copy data
    memcpy(&frame[7], data, data_len);

    // Calculate and append CRC
    uint16_t crc = crc16(frame, 7 + data_len);
    frame[7 + data_len] = crc & 0xFF;
    frame[8 + data_len] = (crc >> 8) & 0xFF;

    return 9 + data_len;
}

/**
 * Send frame over serial port
 */
inline int modbus_rtu::send_frame(const uint8_t* frame, size_t length)
{
    if (!_connected || _fd < 0)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Flush receive buffer before sending
    tcflush(_fd, TCIFLUSH);

    // Send frame
    ssize_t written = write(_fd, frame, length);
    if (written < 0 || (size_t)written != length)
    {
        set_error(RTU_ERROR_CONNECTION, "Write failed: " + std::string(strerror(errno)));
        return RTU_ERROR_CONNECTION;
    }

    // Wait for transmission to complete
    tcdrain(_fd);

    RTU_LOG("Sent %zu bytes", length);
    return RTU_SUCCESS;
}

/**
 * Wait for data with timeout
 */
inline int modbus_rtu::wait_for_data(uint32_t timeout_ms)
{
    fd_set readfds;
    struct timeval timeout;

    FD_ZERO(&readfds);
    FD_SET(_fd, &readfds);

    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(_fd + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret < 0)
    {
        set_error(RTU_ERROR_CONNECTION, "Select failed: " + std::string(strerror(errno)));
        return RTU_ERROR_CONNECTION;
    }
    else if (ret == 0)
    {
        set_error(RTU_ERROR_TIMEOUT, "Response timeout");
        return RTU_ERROR_TIMEOUT;
    }

    return RTU_SUCCESS;
}

/**
 * Receive frame from serial port
 */
inline int modbus_rtu::receive_frame(uint8_t* frame, size_t max_length)
{
    if (!_connected || _fd < 0)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Wait for first byte
    int ret = wait_for_data(_response_timeout_ms);
    if (ret != RTU_SUCCESS)
        return ret;

    // Read available data
    size_t total_received = 0;
    while (total_received < max_length)
    {
        ssize_t n = read(_fd, frame + total_received, max_length - total_received);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                break; // No more data available
            set_error(RTU_ERROR_CONNECTION, "Read failed: " + std::string(strerror(errno)));
            return RTU_ERROR_CONNECTION;
        }
        else if (n == 0)
        {
            break; // No more data
        }

        total_received += n;

        // Check if we have enough data
        if (total_received >= 5) // Minimum response: slave_id + func + 2 data + 2 CRC
        {
            // Wait a bit for remaining data
            usleep(5000); // 5ms

            // Try to read more
            int bytes_available = 0;
            ioctl(_fd, FIONREAD, &bytes_available);
            if (bytes_available == 0)
                break; // No more data coming
        }
    }

    if (total_received < 5)
    {
        set_error(RTU_ERROR_INVALID_RESPONSE, "Response too short");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Verify CRC
    uint16_t received_crc = frame[total_received - 2] | (frame[total_received - 1] << 8);
    uint16_t calculated_crc = crc16(frame, total_received - 2);

    if (received_crc != calculated_crc)
    {
        set_error(RTU_ERROR_CRC, "CRC mismatch");
        return RTU_ERROR_CRC;
    }

    RTU_LOG("Received %zu bytes", total_received);
    return (int)total_received;
}

/**
 * Parse read response
 */
inline int modbus_rtu::parse_read_response(const uint8_t* frame, size_t length,
                                           uint8_t expected_function, uint16_t expected_count,
                                           uint8_t* data_out, size_t* data_len_out)
{
    // Check slave ID
    if (frame[0] != _slave_id)
    {
        set_error(RTU_ERROR_INVALID_RESPONSE, "Wrong slave ID in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Check for exception
    if (frame[1] & 0x80)
    {
        handle_exception(frame[2]);
        return RTU_ERROR_EXCEPTION;
    }

    // Check function code
    if (frame[1] != expected_function)
    {
        set_error(RTU_ERROR_INVALID_RESPONSE, "Wrong function code in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Get byte count
    uint8_t byte_count = frame[2];

    // Verify length
    if (length < (size_t)(3 + byte_count + 2)) // slave_id + func + byte_count + data + CRC
    {
        set_error(RTU_ERROR_INVALID_RESPONSE, "Response length mismatch");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Copy data
    *data_len_out = byte_count;
    memcpy(data_out, &frame[3], byte_count);

    return RTU_SUCCESS;
}

/**
 * Parse write response
 */
inline int modbus_rtu::parse_write_response(const uint8_t* frame, size_t length,
                                            uint8_t expected_function)
{
    // Check minimum length
    if (length < 8) // slave_id + func + address(2) + value(2) + CRC(2)
    {
        set_error(RTU_ERROR_INVALID_RESPONSE, "Write response too short");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Check slave ID
    if (frame[0] != _slave_id)
    {
        set_error(RTU_ERROR_INVALID_RESPONSE, "Wrong slave ID in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Check for exception
    if (frame[1] & 0x80)
    {
        handle_exception(frame[2]);
        return RTU_ERROR_EXCEPTION;
    }

    // Check function code
    if (frame[1] != expected_function)
    {
        set_error(RTU_ERROR_INVALID_RESPONSE, "Wrong function code in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    return RTU_SUCCESS;
}

/**
 * Read holding registers (function 0x03)
 */
inline int modbus_rtu::read_holding_registers(uint16_t address, uint16_t count, uint16_t* buffer)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Build and send request
    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_request_frame(request, RTU_READ_HOLDING_REGISTERS, address, count);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    // Wait inter-frame delay
    usleep(_inter_frame_delay_ms * 1000);

    // Receive response
    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    // Parse response
    uint8_t data[RTU_MAX_FRAME_SIZE];
    size_t data_len = 0;
    ret = parse_read_response(response, response_len, RTU_READ_HOLDING_REGISTERS, count, data, &data_len);
    if (ret != RTU_SUCCESS)
        return ret;

    // Convert bytes to uint16_t
    for (uint16_t i = 0; i < count; i++)
    {
        buffer[i] = (data[i * 2] << 8) | data[i * 2 + 1];
    }

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Read input registers (function 0x04)
 */
inline int modbus_rtu::read_input_registers(uint16_t address, uint16_t count, uint16_t* buffer)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_request_frame(request, RTU_READ_INPUT_REGISTERS, address, count);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    usleep(_inter_frame_delay_ms * 1000);

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    uint8_t data[RTU_MAX_FRAME_SIZE];
    size_t data_len = 0;
    ret = parse_read_response(response, response_len, RTU_READ_INPUT_REGISTERS, count, data, &data_len);
    if (ret != RTU_SUCCESS)
        return ret;

    for (uint16_t i = 0; i < count; i++)
    {
        buffer[i] = (data[i * 2] << 8) | data[i * 2 + 1];
    }

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Read coils (function 0x01)
 */
inline int modbus_rtu::read_coils(uint16_t address, uint16_t count, bool* buffer)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_request_frame(request, RTU_READ_COILS, address, count);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    usleep(_inter_frame_delay_ms * 1000);

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    uint8_t data[RTU_MAX_FRAME_SIZE];
    size_t data_len = 0;
    ret = parse_read_response(response, response_len, RTU_READ_COILS, count, data, &data_len);
    if (ret != RTU_SUCCESS)
        return ret;

    // Convert bits to bool array
    for (uint16_t i = 0; i < count; i++)
    {
        buffer[i] = (data[i / 8] >> (i % 8)) & 0x01;
    }

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Read discrete inputs (function 0x02)
 */
inline int modbus_rtu::read_discrete_inputs(uint16_t address, uint16_t count, bool* buffer)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_request_frame(request, RTU_READ_DISCRETE_INPUTS, address, count);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    usleep(_inter_frame_delay_ms * 1000);

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    uint8_t data[RTU_MAX_FRAME_SIZE];
    size_t data_len = 0;
    ret = parse_read_response(response, response_len, RTU_READ_DISCRETE_INPUTS, count, data, &data_len);
    if (ret != RTU_SUCCESS)
        return ret;

    for (uint16_t i = 0; i < count; i++)
    {
        buffer[i] = (data[i / 8] >> (i % 8)) & 0x01;
    }

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Write single register (function 0x06)
 */
inline int modbus_rtu::write_register(uint16_t address, uint16_t value)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_request_frame(request, RTU_WRITE_SINGLE_REGISTER, address, value);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    usleep(_inter_frame_delay_ms * 1000);

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    ret = parse_write_response(response, response_len, RTU_WRITE_SINGLE_REGISTER);
    if (ret != RTU_SUCCESS)
        return ret;

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Write single coil (function 0x05)
 */
inline int modbus_rtu::write_coil(uint16_t address, bool value)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint16_t coil_value = value ? 0xFF00 : 0x0000;

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_request_frame(request, RTU_WRITE_SINGLE_COIL, address, coil_value);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    usleep(_inter_frame_delay_ms * 1000);

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    ret = parse_write_response(response, response_len, RTU_WRITE_SINGLE_COIL);
    if (ret != RTU_SUCCESS)
        return ret;

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Write multiple registers (function 0x10)
 */
inline int modbus_rtu::write_registers(uint16_t address, uint16_t count, const uint16_t* values)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Convert uint16_t array to bytes
    uint8_t data[RTU_MAX_FRAME_SIZE];
    for (uint16_t i = 0; i < count; i++)
    {
        data[i * 2] = (values[i] >> 8) & 0xFF;
        data[i * 2 + 1] = values[i] & 0xFF;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_write_multiple_frame(request, RTU_WRITE_MULTIPLE_REGISTERS,
                                                    address, count, data, count * 2);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    usleep(_inter_frame_delay_ms * 1000);

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    ret = parse_write_response(response, response_len, RTU_WRITE_MULTIPLE_REGISTERS);
    if (ret != RTU_SUCCESS)
        return ret;

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Write multiple coils (function 0x0F)
 */
inline int modbus_rtu::write_coils(uint16_t address, uint16_t count, const bool* values)
{
    if (!_connected)
    {
        set_error(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Convert bool array to packed bits
    size_t byte_count = (count + 7) / 8;
    uint8_t data[RTU_MAX_FRAME_SIZE];
    memset(data, 0, byte_count);

    for (uint16_t i = 0; i < count; i++)
    {
        if (values[i])
        {
            data[i / 8] |= (1 << (i % 8));
        }
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = build_write_multiple_frame(request, RTU_WRITE_MULTIPLE_COILS,
                                                    address, count, data, byte_count);

    int ret = send_frame(request, request_len);
    if (ret != RTU_SUCCESS)
        return ret;

    usleep(_inter_frame_delay_ms * 1000);

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receive_frame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0)
        return response_len;

    ret = parse_write_response(response, response_len, RTU_WRITE_MULTIPLE_COILS);
    if (ret != RTU_SUCCESS)
        return ret;

    clear_error();
    return RTU_SUCCESS;
}

/**
 * Set error state
 */
inline void modbus_rtu::set_error(int code, const std::string& msg)
{
    err = true;
    err_no = code;
    error_msg = msg;
    RTU_LOG("ERROR: %s (code %d)", msg.c_str(), code);
}

/**
 * Clear error state
 */
inline void modbus_rtu::clear_error()
{
    err = false;
    err_no = 0;
    error_msg = "";
}

/**
 * Handle Modbus exception
 */
inline void modbus_rtu::handle_exception(uint8_t exception_code)
{
    std::string msg;
    switch (exception_code)
    {
        case RTU_EX_ILLEGAL_FUNCTION:
            msg = "Illegal Function";
            break;
        case RTU_EX_ILLEGAL_ADDRESS:
            msg = "Illegal Data Address";
            break;
        case RTU_EX_ILLEGAL_VALUE:
            msg = "Illegal Data Value";
            break;
        case RTU_EX_SLAVE_DEVICE_FAILURE:
            msg = "Slave Device Failure";
            break;
        case RTU_EX_ACKNOWLEDGE:
            msg = "Acknowledge";
            break;
        case RTU_EX_SLAVE_BUSY:
            msg = "Slave Device Busy";
            break;
        case RTU_EX_NEGATIVE_ACK:
            msg = "Negative Acknowledge";
            break;
        case RTU_EX_MEMORY_PARITY:
            msg = "Memory Parity Error";
            break;
        case RTU_EX_GATEWAY_PATH:
            msg = "Gateway Path Unavailable";
            break;
        case RTU_EX_GATEWAY_TARGET:
            msg = "Gateway Target Device Failed to Respond";
            break;
        default:
            msg = "Unknown Exception (code " + std::to_string(exception_code) + ")";
            break;
    }
    set_error(RTU_ERROR_EXCEPTION, "Modbus Exception: " + msg);
}

#endif // QNC_MODBUS_RTU_H

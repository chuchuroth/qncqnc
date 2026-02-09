#include "modbus_rtu_esp32.h"
#include <driver/uart.h>

// ============================================================================
// Construction / Destruction
// ============================================================================

ModbusRTU_ESP32::ModbusRTU_ESP32(uint8_t uart_num, int tx_pin, int rx_pin,
                                 int de_pin, uint32_t baudrate, uint8_t slave_id)
    : _serial(nullptr)
    , _uart_num(uart_num)
    , _tx_pin(tx_pin)
    , _rx_pin(rx_pin)
    , _de_pin(de_pin)
    , _baudrate(baudrate)
    , _slave_id(slave_id)
    , _connected(false)
    , _response_timeout_ms(500)
    , _inter_frame_delay_ms(50)
    , _err(false)
    , _err_code(0)
{
    _err_msg[0] = '\0';
}

ModbusRTU_ESP32::~ModbusRTU_ESP32()
{
    if (_connected) {
        end();
    }
}

// ============================================================================
// Connection — replaces POSIX open()/termios (modbus_rtu.h:211-315)
// ============================================================================

bool ModbusRTU_ESP32::begin()
{
    _serial = new HardwareSerial(_uart_num);
    _serial->begin(_baudrate, SERIAL_8N1, _rx_pin, _tx_pin);

    if (_de_pin >= 0) {
        // Use ESP32 hardware RS485 half-duplex mode.
        // This automatically drives DE high during TX, low during RX,
        // replacing the manual digitalWrite() approach.
        _serial->setPins(_rx_pin, _tx_pin, -1, _de_pin);
        uart_set_mode((uart_port_t)_uart_num, UART_MODE_RS485_HALF_DUPLEX);
    }

    // Flush any boot noise
    delay(100);
    while (_serial->available()) _serial->read();

    _connected = true;
    clearError();
    Serial.printf("[ModbusRTU] Connected on UART%d (TX=%d, RX=%d, DE=%d) at %u baud\n",
                  _uart_num, _tx_pin, _rx_pin, _de_pin, _baudrate);
    return true;
}

void ModbusRTU_ESP32::end()
{
    if (_serial) {
        _serial->end();
        delete _serial;
        _serial = nullptr;
    }
    _connected = false;
}

// ============================================================================
// CRC-16 Modbus — verbatim from modbus_rtu.h:336-355
// ============================================================================

uint16_t ModbusRTU_ESP32::crc16(const uint8_t* buffer, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// Frame Building — verbatim logic from modbus_rtu.h:360-402
// ============================================================================

size_t ModbusRTU_ESP32::buildRequestFrame(uint8_t* frame, uint8_t fc,
                                          uint16_t address, uint16_t count_or_value)
{
    frame[0] = _slave_id;
    frame[1] = fc;
    frame[2] = (address >> 8) & 0xFF;
    frame[3] = address & 0xFF;
    frame[4] = (count_or_value >> 8) & 0xFF;
    frame[5] = count_or_value & 0xFF;

    uint16_t crc = crc16(frame, 6);
    frame[6] = crc & 0xFF;        // CRC low byte
    frame[7] = (crc >> 8) & 0xFF; // CRC high byte

    return 8;
}

size_t ModbusRTU_ESP32::buildWriteMultipleFrame(uint8_t* frame, uint8_t fc,
                                                uint16_t address, uint16_t count,
                                                const uint8_t* data, size_t data_len)
{
    frame[0] = _slave_id;
    frame[1] = fc;
    frame[2] = (address >> 8) & 0xFF;
    frame[3] = address & 0xFF;
    frame[4] = (count >> 8) & 0xFF;
    frame[5] = count & 0xFF;
    frame[6] = data_len;

    memcpy(&frame[7], data, data_len);

    uint16_t crc = crc16(frame, 7 + data_len);
    frame[7 + data_len] = crc & 0xFF;
    frame[8 + data_len] = (crc >> 8) & 0xFF;

    return 9 + data_len;
}

// ============================================================================
// Communication — replaces POSIX write/read/select (modbus_rtu.h:407-529)
// ============================================================================

int ModbusRTU_ESP32::sendFrame(const uint8_t* frame, size_t length)
{
    if (!_connected || !_serial) {
        setError(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Flush RX buffer before sending (equivalent to tcflush(TCIFLUSH))
    while (_serial->available()) _serial->read();

    // Send frame
    size_t written = _serial->write(frame, length);
    if (written != length) {
        setError(RTU_ERROR_CONNECTION, "Write failed");
        return RTU_ERROR_CONNECTION;
    }

    // Wait for TX to complete (equivalent to tcdrain())
    _serial->flush();

    return RTU_SUCCESS;
}

int ModbusRTU_ESP32::receiveFrame(uint8_t* frame, size_t max_length)
{
    if (!_connected || !_serial) {
        setError(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Wait for first byte (replaces select() with timeout)
    uint32_t start = millis();
    while (!_serial->available()) {
        if (millis() - start > _response_timeout_ms) {
            setError(RTU_ERROR_TIMEOUT, "Response timeout");
            return RTU_ERROR_TIMEOUT;
        }
        vTaskDelay(1); // Yield to other tasks
    }

    // Read all available data with inter-byte timeout
    // (replaces the read() + ioctl(FIONREAD) loop from modbus_rtu.h:480-509)
    size_t total = 0;
    uint32_t last_rx = millis();
    while (total < max_length) {
        if (_serial->available()) {
            frame[total++] = _serial->read();
            last_rx = millis();
        } else if (millis() - last_rx > 5) {
            // 5ms inter-byte timeout (matches usleep(5000) in original)
            break;
        } else {
            delayMicroseconds(100);
        }
    }

    if (total < 5) { // Minimum: slave_id + func + byte_count + CRC(2)
        setError(RTU_ERROR_INVALID_RESPONSE, "Response too short");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // CRC verification (identical to modbus_rtu.h:518-525)
    uint16_t received_crc = frame[total - 2] | (frame[total - 1] << 8);
    uint16_t calculated_crc = crc16(frame, total - 2);
    if (received_crc != calculated_crc) {
        setError(RTU_ERROR_CRC, "CRC mismatch");
        return RTU_ERROR_CRC;
    }

    return (int)total;
}

// ============================================================================
// Response Parsing — verbatim logic from modbus_rtu.h:534-611
// ============================================================================

int ModbusRTU_ESP32::parseReadResponse(const uint8_t* frame, size_t length,
                                       uint8_t expected_fc, uint16_t expected_count,
                                       uint8_t* data_out, size_t* data_len_out)
{
    // Check slave ID
    if (frame[0] != _slave_id) {
        setError(RTU_ERROR_INVALID_RESPONSE, "Wrong slave ID in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Check for exception
    if (frame[1] & 0x80) {
        handleException(frame[2]);
        return RTU_ERROR_EXCEPTION;
    }

    // Check function code
    if (frame[1] != expected_fc) {
        setError(RTU_ERROR_INVALID_RESPONSE, "Wrong function code in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Get byte count
    uint8_t byte_count = frame[2];

    // Verify length
    if (length < (size_t)(3 + byte_count + 2)) {
        setError(RTU_ERROR_INVALID_RESPONSE, "Response length mismatch");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    // Copy data
    *data_len_out = byte_count;
    memcpy(data_out, &frame[3], byte_count);

    return RTU_SUCCESS;
}

int ModbusRTU_ESP32::parseWriteResponse(const uint8_t* frame, size_t length,
                                        uint8_t expected_fc)
{
    if (length < 8) {
        setError(RTU_ERROR_INVALID_RESPONSE, "Write response too short");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    if (frame[0] != _slave_id) {
        setError(RTU_ERROR_INVALID_RESPONSE, "Wrong slave ID in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    if (frame[1] & 0x80) {
        handleException(frame[2]);
        return RTU_ERROR_EXCEPTION;
    }

    if (frame[1] != expected_fc) {
        setError(RTU_ERROR_INVALID_RESPONSE, "Wrong function code in response");
        return RTU_ERROR_INVALID_RESPONSE;
    }

    return RTU_SUCCESS;
}

// ============================================================================
// Read/Write Operations — ported from modbus_rtu.h:616-887
// ============================================================================

int ModbusRTU_ESP32::readHoldingRegisters(uint16_t address, uint16_t count, uint16_t* buffer)
{
    if (!_connected) {
        setError(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = buildRequestFrame(request, RTU_READ_HOLDING_REGISTERS, address, count);

    int ret = sendFrame(request, request_len);
    if (ret != RTU_SUCCESS) return ret;

    vTaskDelay(pdMS_TO_TICKS(_inter_frame_delay_ms));

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receiveFrame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0) return response_len;

    uint8_t data[RTU_MAX_FRAME_SIZE];
    size_t data_len = 0;
    ret = parseReadResponse(response, response_len, RTU_READ_HOLDING_REGISTERS, count, data, &data_len);
    if (ret != RTU_SUCCESS) return ret;

    // Convert bytes to uint16_t (big-endian)
    for (uint16_t i = 0; i < count; i++) {
        buffer[i] = (data[i * 2] << 8) | data[i * 2 + 1];
    }

    clearError();
    return RTU_SUCCESS;
}

int ModbusRTU_ESP32::readInputRegisters(uint16_t address, uint16_t count, uint16_t* buffer)
{
    if (!_connected) {
        setError(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = buildRequestFrame(request, RTU_READ_INPUT_REGISTERS, address, count);

    int ret = sendFrame(request, request_len);
    if (ret != RTU_SUCCESS) return ret;

    vTaskDelay(pdMS_TO_TICKS(_inter_frame_delay_ms));

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receiveFrame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0) return response_len;

    uint8_t data[RTU_MAX_FRAME_SIZE];
    size_t data_len = 0;
    ret = parseReadResponse(response, response_len, RTU_READ_INPUT_REGISTERS, count, data, &data_len);
    if (ret != RTU_SUCCESS) return ret;

    for (uint16_t i = 0; i < count; i++) {
        buffer[i] = (data[i * 2] << 8) | data[i * 2 + 1];
    }

    clearError();
    return RTU_SUCCESS;
}

int ModbusRTU_ESP32::writeRegister(uint16_t address, uint16_t value)
{
    if (!_connected) {
        setError(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = buildRequestFrame(request, RTU_WRITE_SINGLE_REGISTER, address, value);

    int ret = sendFrame(request, request_len);
    if (ret != RTU_SUCCESS) return ret;

    vTaskDelay(pdMS_TO_TICKS(_inter_frame_delay_ms));

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receiveFrame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0) return response_len;

    ret = parseWriteResponse(response, response_len, RTU_WRITE_SINGLE_REGISTER);
    if (ret != RTU_SUCCESS) return ret;

    clearError();
    return RTU_SUCCESS;
}

int ModbusRTU_ESP32::writeRegisters(uint16_t address, uint16_t count, const uint16_t* values)
{
    if (!_connected) {
        setError(RTU_ERROR_CONNECTION, "Not connected");
        return RTU_ERROR_CONNECTION;
    }

    // Convert uint16_t array to bytes (big-endian)
    uint8_t data[RTU_MAX_FRAME_SIZE];
    for (uint16_t i = 0; i < count; i++) {
        data[i * 2] = (values[i] >> 8) & 0xFF;
        data[i * 2 + 1] = values[i] & 0xFF;
    }

    uint8_t request[RTU_MAX_FRAME_SIZE];
    size_t request_len = buildWriteMultipleFrame(request, RTU_WRITE_MULTIPLE_REGISTERS,
                                                 address, count, data, count * 2);

    int ret = sendFrame(request, request_len);
    if (ret != RTU_SUCCESS) return ret;

    vTaskDelay(pdMS_TO_TICKS(_inter_frame_delay_ms));

    uint8_t response[RTU_MAX_FRAME_SIZE];
    int response_len = receiveFrame(response, RTU_MAX_FRAME_SIZE);
    if (response_len < 0) return response_len;

    ret = parseWriteResponse(response, response_len, RTU_WRITE_MULTIPLE_REGISTERS);
    if (ret != RTU_SUCCESS) return ret;

    clearError();
    return RTU_SUCCESS;
}

// ============================================================================
// Error Handling — ported from modbus_rtu.h:939-1000
// ============================================================================

void ModbusRTU_ESP32::setError(int code, const char* msg)
{
    _err = true;
    _err_code = code;
    strncpy(_err_msg, msg, sizeof(_err_msg) - 1);
    _err_msg[sizeof(_err_msg) - 1] = '\0';
}

void ModbusRTU_ESP32::clearError()
{
    _err = false;
    _err_code = 0;
    _err_msg[0] = '\0';
}

void ModbusRTU_ESP32::handleException(uint8_t exception_code)
{
    const char* msg;
    switch (exception_code) {
        case RTU_EX_ILLEGAL_FUNCTION:       msg = "Illegal Function"; break;
        case RTU_EX_ILLEGAL_ADDRESS:        msg = "Illegal Data Address"; break;
        case RTU_EX_ILLEGAL_VALUE:          msg = "Illegal Data Value"; break;
        case RTU_EX_SLAVE_DEVICE_FAILURE:   msg = "Slave Device Failure"; break;
        case RTU_EX_ACKNOWLEDGE:            msg = "Acknowledge"; break;
        case RTU_EX_SLAVE_BUSY:             msg = "Slave Device Busy"; break;
        case RTU_EX_NEGATIVE_ACK:           msg = "Negative Acknowledge"; break;
        case RTU_EX_MEMORY_PARITY:          msg = "Memory Parity Error"; break;
        case RTU_EX_GATEWAY_PATH:           msg = "Gateway Path Unavailable"; break;
        case RTU_EX_GATEWAY_TARGET:         msg = "Gateway Target Failed"; break;
        default:                            msg = "Unknown Exception"; break;
    }

    char full_msg[128];
    snprintf(full_msg, sizeof(full_msg), "Modbus Exception: %s (0x%02X)", msg, exception_code);
    setError(RTU_ERROR_EXCEPTION, full_msg);
}

#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

// Modbus RTU Function Codes (from modbus_rtu.h:37-44)
#define RTU_READ_COILS                0x01
#define RTU_READ_DISCRETE_INPUTS      0x02
#define RTU_READ_HOLDING_REGISTERS    0x03
#define RTU_READ_INPUT_REGISTERS      0x04
#define RTU_WRITE_SINGLE_COIL         0x05
#define RTU_WRITE_SINGLE_REGISTER     0x06
#define RTU_WRITE_MULTIPLE_COILS      0x0F
#define RTU_WRITE_MULTIPLE_REGISTERS  0x10

// Modbus RTU Exception Codes (from modbus_rtu.h:47-57)
#define RTU_EX_ILLEGAL_FUNCTION       0x01
#define RTU_EX_ILLEGAL_ADDRESS        0x02
#define RTU_EX_ILLEGAL_VALUE          0x03
#define RTU_EX_SLAVE_DEVICE_FAILURE   0x04
#define RTU_EX_ACKNOWLEDGE            0x05
#define RTU_EX_SLAVE_BUSY             0x06
#define RTU_EX_NEGATIVE_ACK           0x07
#define RTU_EX_MEMORY_PARITY          0x08
#define RTU_EX_GATEWAY_PATH           0x0A
#define RTU_EX_GATEWAY_TARGET         0x0B

// Error codes (from modbus_rtu.h:59-64)
#define RTU_SUCCESS                    0
#define RTU_ERROR_CONNECTION          -1
#define RTU_ERROR_TIMEOUT             -2
#define RTU_ERROR_CRC                 -3
#define RTU_ERROR_EXCEPTION           -4
#define RTU_ERROR_INVALID_RESPONSE    -5

#define RTU_MAX_FRAME_SIZE            256

class ModbusRTU_ESP32 {
public:
    ModbusRTU_ESP32(uint8_t uart_num, int tx_pin, int rx_pin, int de_pin,
                    uint32_t baudrate, uint8_t slave_id);
    ~ModbusRTU_ESP32();

    bool begin();
    void end();
    bool isConnected() const { return _connected; }

    void setSlaveId(uint8_t id) { _slave_id = id; }
    uint8_t getSlaveId() const { return _slave_id; }

    void setResponseTimeoutMs(uint32_t ms) { _response_timeout_ms = ms; }
    void setInterFrameDelayMs(uint32_t ms) { _inter_frame_delay_ms = ms; }

    // Read operations
    int readHoldingRegisters(uint16_t address, uint16_t count, uint16_t* buffer);
    int readInputRegisters(uint16_t address, uint16_t count, uint16_t* buffer);

    // Write operations
    int writeRegister(uint16_t address, uint16_t value);
    int writeRegisters(uint16_t address, uint16_t count, const uint16_t* values);

    // Error state
    bool hasError() const { return _err; }
    int getErrorCode() const { return _err_code; }
    const char* getErrorMessage() const { return _err_msg; }

private:
    HardwareSerial* _serial;
    uint8_t  _uart_num;
    int      _tx_pin, _rx_pin, _de_pin;
    uint32_t _baudrate;
    uint8_t  _slave_id;
    bool     _connected;
    uint32_t _response_timeout_ms;
    uint32_t _inter_frame_delay_ms;

    bool     _err;
    int      _err_code;
    char     _err_msg[128];

    // CRC-16 Modbus (verbatim from modbus_rtu.h:336-355)
    static uint16_t crc16(const uint8_t* buffer, size_t length);

    // Frame building (logic from modbus_rtu.h:360-402)
    size_t buildRequestFrame(uint8_t* frame, uint8_t fc,
                             uint16_t address, uint16_t count_or_value);
    size_t buildWriteMultipleFrame(uint8_t* frame, uint8_t fc,
                                   uint16_t address, uint16_t count,
                                   const uint8_t* data, size_t data_len);

    // Communication (replaces POSIX read/write/select)
    int sendFrame(const uint8_t* frame, size_t length);
    int receiveFrame(uint8_t* frame, size_t max_length);

    // Response parsing (logic from modbus_rtu.h:534-611)
    int parseReadResponse(const uint8_t* frame, size_t length,
                          uint8_t expected_fc, uint16_t expected_count,
                          uint8_t* data_out, size_t* data_len_out);
    int parseWriteResponse(const uint8_t* frame, size_t length,
                           uint8_t expected_fc);

    void setError(int code, const char* msg);
    void clearError();
    void handleException(uint8_t exception_code);
};

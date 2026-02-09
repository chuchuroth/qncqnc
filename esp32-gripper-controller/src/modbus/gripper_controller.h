#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include "config.h"
#include "modbus/modbus_rtu_esp32.h"

// Thread-safe shared state (read by GUI task)
struct GripperState {
    uint16_t init_state;        // 0=not_init, 1=initializing, 2=ready
    uint16_t gripper_state;     // 0=moving, 1=reached, 2=caught, 3=dropped
    uint16_t current_position;  // 0-1000
    uint16_t target_position;   // 0-1000 (last commanded)
    uint16_t force_percent;     // 20-100
    uint16_t speed_percent;     // 1-100
    bool     online;            // true if Modbus communication is working
    uint32_t last_update_ms;    // millis() of last successful read
    int      last_error;        // RTU_SUCCESS or error code
    uint32_t error_count;       // Cumulative error counter
    uint32_t success_count;     // Cumulative success counter
};

// Command queue entry (GUI task -> Modbus task)
enum class GripperCommand : uint8_t {
    NONE = 0,
    INIT,
    FULL_INIT,
    SET_POSITION,
    SET_FORCE,
    SET_SPEED,
    OPEN,
    CLOSE
};

struct GripperCommandMsg {
    GripperCommand cmd;
    uint16_t value;  // Position, force%, or speed% depending on cmd
};

class GripperController {
public:
    GripperController(ModbusRTU_ESP32& modbus);

    // Initialize FreeRTOS primitives (must be called from setup())
    bool begin();

    // High-level commands (thread-safe, can be called from any task)
    void init(bool full_calibration = false);
    void setPosition(uint16_t position);  // 0-1000
    void setForce(uint16_t percent);      // 20-100
    void setSpeed(uint16_t percent);      // 1-100
    void open();
    void close();

    // Poll cycle (called from modbusTask loop)
    void poll();

    // Thread-safe state access — returns a snapshot
    GripperState getState();

private:
    ModbusRTU_ESP32& _modbus;
    GripperState     _state;
    SemaphoreHandle_t _stateMutex;
    QueueHandle_t     _cmdQueue;

    void processCommand(const GripperCommandMsg& cmd);
    void readStatus();
    void sendCommand(GripperCommand cmd, uint16_t value = 0);
};

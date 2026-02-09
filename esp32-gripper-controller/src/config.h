#pragma once
#include <Arduino.h>

// ============================================================================
// Hardware Pins — ESP32-2432S028R with external RS485 transceiver
// ============================================================================

// RS485 transceiver (MAX3485/XY-017) connected to UART2
#define RS485_TX_PIN        22
#define RS485_RX_PIN        27
#define RS485_DE_RE_PIN     32    // Direction control (HIGH=TX, LOW=RX)
                                   // Set to -1 if using auto-direction module
#define RS485_UART_NUM      2     // ESP32 UART peripheral number

// Display backlight
#define TFT_BACKLIGHT_PIN   21

// ============================================================================
// Modbus RTU Communication Parameters
// ============================================================================

#define MODBUS_BAUDRATE              115200
#define MODBUS_SLAVE_ID              1
#define MODBUS_RESPONSE_TIMEOUT_MS   500   // Max wait for slave response
#define MODBUS_INTER_FRAME_DELAY_MS  100   // Delay between transactions
#define MODBUS_MAX_RETRIES           3     // Retry count on failure

// ============================================================================
// DH AG-95 Gripper Register Map
// Source: gripper_control.cpp:27-34, gripper_control.py:34-36,
//         DH_AG95_descriptor.json
// ============================================================================

// Holding Registers (FC 0x06 write, FC 0x03 read)
#define REG_INIT_COMMAND        256   // 0x0100 — Write 1=init, 0xA5=full cal
#define REG_FORCE_PERCENT       257   // 0x0101 — Force 20-100%
#define REG_SPEED_PERCENT       258   // 0x0102 — Speed 1-100%
#define REG_TARGET_POSITION     259   // 0x0103 — Position 0(open)-1000(closed)

// Input Registers (FC 0x04 read-only)
#define REG_INIT_STATE          512   // 0x0200 — 0=not_init, 1=initializing, 2=ready
#define REG_GRIPPER_STATE       513   // 0x0201 — 0=moving, 1=reached, 2=caught, 3=dropped
#define REG_CURRENT_POSITION    514   // 0x0202 — Position feedback 0-1000

// ============================================================================
// Gripper Physical Parameters
// ============================================================================

#define GRIPPER_POSITION_MIN    0
#define GRIPPER_POSITION_MAX    1000
#define GRIPPER_FORCE_MIN       20
#define GRIPPER_FORCE_MAX       100
#define GRIPPER_SPEED_MIN       1
#define GRIPPER_SPEED_MAX       100
#define GRIPPER_STROKE_MM       95.0f
#define GRIPPER_POS_TO_MM(p)    ((1000.0f - (p)) * 0.095f)

// ============================================================================
// State Enumerations (matching existing code exactly)
// ============================================================================

enum class InitState : uint16_t {
    NOT_INITIALIZED = 0,
    INITIALIZING    = 1,
    INITIALIZED     = 2
};

enum class GripperMotionState : uint16_t {
    IN_MOTION        = 0,
    POSITION_REACHED = 1,
    OBJECT_CAUGHT    = 2,
    OBJECT_DROPPED   = 3
};

inline const char* initStateStr(uint16_t s) {
    static const char* names[] = {"NOT INIT", "INITIALIZING", "READY"};
    return (s < 3) ? names[s] : "UNKNOWN";
}

inline const char* gripperStateStr(uint16_t s) {
    static const char* names[] = {"MOVING", "POS REACHED", "OBJ CAUGHT", "OBJ DROPPED"};
    return (s < 4) ? names[s] : "UNKNOWN";
}

// ============================================================================
// FreeRTOS Task Configuration
// ============================================================================

#define TASK_MODBUS_STACK_SIZE  4096
#define TASK_MODBUS_PRIORITY    3     // Highest — timing-critical
#define TASK_MODBUS_CORE        0     // Protocol processing on core 0
#define TASK_MODBUS_POLL_MS     100   // Read status every 100ms

#define TASK_GUI_STACK_SIZE     4096
#define TASK_GUI_PRIORITY       2
#define TASK_GUI_CORE           1     // GUI rendering on core 1 (Arduino default)
#define TASK_GUI_TICK_MS        5     // LVGL timer handler interval

// ============================================================================
// WiFi / MQTT Configuration
// ============================================================================

#define WIFI_SSID               "YOUR_WIFI_SSID"
#define WIFI_PASS               "YOUR_WIFI_PASSWORD"
#define MQTT_BROKER             "192.168.1.100"   // Host machine IP
#define MQTT_PORT               1883
#define MQTT_TOPIC              "qnc/modbus/device_state"

// ============================================================================
// MQTT Task Configuration
// ============================================================================

#define TASK_MQTT_STACK_SIZE    4096
#define TASK_MQTT_PRIORITY      1     // Lowest — non-critical telemetry
#define TASK_MQTT_CORE          0     // Same core as WiFi stack
#define TASK_MQTT_PUBLISH_MS    500   // Publish interval

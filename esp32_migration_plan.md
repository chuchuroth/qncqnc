# ModbusRTU Gripper Controller — ESP32-2432S028R Migration Plan

## 1. Project Overview

**Goal:** Port the Linux/Docker-based ModbusRTU gripper controller + NeuraSync dashboard into a standalone ESP32-2432S028R firmware that:

- Controls the gripper via RS485 Modbus RTU
- Displays a real-time dashboard on the built-in 2.8" ILI9341 TFT (320×240)
- Optionally publishes state over WiFi (MQTT replaces NeuraSync/DDS)
- Runs entirely as bare-metal firmware (no Docker, no Linux)

---

## 2. Hardware Profile — ESP32-2432S028R

| Resource | Spec |
|---|---|
| MCU | ESP32-WROOM-32, dual-core 240 MHz |
| SRAM | 520 KB |
| Flash | 4 MB |
| Display | 2.8" ILI9341 TFT, 320×240, SPI bus |
| Touch | Resistive (XPT2046), SPI bus |
| Connectivity | WiFi 802.11 b/g/n, Bluetooth 4.2 |
| Onboard peripherals | RGB LED, speaker driver, photosensor, TF card slot |
| Power | 5V via Micro-USB or 4P 1.25 connector |
| Available GPIOs | **Very limited** — see pin audit below |



## 3. Physical Wiring

### 3.1 RS485 Module Selection

Per your RS485 analysis document, use a **3.3V-native** transceiver:



### 3.2 Wiring Diagram

```
ESP32 Board (top-right JST)       Auto-Direction RS485 Module
┌──────────────────────┐          ┌──────────────────────┐
│  3.3V                ├──────────┤ VCC                  │
│  IO27                ├──────────┤ RXD  ← data from bus │
│  IO22                ├──────────┤ TXD  → data to bus   │
│  GND                 ├──────────┤ GND                  │
└──────────────────────┘          │            A+ ├───── Gripper A
                                  │            B- ├───── Gripper B
                                  │           GND ├───── Gripper GND
                                  └──────────────────────┘
```

### 3.3 Bus Termination & Biasing

- Place a **120Ω termination resistor** across A-B at each physical end of the bus (ESP32 side and gripper side). If the XY-017 has a built-in 120Ω (check the jumper), use that for one end.
- Ensure **common ground** between ESP32 power supply and gripper power supply.
- If using XY-017: fail-safe biasing resistors (4.7kΩ) are built in.
- If using MAX3485: add external bias — 4.7kΩ from A to 3.3V, 4.7kΩ from B to GND.

### 3.4 Power Considerations

- The ESP32-2432S028R draws ~115mA. The RS485 module adds ~5-10mA.
- Power via the Micro-USB port (5V) is sufficient for the board + RS485 module.
- The gripper should have its **own power supply** — do NOT power it from the ESP32's 3.3V rail.

---

## 4. Software Architecture

### 4.1 What Changes from Linux → ESP32

| Component | Linux/Docker | ESP32 Equivalent |
|---|---|---|
| OS | Ubuntu + Docker | FreeRTOS (built into ESP-IDF) |
| Modbus RTU | `modbus_rtu.h` (POSIX serial) | Rewrite using ESP32 UART API / HardwareSerial |
| DDS / NeuraSync | FastDDS 3.x | **Drop entirely.** Replace with MQTT over WiFi (or remove if standalone) |
| Dashboard | `gripper_dashboard.cpp` (CLI subscriber) | LVGL GUI on the built-in ILI9341 display |
| Build system | CMake + Docker | ESP-IDF (CMake) or PlatformIO (Arduino) |
| Deployment | `docker build` / `docker save` | `idf.py flash` or PlatformIO upload via USB |

### 4.2 Recommended Framework: PlatformIO + Arduino Core

For this board, **PlatformIO with Arduino framework** is the fastest path because:

- The ESP32-2432S028R community ("Cheap Yellow Display" / CYD) has extensive Arduino examples
- TFT_eSPI and LVGL already have tested configs for this exact board
- HardwareSerial is simpler than raw ESP-IDF UART for Modbus
- You can still use ESP-IDF APIs (like `uart_set_mode()` for hardware RS485) from Arduino

### 4.3 Codebase Structure

```
esp32-gripper-controller/
├── platformio.ini                  # PlatformIO project config
├── README.md
│
├── src/
│   ├── main.cpp                    # Entry point, task orchestration
│   ├── config.h                    # Pin definitions, Modbus params, WiFi creds
│   │
│   ├── modbus/
│   │   ├── modbus_rtu_esp32.h      # Modbus RTU driver (ESP32 UART-native)
│   │   ├── modbus_rtu_esp32.cpp
│   │   ├── gripper_controller.h    # High-level gripper commands (init, set_pos, read_state)
│   │   └── gripper_controller.cpp
│   │
│   ├── gui/
│   │   ├── dashboard.h             # LVGL dashboard screen definition
│   │   ├── dashboard.cpp           # Widgets: position bar, force, state indicators
│   │   ├── screens.h               # Screen manager (dashboard, settings, etc.)
│   │   └── theme.h                 # Color palette, fonts
│   │
│   ├── network/                    # (Optional — only if WiFi publishing is needed)
│   │   ├── mqtt_publisher.h        # MQTT client wrapper
│   │   └── mqtt_publisher.cpp      # Publishes DeviceState JSON to MQTT broker
│   │
│   └── utils/
│       ├── task_manager.h          # FreeRTOS task creation helpers
│       └── ring_buffer.h           # Thread-safe buffer for Modbus ↔ GUI data
│
├── lib/
│   ├── TFT_eSPI/                   # Display driver (auto-configured via platformio.ini)
│   └── lvgl/                       # GUI framework
│
├── data/                           # SPIFFS/LittleFS assets (icons, fonts if needed)
│
├── include/
│   └── lv_conf.h                   # LVGL configuration (resolution, color depth, memory)
│
└── boards/
    └── esp32-2432s028r.json        # (Optional) Custom board definition
```

### 4.4 Key Config Files

**`platformio.ini`**
```ini
[env:esp32-2432s028r]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
upload_speed = 921600

lib_deps =
    lvgl/lvgl@^9.1
    bodmer/TFT_eSPI@^2.5
    knolleary/PubSubClient@^2.8    ; MQTT (optional)

build_flags =
    ; --- Display: ILI9341 on this board's SPI pinout ---
    -D USER_SETUP_LOADED=1
    -D ILI9341_DRIVER=1
    -D TFT_WIDTH=240
    -D TFT_HEIGHT=320
    -D TFT_MOSI=13
    -D TFT_MISO=12
    -D TFT_SCLK=14
    -D TFT_CS=15
    -D TFT_DC=2
    -D TFT_RST=-1
    -D TFT_BL=21
    -D SPI_FREQUENCY=40000000
    -D SPI_READ_FREQUENCY=20000000
    -D SPI_TOUCH_FREQUENCY=2500000
    ; --- Touch: XPT2046 ---
    -D TOUCH_CS=25
    ; --- LVGL ---
    -D LV_CONF_INCLUDE_SIMPLE=1
    -D LV_TICK_PERIOD_MS=5
```

**`src/config.h`**
```cpp
#pragma once

// ---- RS485 / Modbus Pins ----
#define RS485_TX_PIN       22
#define RS485_RX_PIN       27
#define RS485_DE_RE_PIN    32   // Direction control (HIGH=TX, LOW=RX)
#define RS485_UART_NUM     2    // UART2

// ---- Modbus Settings ----
#define MODBUS_BAUDRATE     115200
#define MODBUS_SLAVE_ID     1
#define MODBUS_DATA_BITS    SERIAL_8N1

// ---- Gripper Registers (from your existing code) ----
#define REG_INIT_STATE      512   // Input register: init state
#define REG_GRIPPER_STATE   513   // Input register: gripper state
#define REG_POSITION        514   // Input register: current position
#define REG_TARGET_POS      259   // Holding register: target position
#define REG_FORCE           260   // Holding register: force
#define REG_SPEED           261   // Holding register: speed

// ---- WiFi / MQTT (optional) ----
#define WIFI_SSID           "your_ssid"
#define WIFI_PASS           "your_password"
#define MQTT_BROKER         "192.168.1.100"
#define MQTT_PORT           1883
#define MQTT_TOPIC          "qnc/modbus/device_state"

// ---- Task Priorities ----
#define TASK_MODBUS_PRIORITY    3  // Highest — timing-critical
#define TASK_GUI_PRIORITY       2
#define TASK_MQTT_PRIORITY      1  // Lowest
```

---

## 5. Core Implementation Approach

### 5.1 Modbus RTU Driver (ESP32-native)

Your existing `modbus_rtu.h` uses POSIX `termios` and `read()`/`write()` — these don't exist on ESP32. You need a rewrite using ESP32's UART API with hardware RS485 support:

```cpp
// modbus_rtu_esp32.h — key excerpt
#include <HardwareSerial.h>
#include <driver/uart.h>

class ModbusRTU_ESP32 {
public:
    ModbusRTU_ESP32(uint8_t uart_num, int tx_pin, int rx_pin, int de_pin, 
                     uint32_t baud, uint8_t slave_id);

    bool begin();
    bool read_input_registers(uint16_t start_addr, uint16_t count, uint16_t* dest);
    bool write_register(uint16_t addr, uint16_t value);

private:
    HardwareSerial* _serial;
    uint8_t _uart_num, _slave_id;
    int _de_pin;

    void send_frame(uint8_t* frame, size_t len);
    int  recv_frame(uint8_t* buf, size_t max_len, uint32_t timeout_ms);
    uint16_t compute_crc(uint8_t* data, size_t len);
};
```

**Critical:** Use the ESP32 hardware RS485 half-duplex mode as described in your recommendations document:

```cpp
bool ModbusRTU_ESP32::begin() {
    _serial = &Serial2;
    _serial->begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    
    // Use ESP32 hardware RS485 — handles DE/RE timing automatically
    _serial->setPins(RS485_RX_PIN, RS485_TX_PIN, -1, RS485_DE_RE_PIN);
    uart_set_mode((uart_port_t)_uart_num, UART_MODE_RS485_HALF_DUPLEX);
    
    return true;
}
```

### 5.2 Dashboard GUI (LVGL on ILI9341)

The 320×240 screen is small but perfectly adequate for a gripper status dashboard. Use LVGL (Light and Versatile Graphics Library) for the UI.

**Dashboard layout concept (320×240 landscape):**

```
┌──────────────────────────────────────────────┐
│  GRIPPER DASHBOARD          ● ONLINE   │ 
│─────────────────────────────────────────────│
│                                              │
│  State: [READY]        Init: [INITIALIZED]   │
│                                              │
│  Position  ████████████░░░░  725/1000        │
│  Force     ██████░░░░░░░░░░   45%            │
│  Speed     ████████████████  100%            │
│                                              │
│─────────────────────────────────────────────│
│  [OPEN]  [CLOSE]  [SET POS]  [INIT]         │
└──────────────────────────────────────────────┘
```

With resistive touch, the bottom buttons are tappable to send commands.

### 5.3 FreeRTOS Task Architecture

Three tasks running on two cores:

```
Core 0:                         Core 1 (default Arduino core):
┌─────────────────────┐        ┌───────────────────────┐
│  Task: modbusTask    │        │  Task: guiTask         │
│  Priority: 3         │        │  Priority: 2           │
│  Period: 100ms poll  │        │  Period: 5ms (lv_tick) │
│                     │        │                       │
│  - Send Modbus req  │        │  - lv_timer_handler() │
│  - Parse response   │        │  - Update widgets     │
│  - Write to shared  │        │  - Read from shared   │
│    GripperState     │        │    GripperState       │
│    (mutex-guarded)  │        │  - Handle touch input │
└─────────────────────┘        └───────────────────────┘

         Optional:
┌─────────────────────┐
│  Task: mqttTask      │
│  Priority: 1         │
│  Period: 500ms       │
│  - Publish JSON to   │
│    MQTT broker       │
└─────────────────────┘
```

**Shared data structure (thread-safe):**

```cpp
struct GripperState {
    uint16_t init_state;
    uint16_t gripper_state;
    uint16_t position;
    uint16_t force;
    uint16_t speed;
    bool     online;
    uint32_t last_update_ms;
};

// Protected by a FreeRTOS mutex
extern SemaphoreHandle_t stateMutex;
extern GripperState sharedState;
```

---

## 6. Migration Checklist

### Phase 1: Hardware Validation (Week 1)
- [ ] Confirm free GPIOs on your physical ESP32-2432S028R board (check Extended IO header)
- [ ] Purchase RS485 module (MAX3485 or XY-017)
- [ ] Wire RS485 module to ESP32 using GPIO 22/27/32
- [ ] Connect common ground between ESP32 and gripper
- [ ] Place 120Ω termination resistors at bus ends
- [ ] Flash a basic "Hello World" to the display to confirm TFT_eSPI config works

### Phase 2: Modbus Communication (Week 2)
- [ ] Port `modbus_rtu.h` to ESP32 UART API (`modbus_rtu_esp32.h/cpp`)
- [ ] Implement hardware RS485 half-duplex mode
- [ ] Test reading input registers (position, state) from gripper
- [ ] Test writing holding registers (target position, force, speed)
- [ ] Validate CRC calculation matches your existing library
- [ ] Add timeout and retry logic

### Phase 3: Dashboard GUI (Week 3)
- [ ] Set up LVGL with TFT_eSPI driver
- [ ] Create dashboard screen with status indicators
- [ ] Add progress bars for position/force/speed
- [ ] Add touch buttons for gripper commands (Open, Close, Set Position, Init)
- [ ] Wire GUI to read from `sharedState`
- [ ] Wire touch commands to write Modbus registers

### Phase 4: Integration & Polish (Week 4)
- [ ] Run modbusTask on Core 0, guiTask on Core 1
- [ ] Stress-test concurrent Modbus polling + GUI rendering
- [ ] Add error handling (gripper offline, CRC errors, timeouts)
- [ ] Add a settings screen (touchscreen) for baudrate/slave ID changes
- [ ] (Optional) Add WiFi + MQTT publishing to bridge back to NeuraSync ecosystem

### Phase 5: Optional Enhancements
- [ ] OTA firmware updates over WiFi
- [ ] Save settings to NVS (non-volatile storage)
- [ ] SD card data logging via the built-in TF slot
- [ ] mDNS for easy discovery on the network

---

## 7. Key Risks & Mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| **Not enough free GPIOs** | Cannot wire RS485 | Verify physical board first. Sacrifice RGB LED (GPIO 4/16/17) if needed — reassign those pins to UART2 |
| **Display SPI bus contention** | Glitches when Modbus + display both active | Put Modbus on UART (separate peripheral), display on SPI — they're independent hardware. Use FreeRTOS mutexes for SPI if touch/display/SD share the bus |
| **4MB flash too small** | Can't fit LVGL + fonts + app logic | Use LVGL's built-in fonts (no external assets), strip unused LVGL widgets in `lv_conf.h`, target <2MB firmware |
| **RS485 timing errors** | Modbus CRC failures, dropped frames | Use hardware RS485 mode (not software `digitalWrite`). Add inter-frame delays per Modbus spec (3.5 char times @ 115200 = ~0.3ms) |
| **Resistive touch inaccuracy** | Buttons hard to press | Use large touch targets (min 60×40 px). Calibrate touch at startup |
| **No DDS on ESP32** | Can't talk to existing NeuraSync dashboard | Use MQTT as bridge. Run a small MQTT↔DDS bridge on your Linux machine if you need both ecosystems |

---

## 8. MQTT ↔ NeuraSync Bridge (Optional)

If you want the ESP32 to publish to your existing NeuraSync dashboard, run a lightweight bridge on any Linux machine:

```
ESP32 ──(MQTT/WiFi)──→ MQTT Broker ──→ Bridge App ──(DDS)──→ NeuraSync Dashboard
```

The bridge subscribes to the MQTT topic and republishes as a DDS `DeviceState` message. This keeps the ESP32 firmware simple while maintaining compatibility with your existing infrastructure.

---

## 9. Bill of Materials

| Item | Qty | Approx Cost | Notes |
|---|---|---|---|
| ESP32-2432S028R | 1 | ~$10 | You already have this |
| MAX3485 breakout (3.3V) or XY-017 | 1 | $2–8 | 3.3V native is critical |
| 120Ω resistor (1/4W) | 2 | <$1 | Bus termination |
| 4.7kΩ resistors | 2 | <$1 | Fail-safe bias (only if using MAX3485) |
| Dupont jumper wires | ~6 | <$1 | For prototyping |
| JST connector cable (4P 1.25mm) | 1 | <$1 | If using the board's serial header |
| (Optional) MQTT broker device | 1 | — | Any Linux machine, or use a cloud broker |

---

## 10. Summary

Your project is very achievable on this board. The ESP32-2432S028R gives you the display for the dashboard and enough UART/GPIO resources for RS485 Modbus — you just need to be careful about pin assignments since so many GPIOs are consumed by the onboard peripherals. The biggest architectural shift is replacing NeuraSync/DDS with either a standalone display (no network needed) or MQTT over WiFi as a lightweight alternative.

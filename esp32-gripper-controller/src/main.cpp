#include <Arduino.h>
#include <TFT_eSPI.h>
#include <lvgl.h>

#include "config.h"
#include "modbus/modbus_rtu_esp32.h"
#include "modbus/gripper_controller.h"
#include "network/mqtt_publisher.h"
#include "gui/dashboard.h"
#include "gui/theme.h"

// ============================================================================
// Global objects
// ============================================================================

static TFT_eSPI tft;
static ModbusRTU_ESP32 modbus(RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN,
                               RS485_DE_RE_PIN, MODBUS_BAUDRATE, MODBUS_SLAVE_ID);
static GripperController gripper(modbus);
static MqttPublisher mqtt;
static Dashboard dashboard(gripper, mqtt);

// LVGL display buffer — single buffer, 1/10 of screen to save DRAM
static lv_color_t buf1[320 * 24];

// ============================================================================
// LVGL display flush callback
// ============================================================================

static void lvgl_flush_cb(lv_display_t* disp, const lv_area_t* area,
                          uint8_t* px_map)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t*)px_map, w * h, true);
    tft.endWrite();

    lv_display_flush_ready(disp);
}

// ============================================================================
// LVGL touch read callback
// ============================================================================

static void lvgl_touch_cb(lv_indev_t* indev, lv_indev_data_t* data)
{
    uint16_t x, y;
    if (tft.getTouch(&x, &y, 600)) { // 600 = pressure threshold
        data->point.x = x;
        data->point.y = y;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// ============================================================================
// FreeRTOS Task: Modbus polling (Core 0, Priority 3)
// ============================================================================

static void modbusTask(void* param)
{
    modbus.setResponseTimeoutMs(MODBUS_RESPONSE_TIMEOUT_MS);
    modbus.setInterFrameDelayMs(MODBUS_INTER_FRAME_DELAY_MS);

    if (!modbus.begin()) {
        Serial.println("[Modbus] FATAL: begin() failed");
        vTaskDelete(NULL);
        return;
    }

    Serial.println("[Modbus] Task started — polling gripper");

    for (;;) {
        gripper.poll();
        vTaskDelay(pdMS_TO_TICKS(TASK_MODBUS_POLL_MS));
    }
}

// ============================================================================
// FreeRTOS Task: GUI rendering (Core 1, Priority 2)
// ============================================================================

static void guiTask(void* param)
{
    Serial.println("[GUI] Task started");

    for (;;) {
        lv_timer_handler();

        // Update dashboard from shared state every ~100ms
        static uint32_t last_update = 0;
        if (millis() - last_update > 100) {
            GripperState state = gripper.getState();
            dashboard.update(state);

            NetworkStatus net = mqtt.getStatus();
            dashboard.updateNetwork(net);

            last_update = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_GUI_TICK_MS));
    }
}

// ============================================================================
// FreeRTOS Task: MQTT publishing (Core 0, Priority 1)
// ============================================================================

static void mqttTask(void* param)
{
    Serial.println("[MQTT] Task started");

    for (;;) {
        mqtt.loop();

        // Publish gripper state at configured interval
        static uint32_t lastPublish = 0;
        uint32_t now = millis();
        if (now - lastPublish >= TASK_MQTT_PUBLISH_MS) {
            GripperState state = gripper.getState();
            mqtt.publish(state);
            lastPublish = now;
        }

        vTaskDelay(pdMS_TO_TICKS(100));  // Service MQTT client every 100ms
    }
}

// ============================================================================
// Arduino setup()
// ============================================================================

void setup()
{
    Serial.begin(115200);
    Serial.println("\n=== ESP32-2432S028R Gripper Controller ===");

    // Initialize display
    tft.init();
    tft.setRotation(1); // Landscape
    tft.fillScreen(TFT_BLACK);
    pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
    digitalWrite(TFT_BACKLIGHT_PIN, HIGH);

    // Calibrate touch (values determined empirically per board)
    uint16_t calData[5] = {300, 3600, 300, 3600, 1};
    tft.setTouch(calData);

    // Initialize LVGL
    lv_init();

    // Create LVGL display
    lv_display_t* disp = lv_display_create(320, 240);
    lv_display_set_flush_cb(disp, lvgl_flush_cb);
    lv_display_set_buffers(disp, buf1, NULL, sizeof(buf1),
                           LV_DISPLAY_RENDER_MODE_PARTIAL);

    // Create LVGL touch input device
    lv_indev_t* indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, lvgl_touch_cb);

    // Initialize gripper controller (creates mutex and command queue)
    gripper.begin();

    // Build the dashboard UI
    dashboard.create(lv_screen_active());

    // Initialize WiFi + MQTT (non-blocking WiFi connect)
    mqtt.begin();

    Serial.println("[Setup] UI created, spawning tasks...");

    // Spawn FreeRTOS tasks
    xTaskCreatePinnedToCore(modbusTask, "modbus",
                            TASK_MODBUS_STACK_SIZE, NULL,
                            TASK_MODBUS_PRIORITY, NULL,
                            TASK_MODBUS_CORE);

    xTaskCreatePinnedToCore(guiTask, "gui",
                            TASK_GUI_STACK_SIZE, NULL,
                            TASK_GUI_PRIORITY, NULL,
                            TASK_GUI_CORE);

    xTaskCreatePinnedToCore(mqttTask, "mqtt",
                            TASK_MQTT_STACK_SIZE, NULL,
                            TASK_MQTT_PRIORITY, NULL,
                            TASK_MQTT_CORE);

    Serial.println("[Setup] All tasks running");
}

// ============================================================================
// Serial command parser — accepts text commands from host USB serial
// Commands: INIT, OPEN, CLOSE, POS <0-1000>, FORCE <20-100>, SPEED <1-100>, STATUS
// ============================================================================

static char cmdBuf[64];
static uint8_t cmdLen = 0;

static void processSerialCommand(const char* cmd)
{
    // Skip leading whitespace
    while (*cmd == ' ') cmd++;

    if (strncasecmp(cmd, "INIT", 4) == 0) {
        Serial.println("[CMD] Initializing gripper...");
        gripper.init(false);
    }
    else if (strncasecmp(cmd, "OPEN", 4) == 0) {
        Serial.println("[CMD] Opening gripper...");
        gripper.open();
    }
    else if (strncasecmp(cmd, "CLOSE", 5) == 0) {
        Serial.println("[CMD] Closing gripper...");
        gripper.close();
    }
    else if (strncasecmp(cmd, "POS ", 4) == 0) {
        int val = atoi(cmd + 4);
        if (val >= GRIPPER_POSITION_MIN && val <= GRIPPER_POSITION_MAX) {
            Serial.printf("[CMD] Setting position to %d\n", val);
            gripper.setPosition((uint16_t)val);
        } else {
            Serial.printf("[CMD] ERROR: position must be %d-%d\n",
                          GRIPPER_POSITION_MIN, GRIPPER_POSITION_MAX);
        }
    }
    else if (strncasecmp(cmd, "FORCE ", 6) == 0) {
        int val = atoi(cmd + 6);
        if (val >= GRIPPER_FORCE_MIN && val <= GRIPPER_FORCE_MAX) {
            Serial.printf("[CMD] Setting force to %d%%\n", val);
            gripper.setForce((uint16_t)val);
        } else {
            Serial.printf("[CMD] ERROR: force must be %d-%d\n",
                          GRIPPER_FORCE_MIN, GRIPPER_FORCE_MAX);
        }
    }
    else if (strncasecmp(cmd, "SPEED ", 6) == 0) {
        int val = atoi(cmd + 6);
        if (val >= GRIPPER_SPEED_MIN && val <= GRIPPER_SPEED_MAX) {
            Serial.printf("[CMD] Setting speed to %d%%\n", val);
            gripper.setSpeed((uint16_t)val);
        } else {
            Serial.printf("[CMD] ERROR: speed must be %d-%d\n",
                          GRIPPER_SPEED_MIN, GRIPPER_SPEED_MAX);
        }
    }
    else if (strncasecmp(cmd, "STATUS", 6) == 0) {
        GripperState s = gripper.getState();
        Serial.println("--- Gripper Status ---");
        Serial.printf("  Online:   %s\n", s.online ? "YES" : "NO");
        Serial.printf("  Init:     %s (%d)\n", initStateStr(s.init_state), s.init_state);
        Serial.printf("  State:    %s (%d)\n", gripperStateStr(s.gripper_state), s.gripper_state);
        Serial.printf("  Position: %d / %d\n", s.current_position, s.target_position);
        Serial.printf("  Force:    %d%%\n", s.force_percent);
        Serial.printf("  Speed:    %d%%\n", s.speed_percent);
        Serial.printf("  Errors:   %lu (last: %d)\n", s.error_count, s.last_error);
        Serial.printf("  Success:  %lu\n", s.success_count);
    }
    else if (strncasecmp(cmd, "HELP", 4) == 0) {
        Serial.println("Commands: INIT, OPEN, CLOSE, POS <0-1000>, FORCE <20-100>, SPEED <1-100>, STATUS");
    }
    else {
        Serial.printf("[CMD] Unknown: '%s' — type HELP for commands\n", cmd);
    }
}

// Arduino loop() — serial command interface
void loop()
{
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                processSerialCommand(cmdBuf);
                cmdLen = 0;
            }
        } else if (cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
}

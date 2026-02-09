#include <Arduino.h>
#include <TFT_eSPI.h>
#include <lvgl.h>

#include "config.h"
#include "modbus/modbus_rtu_esp32.h"
#include "modbus/gripper_controller.h"
#include "gui/dashboard.h"
#include "gui/theme.h"

// ============================================================================
// Global objects
// ============================================================================

static TFT_eSPI tft;
static ModbusRTU_ESP32 modbus(RS485_UART_NUM, RS485_TX_PIN, RS485_RX_PIN,
                               RS485_DE_RE_PIN, MODBUS_BAUDRATE, MODBUS_SLAVE_ID);
static GripperController gripper(modbus);
static Dashboard dashboard(gripper);

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
            last_update = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(TASK_GUI_TICK_MS));
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

    Serial.println("[Setup] All tasks running");
}

// Arduino loop() — unused; all work done in FreeRTOS tasks
void loop()
{
    vTaskDelay(portMAX_DELAY);
}

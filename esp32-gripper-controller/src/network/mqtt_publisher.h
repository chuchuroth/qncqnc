#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <freertos/semphr.h>
#include "config.h"
#include "modbus/gripper_controller.h"

// Network connection status (readable by dashboard via snapshot)
struct NetworkStatus {
    bool     wifi_connected;
    bool     mqtt_connected;
    int8_t   wifi_rssi;           // Signal strength in dBm
    uint32_t mqtt_publish_count;  // Cumulative successful publishes
    uint32_t mqtt_fail_count;     // Cumulative failed publishes
    char     ip_address[16];      // "xxx.xxx.xxx.xxx"
};

class MqttPublisher {
public:
    MqttPublisher();

    // Initialize WiFi and MQTT client (call from setup(), before task creation)
    void begin();

    // Main loop body — called from mqttTask every cycle
    void loop();

    // Publish a GripperState snapshot as JSON
    bool publish(const GripperState& state);

    // Thread-safe status access (called from GUI task)
    NetworkStatus getStatus();

private:
    WiFiClient        _wifiClient;
    PubSubClient      _mqttClient;
    NetworkStatus     _status;
    SemaphoreHandle_t _statusMutex;

    // Reconnection backoff state
    uint32_t _lastReconnectAttempt;
    uint32_t _reconnectInterval;

    static const uint32_t RECONNECT_MIN_MS = 1000;
    static const uint32_t RECONNECT_MAX_MS = 30000;

    void connectMqtt();
    void updateStatus();
};

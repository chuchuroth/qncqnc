#include "mqtt_publisher.h"

MqttPublisher::MqttPublisher()
    : _mqttClient(_wifiClient)
    , _statusMutex(nullptr)
    , _lastReconnectAttempt(0)
    , _reconnectInterval(RECONNECT_MIN_MS)
{
    memset(&_status, 0, sizeof(_status));
    strcpy(_status.ip_address, "---");
}

void MqttPublisher::begin()
{
    _statusMutex = xSemaphoreCreateMutex();

    _mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
    _mqttClient.setBufferSize(512);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.printf("[MQTT] WiFi connecting to %s...\n", WIFI_SSID);
}

void MqttPublisher::loop()
{
    // Step 1: Check WiFi — ESP32 WiFi driver auto-reconnects after WiFi.begin()
    if (WiFi.status() != WL_CONNECTED) {
        updateStatus();
        return;
    }

    // Step 2: Reconnect MQTT with exponential backoff
    if (!_mqttClient.connected()) {
        uint32_t now = millis();
        if (now - _lastReconnectAttempt >= _reconnectInterval) {
            _lastReconnectAttempt = now;
            connectMqtt();
        }
    }

    // Step 3: PubSubClient housekeeping (keepalive pings)
    _mqttClient.loop();

    // Step 4: Update shared status for dashboard
    updateStatus();
}

void MqttPublisher::connectMqtt()
{
    Serial.printf("[MQTT] Connecting to %s:%d...\n", MQTT_BROKER, MQTT_PORT);

    // Unique client ID from chip MAC
    char clientId[32];
    snprintf(clientId, sizeof(clientId), "esp32-gripper-%04X",
             (uint16_t)(ESP.getEfuseMac() & 0xFFFF));

    if (_mqttClient.connect(clientId)) {
        Serial.println("[MQTT] Connected");
        _reconnectInterval = RECONNECT_MIN_MS;
    } else {
        Serial.printf("[MQTT] Failed, rc=%d, retry in %lums\n",
                      _mqttClient.state(), _reconnectInterval);
        _reconnectInterval = min(_reconnectInterval * 2, RECONNECT_MAX_MS);
    }
}

bool MqttPublisher::publish(const GripperState& state)
{
    if (!_mqttClient.connected()) return false;

    StaticJsonDocument<256> doc;
    doc["device_id"]        = MODBUS_SLAVE_ID;
    doc["init_state"]       = state.init_state;
    doc["gripper_state"]    = state.gripper_state;
    doc["current_position"] = state.current_position;
    doc["target_position"]  = state.target_position;
    doc["force_percent"]    = state.force_percent;
    doc["speed_percent"]    = state.speed_percent;
    doc["online"]           = state.online;
    doc["error_count"]      = state.error_count;
    doc["timestamp_ms"]     = millis();

    char buffer[256];
    size_t len = serializeJson(doc, buffer, sizeof(buffer));

    bool ok = _mqttClient.publish(MQTT_TOPIC, buffer, len);

    xSemaphoreTake(_statusMutex, portMAX_DELAY);
    if (ok) {
        _status.mqtt_publish_count++;
    } else {
        _status.mqtt_fail_count++;
    }
    xSemaphoreGive(_statusMutex);

    return ok;
}

NetworkStatus MqttPublisher::getStatus()
{
    NetworkStatus snapshot;
    xSemaphoreTake(_statusMutex, portMAX_DELAY);
    snapshot = _status;
    xSemaphoreGive(_statusMutex);
    return snapshot;
}

void MqttPublisher::updateStatus()
{
    xSemaphoreTake(_statusMutex, portMAX_DELAY);
    _status.wifi_connected = (WiFi.status() == WL_CONNECTED);
    _status.mqtt_connected = _mqttClient.connected();
    _status.wifi_rssi = WiFi.RSSI();
    if (_status.wifi_connected) {
        WiFi.localIP().toString().toCharArray(_status.ip_address,
                                              sizeof(_status.ip_address));
    } else {
        strcpy(_status.ip_address, "---");
    }
    xSemaphoreGive(_statusMutex);
}

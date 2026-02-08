/**
 * ESP32 RS485 Bus Sniffer + Bridge Diagnostic
 * Tests both RX and TX paths of the TTL-to-RS485 module
 */

#include <Arduino.h>

#define RS485_TX_PIN    17
#define RS485_RX_PIN    16
#define RS485_DE_PIN    4

#define USB_BAUD        115200
#define BUF_SIZE 256

uint8_t rxBuf[BUF_SIZE];

uint16_t calcCRC16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

void setup() {
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // Receive mode

    Serial.begin(USB_BAUD);
    Serial2.begin(115200, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
    delay(100);
    while (Serial2.available()) Serial2.read();

    Serial.println("READY - Bus sniffer mode");
    Serial.println("Commands: L=listen, T=transmit test, S=send+receive test");
    Serial.println("Listening on RS485 bus...");
}

void loop() {
    // Always report any data seen on RS485 bus
    if (Serial2.available()) {
        unsigned long lastRx = millis();
        size_t len = 0;
        while (len < BUF_SIZE) {
            if (Serial2.available()) {
                rxBuf[len++] = Serial2.read();
                lastRx = millis();
            } else if (millis() - lastRx > 5) {
                break;
            }
        }
        Serial.printf("BUS RX [%d bytes]:", len);
        for (size_t i = 0; i < len && i < 40; i++)
            Serial.printf(" %02X", rxBuf[i]);
        Serial.println();
    }

    // Handle commands from USB
    if (Serial.available()) {
        char cmd = Serial.read();

        if (cmd == 'T' || cmd == 't') {
            // TX test: send a known Modbus frame and see what comes back
            Serial.println("\n--- TX TEST: Sending FC04 read reg 0x0200 addr=1 ---");
            uint8_t frame[8] = {0x01, 0x04, 0x02, 0x00, 0x00, 0x01};
            uint16_t crc = calcCRC16(frame, 6);
            frame[6] = crc & 0xFF;
            frame[7] = (crc >> 8) & 0xFF;

            Serial.printf("TX:");
            for (int i = 0; i < 8; i++) Serial.printf(" %02X", frame[i]);
            Serial.println();

            while (Serial2.available()) Serial2.read();
            digitalWrite(RS485_DE_PIN, HIGH);
            delayMicroseconds(100);
            Serial2.write(frame, 8);
            Serial2.flush();
            delayMicroseconds(200);
            digitalWrite(RS485_DE_PIN, LOW);

            // Read everything that comes back (including noise)
            unsigned long start = millis();
            while (!Serial2.available() && millis() - start < 1000);
            size_t len = 0;
            if (Serial2.available()) {
                unsigned long lastRx = millis();
                while (len < BUF_SIZE) {
                    if (Serial2.available()) {
                        rxBuf[len++] = Serial2.read();
                        lastRx = millis();
                    } else if (millis() - lastRx > 5) {
                        break;
                    }
                }
                Serial.printf("RX [%d bytes]:", len);
                for (size_t i = 0; i < len && i < 40; i++)
                    Serial.printf(" %02X", rxBuf[i]);
                Serial.println();
            } else {
                Serial.println("RX: nothing");
            }
        }
    }
}

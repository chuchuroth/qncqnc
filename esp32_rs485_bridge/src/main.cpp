/**
 * ESP32 RS485 Bridge for Modbus RTU (Debug Version)
 *
 * Bridges USB Serial (Serial) to RS485 (Serial2) with proper
 * direction control using GPIO4 for DE/nRE pins.
 *
 * Wiring:
 *   ESP32 TX2 (GPIO17) -> RS485 TX
 *   ESP32 RX2 (GPIO16) -> RS485 RX
 *   ESP32 GPIO4        -> RS485 DE & nRE (active high for transmit)
 *
 * The direction pin (GPIO4) is set HIGH during transmission and
 * LOW for receiving responses.
 */

#include <Arduino.h>

// Pin definitions
#define RS485_TX_PIN    17      // Serial2 TX
#define RS485_RX_PIN    16      // Serial2 RX
#define RS485_DE_PIN    4       // Direction control (DE/nRE)

// Serial configuration
#define USB_BAUD        115200  // USB Serial baud rate
#define RS485_BAUD      115200  // RS485 baud rate (must match gripper)

// Timing constants
#define TURNAROUND_DELAY_US  500   // Delay after TX before switching to RX
#define INTER_CHAR_TIMEOUT_MS 5    // Timeout between characters (3.5 char times at 115200)

// Buffer
#define BUFFER_SIZE 256
uint8_t txBuffer[BUFFER_SIZE];
uint8_t rxBuffer[BUFFER_SIZE];

// Debug mode - set to true to see hex dumps
#define DEBUG_MODE true

void setRS485Transmit() {
    digitalWrite(RS485_DE_PIN, HIGH);
    delayMicroseconds(50);  // Small delay for transceiver to switch
}

void setRS485Receive() {
    delayMicroseconds(TURNAROUND_DELAY_US);  // Wait for last bit to transmit
    digitalWrite(RS485_DE_PIN, LOW);
}

void printHex(uint8_t* data, size_t len, const char* prefix) {
    Serial.print(prefix);
    Serial.print(" [");
    Serial.print(len);
    Serial.print(" bytes]: ");
    for (size_t i = 0; i < len; i++) {
        if (data[i] < 0x10) Serial.print("0");
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

void setup() {
    // Initialize direction control pin
    pinMode(RS485_DE_PIN, OUTPUT);
    digitalWrite(RS485_DE_PIN, LOW);  // Start in receive mode

    // Initialize USB Serial
    Serial.begin(USB_BAUD);

    // Initialize RS485 Serial (Serial2)
    Serial2.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);

    // Wait for serial ports to initialize
    delay(100);

    // Startup message (will appear in serial monitor)
    Serial.println();
    Serial.println("ESP32 RS485 Bridge v1.1 (Debug)");
    Serial.println("================================");
    Serial.print("USB Baud: "); Serial.println(USB_BAUD);
    Serial.print("RS485 Baud: "); Serial.println(RS485_BAUD);
    Serial.print("DE Pin: GPIO"); Serial.println(RS485_DE_PIN);
    Serial.print("TX Pin: GPIO"); Serial.println(RS485_TX_PIN);
    Serial.print("RX Pin: GPIO"); Serial.println(RS485_RX_PIN);
    Serial.println("================================");
    Serial.println("Ready for Modbus RTU communication");
    Serial.println("Send 'T' to test loopback (if A/B shorted)");
    Serial.println();
}

void loop() {
    // Check for data from USB (computer -> gripper)
    if (Serial.available() > 0) {
        // Peek first char for test command
        char first = Serial.peek();

        // Test command: 'T' for loopback test
        if (first == 'T' || first == 't') {
            Serial.read();  // consume the 'T'
            Serial.println("\n>>> Loopback test...");

            // Send test pattern
            uint8_t testData[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0A};
            setRS485Transmit();
            Serial2.write(testData, sizeof(testData));
            Serial2.flush();
            setRS485Receive();

            printHex(testData, sizeof(testData), "TX");

            // Wait for loopback
            delay(100);
            if (Serial2.available() > 0) {
                size_t len = 0;
                while (Serial2.available() > 0 && len < BUFFER_SIZE) {
                    rxBuffer[len++] = Serial2.read();
                }
                printHex(rxBuffer, len, "RX");
                Serial.println("Loopback OK - RS485 transceiver working");
            } else {
                Serial.println("RX: No data received");
                Serial.println("Check: A/B wiring, device power, termination");
            }
            Serial.println();
            return;
        }

        // Normal Modbus frame forwarding
        size_t len = 0;
        unsigned long lastCharTime = millis();

        while (len < BUFFER_SIZE) {
            if (Serial.available() > 0) {
                txBuffer[len++] = Serial.read();
                lastCharTime = millis();
            } else if (millis() - lastCharTime > INTER_CHAR_TIMEOUT_MS) {
                // End of frame detected
                break;
            }
        }

        if (len > 0) {
            if (DEBUG_MODE) {
                printHex(txBuffer, len, "TX->RS485");
            }

            // Switch to transmit mode
            setRS485Transmit();

            // Send data to RS485
            Serial2.write(txBuffer, len);
            Serial2.flush();  // Wait for transmission to complete

            // Switch back to receive mode
            setRS485Receive();
        }
    }

    // Check for data from RS485 (gripper -> computer)
    if (Serial2.available() > 0) {
        // Collect the response frame
        size_t len = 0;
        unsigned long lastCharTime = millis();

        while (len < BUFFER_SIZE) {
            if (Serial2.available() > 0) {
                rxBuffer[len++] = Serial2.read();
                lastCharTime = millis();
            } else if (millis() - lastCharTime > INTER_CHAR_TIMEOUT_MS) {
                // End of frame detected
                break;
            }
        }

        if (len > 0) {
            if (DEBUG_MODE) {
                printHex(rxBuffer, len, "RX<-RS485");
            }
            // Forward to USB (raw binary for Modbus tools)
            Serial.write(rxBuffer, len);
        }
    }
}

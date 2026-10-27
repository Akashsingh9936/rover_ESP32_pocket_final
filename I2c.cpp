/*
  ENCODER ESP32 — I2C Slave (address 0x10)
  =========================================
  Reads two quadrature encoders (left + right wheels).
  Responds to I2C master (main rover ESP32) with latest tick counts.

  Wiring:
    Left  encoder A (white)  → GPIO34
    Left  encoder B (green)  → GPIO35
    Right encoder A (white)  → GPIO27
    Right encoder B (green)  → GPIO19
    I2C SDA                  → GPIO21  (connect to main ESP32 GPIO21)
    I2C SCL                  → GPIO22  (connect to main ESP32 GPIO22)
    GND                      → shared with main ESP32

  Encoder: 600 PPR, quadrature (×4) = 2400 counts/rev
  I2C: slave at 0x10, responds with 12-byte EncoderPacket on request
*/

#include <Arduino.h>
#include <Wire.h>

#define I2C_SLAVE_ADDR 0x10
#define I2C_SDA 21
#define I2C_SCL 22

// --- Left Encoder (GPIO 34, 35) ---
#define LEFT_ENC_A 34   // White wire
#define LEFT_ENC_B 35   // Green wire

// --- Right Encoder (GPIO 27, 19) ---
#define RIGHT_ENC_A 27  // White wire
#define RIGHT_ENC_B 19  // Green wire

// --- Encoder State ---
volatile long leftCount = 0;
volatile long rightCount = 0;

// --- Left Encoder ISRs ---
void IRAM_ATTR leftISR_A() {
    bool a = digitalRead(LEFT_ENC_A);
    bool b = digitalRead(LEFT_ENC_B);
    if (a == b) leftCount++;
    else leftCount--;
}

void IRAM_ATTR leftISR_B() {
    bool a = digitalRead(LEFT_ENC_A);
    bool b = digitalRead(LEFT_ENC_B);
    if (a == b) leftCount--;
    else leftCount++;
}

// --- Right Encoder ISRs (inverted direction) ---
void IRAM_ATTR rightISR_A() {
    bool a = digitalRead(RIGHT_ENC_A);
    bool b = digitalRead(RIGHT_ENC_B);
    if (a == b) rightCount--;
    else rightCount++;
}

void IRAM_ATTR rightISR_B() {
    bool a = digitalRead(RIGHT_ENC_A);
    bool b = digitalRead(RIGHT_ENC_B);
    if (a == b) rightCount++;
    else rightCount--;
}

const int PPR = 600;
const int COUNTS_PER_REV = PPR * 4;  // 2400

// I2C data packet: 2 x int32 (left ticks, right ticks) + 1 x uint32 (timestamp)
struct EncoderPacket {
    int32_t leftTicks;
    int32_t rightTicks;
    uint32_t timestampMs;
};

// Prepared packet — updated in loop(), sent in onRequest callback
static volatile EncoderPacket latestPacket = {0, 0, 0};

// Called when main ESP32 (master) requests data
void onRequest() {
    Wire.write((uint8_t*)&latestPacket, sizeof(latestPacket));
}

void setup() {
    Serial.begin(115200);

    Wire.begin((uint8_t)I2C_SLAVE_ADDR, I2C_SDA, I2C_SCL, 400000);
    Wire.onRequest(onRequest);

    pinMode(LEFT_ENC_A, INPUT);
    pinMode(LEFT_ENC_B, INPUT);
    pinMode(RIGHT_ENC_A, INPUT);
    pinMode(RIGHT_ENC_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B), leftISR_B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightISR_A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), rightISR_B, CHANGE);

    Serial.println("========================================");
    Serial.println("ENCODER ESP32 — I2C Slave (0x10)");
    Serial.println("========================================");
    Serial.println("Left: GPIO 34,35 | Right: GPIO 27,19");
    Serial.println("I2C: SDA=21, SCL=22 | Slave addr=0x10");
    Serial.printf("Encoder: %d PPR, x4 = %d counts/rev\n", PPR, COUNTS_PER_REV);
    Serial.println();
}

unsigned long lastPrint = 0;
long lastLeft = 0;
long lastRight = 0;

void loop() {
    // Keep the packet fresh for when master requests it
    noInterrupts();
    latestPacket.leftTicks = leftCount;
    latestPacket.rightTicks = rightCount;
    interrupts();
    latestPacket.timestampMs = millis();

    // Debug print at 10 Hz
    unsigned long now = millis();
    if (now - lastPrint >= 100) {
        long left = latestPacket.leftTicks;
        long right = latestPacket.rightTicks;

        float leftRPM = ((left - lastLeft) * (1000.0 / (now - lastPrint))) / COUNTS_PER_REV * 60.0;
        float rightRPM = ((right - lastRight) * (1000.0 / (now - lastPrint))) / COUNTS_PER_REV * 60.0;

        Serial.printf("L: %7ld (%6.1f RPM) | R: %7ld (%6.1f RPM)\n",
                       left, leftRPM, right, rightRPM);

        lastLeft = left;
        lastRight = right;
        lastPrint = now;
    }
}

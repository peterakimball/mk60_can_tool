/*
MIT License
Copyright (c) 2023 Peter Kimball
(See LICENSE.txt for full text)

brz_steering_test.cpp -- BRZ steering angle CAN output test

Sends CAN ID 0x18 (bench-verified Haltech steering angle input) cycling
through three positions:

  LEFT   : -300 degrees  -> raw -3000  (0.1 deg/LSB, positive = right)
  CENTER :    0 degrees  -> raw     0
  RIGHT  : +450 degrees  -> raw +4500

Byte order: big-endian (Motorola), bench verified.
The left and right angles are asymmetric so a left/right swap is unambiguous.

Build and upload:
  pio run -e brz_steering_test -t upload

Expected Haltech display:
  LEFT   : -300.0 deg
  CENTER :    0.0 deg
  RIGHT  : +450.0 deg

If signs are inverted, negate BRZ_STEERING_FACTOR in brz.h.
If magnitudes are wrong, compute: actual_factor = displayed / raw, update BRZ_STEERING_FACTOR.
If left/right are swapped, the MK60 LWS sign convention in mk60_decode.h needs investigation.
*/

#include <Arduino.h>
#include <CANSAME5x.h>

CANSAME5x CAN;
#include <mk60.h>
#include <brz.h>
#include <project.h>

// ---------------------------------------------------------------------------
// Test positions
// Raw = physical_deg / BRZ_STEERING_FACTOR = physical_deg / 0.1 = physical * 10
// Byte order: big-endian (Motorola)
// ---------------------------------------------------------------------------
struct SteeringPosition {
    const char *label;
    float       physical_deg;
    int16_t     raw;
};

static const SteeringPosition POSITIONS[] = {
    { "LEFT  ",  -300.0f, -3000 },   // -300 / 0.1 = -3000
    { "CENTER",     0.0f,     0 },
    { "RIGHT ",  +450.0f, +4500 },   // +450 / 0.1 = +4500
};
static const int POSITION_COUNT = sizeof(POSITIONS) / sizeof(POSITIONS[0]);

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
#define STEERING_TX_HZ           (50)
#define STEERING_TX_INTERVAL_MS  (1000 / STEERING_TX_HZ)
#define HOLD_DURATION_MS         (5000)

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static int           g_position_index   = 0;
static unsigned long g_position_start_ms = 0;
static unsigned long g_last_tx_ms        = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline bool serial_connected() {
    return (bool)Serial;
}

// Pack and send 0x18 with steering raw in big-endian bytes 0-1, yaw zeroed.
static void send_steering(int16_t raw) {
    uint8_t buf[8];
    memset(buf, 0, 8);
    // Big-endian (Motorola): MSB first
    buf[0] = (uint8_t)(((uint16_t)raw >> 8) & 0xFF);
    buf[1] = (uint8_t)((uint16_t)raw & 0xFF);
    CAN.beginPacket(BRZ_DYNAMICS_ID);
    CAN.write(buf, MK60_PACKET_LEN);
    CAN.endPacket();
}

static void print_position(const SteeringPosition &pos) {
    if (!serial_connected()) return;
    Serial.print("Holding: ");
    Serial.print(pos.label);
    Serial.print("  physical ");
    Serial.print(pos.physical_deg, 0);
    Serial.print(" deg  raw ");
    Serial.print(pos.raw);
    Serial.print("  bytes ");
    uint16_t u = (uint16_t)pos.raw;
    Serial.print((u >> 8) & 0xFF, HEX);
    Serial.print(" ");
    Serial.println(u & 0xFF, HEX);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(PROJECT_SERIAL_BAUD);

    unsigned long t = millis();
    while (!Serial && (millis() - t) < 1000) {}

    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, LOW);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, HIGH);

    if (!CAN.begin(MK60_BUS_SPEED)) {
        if (serial_connected()) Serial.println("CAN init failed - halting");
        while (1) {}
    }

    if (serial_connected()) {
        Serial.println("BRZ steering angle test running");
        Serial.print("CAN ID: 0x");
        Serial.println(BRZ_DYNAMICS_ID, HEX);
        Serial.println("Byte order: big-endian (Motorola)");
        Serial.println("Factor: 0.1 deg/LSB, positive = right");
        Serial.print("Rate: ");
        Serial.print(STEERING_TX_HZ);
        Serial.println(" Hz,  5 s per position");
        Serial.println("Sequence: LEFT (-300) -> CENTER (0) -> RIGHT (+450) -> repeat");
        Serial.println();
    }

    g_position_start_ms = millis();
    g_last_tx_ms        = millis();
    print_position(POSITIONS[g_position_index]);
}

void loop() {
    unsigned long now = millis();

    if (now - g_position_start_ms >= HOLD_DURATION_MS) {
        g_position_index    = (g_position_index + 1) % POSITION_COUNT;
        g_position_start_ms = now;
        print_position(POSITIONS[g_position_index]);
    }

    if (now - g_last_tx_ms >= STEERING_TX_INTERVAL_MS) {
        g_last_tx_ms = now;
        send_steering(POSITIONS[g_position_index].raw);
    }
}

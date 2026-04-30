/*
MIT License
Copyright (c) 2023-2026 Peter Kimball
(See LICENSE.txt for full text)

brz_wheel_sweep_test.cpp -- BRZ wheel speed CAN output test

Sends CAN ID 0xD4 (BRZ wheel speeds) with each wheel's 16-bit raw value
swept independently across the full unsigned 16-bit range (0x0000 to 0xFFFF),
cycling through FL -> FR -> RL -> RR. One full sweep takes approximately 5
seconds per wheel (20 seconds total per cycle), at 50 Hz transmit rate.

The three stationary wheels are held at 0x0000 during each sweep so the
Haltech channel under test is unambiguous.

Use this to verify that the Haltech Elite 1500 is receiving and decoding
the BRZ wheel speed message correctly before enabling the full translator.

Build and upload:
  pio run -e brz_wheel_sweep_test -t upload

Expected Haltech behaviour:
  Each wheel speed channel should ramp from 0 to ~3767 km/h over 5 seconds
  (the full 16-bit range at 1/28 mph/LSB = 65535 / 28 * 1.60934 ≈ 3767 km/h),
  then jump back to 0 when the next wheel begins its sweep. If the Haltech
  shows a static or zero value on all channels, the message ID or encoding
  is not being recognised.
*/

#include <Arduino.h>
#include <CANSAME5x.h>

CANSAME5x CAN;
#include <mk60.h>      // for MK60_BUS_SPEED, MK60_PACKET_LEN
#include <brz.h>       // for BRZ_WHEEL_SPEEDS_ID
#include <project.h>   // for PROJECT_SERIAL_BAUD

// ---------------------------------------------------------------------------
// Sweep parameters
// ---------------------------------------------------------------------------

// Transmit rate (Hz). Must match what the Haltech expects.
#define SWEEP_TX_HZ          (50)
#define SWEEP_TX_INTERVAL_MS (1000 / SWEEP_TX_HZ)   // 20 ms

// Number of steps across the 16-bit range per sweep.
// 65536 steps / 256 = 256 steps per wheel.
// At 50 Hz, 256 steps takes 256 * 20 ms = 5120 ms (~5 seconds).
#define SWEEP_STEPS          (256)
#define SWEEP_STEP_SIZE      (65536 / SWEEP_STEPS)   // 256 per step

// Number of wheels
#define WHEEL_COUNT          (4)

// Wheel index names for serial output
static const char * const WHEEL_NAMES[WHEEL_COUNT] = { "FL", "FR", "RL", "RR" };

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static uint8_t  g_active_wheel = 0;       // which wheel is currently sweeping
static uint16_t g_step         = 0;       // current step within the sweep
static unsigned long g_last_tx_ms = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline bool serial_connected() {
    return (bool)Serial;
}

static void send_wheel_speeds(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr) {
    uint8_t buf[8];
    buf[0] = fl & 0xFF;  buf[1] = (fl >> 8) & 0xFF;
    buf[2] = fr & 0xFF;  buf[3] = (fr >> 8) & 0xFF;
    buf[4] = rl & 0xFF;  buf[5] = (rl >> 8) & 0xFF;
    buf[6] = rr & 0xFF;  buf[7] = (rr >> 8) & 0xFF;
    CAN.beginPacket(BRZ_WHEEL_SPEEDS_ID);
    CAN.write(buf, MK60_PACKET_LEN);
    CAN.endPacket();
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(PROJECT_SERIAL_BAUD);

    pinMode(PIN_CAN_STANDBY, OUTPUT);
    digitalWrite(PIN_CAN_STANDBY, LOW);
    pinMode(PIN_CAN_BOOSTEN, OUTPUT);
    digitalWrite(PIN_CAN_BOOSTEN, HIGH);

    if (!CAN.begin(MK60_BUS_SPEED)) {
        if (serial_connected()) Serial.println("CAN init failed - halting");
        while (1) {}
    }

    if (serial_connected()) {
        Serial.println("BRZ wheel speed sweep test running");
        Serial.print("CAN ID: 0x");
        Serial.println(BRZ_WHEEL_SPEEDS_ID, HEX);
        Serial.print("Rate: ");
        Serial.print(SWEEP_TX_HZ);
        Serial.println(" Hz");
        Serial.print("Steps: ");
        Serial.print(SWEEP_STEPS);
        Serial.print(" x ");
        Serial.print(SWEEP_STEP_SIZE);
        Serial.print(" = ~");
        Serial.print((float)SWEEP_STEPS * SWEEP_TX_INTERVAL_MS / 1000.0f, 1);
        Serial.println(" s per wheel");
        Serial.println("Sweeping: FL -> FR -> RL -> RR -> repeat");
        Serial.println();
    }

    g_last_tx_ms = millis();
}

void loop() {
    unsigned long now = millis();
    if (now - g_last_tx_ms < SWEEP_TX_INTERVAL_MS) return;
    g_last_tx_ms = now;

    // Compute the raw value for this step
    uint16_t sweep_value = (uint16_t)((uint32_t)g_step * SWEEP_STEP_SIZE);

    // Build per-wheel values: only the active wheel gets the sweep value
    uint16_t fl = (g_active_wheel == 0) ? sweep_value : 0;
    uint16_t fr = (g_active_wheel == 1) ? sweep_value : 0;
    uint16_t rl = (g_active_wheel == 2) ? sweep_value : 0;
    uint16_t rr = (g_active_wheel == 3) ? sweep_value : 0;

    send_wheel_speeds(fl, fr, rl, rr);

    // Log the transition point when a new wheel starts
    if (g_step == 0 && serial_connected()) {
        Serial.print("Sweeping ");
        Serial.print(WHEEL_NAMES[g_active_wheel]);
        Serial.print("  (raw 0x0000 -> 0x");
        Serial.print((uint16_t)((uint32_t)(SWEEP_STEPS - 1) * SWEEP_STEP_SIZE), HEX);
        Serial.print("  =  0.0 -> ");
        Serial.print((float)((uint32_t)(SWEEP_STEPS - 1) * SWEEP_STEP_SIZE) * BRZ_WHEEL_SPEED_FACTOR, 1);
        Serial.println(" km/h)");
    }

    // Advance step; roll over to next wheel when sweep completes
    g_step++;
    if (g_step >= SWEEP_STEPS) {
        g_step = 0;
        g_active_wheel = (g_active_wheel + 1) % WHEEL_COUNT;
    }
}

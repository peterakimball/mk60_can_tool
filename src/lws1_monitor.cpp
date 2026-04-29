/*
MIT License
Copyright (c) 2023 Peter Kimball
(See LICENSE.txt for full text)

brz_lws1_monitor.cpp -- MK60 LWS1 (steering angle) CAN message monitor

Sends the MK60 keepalive burst in response to RTR on 0x610, then listens
for 0x1F5 (LWS1 steering angle sensor) messages and prints the raw bytes
to serial whenever they change.

Output is suppressed when data is unchanged, keeping the serial monitor
readable while physically turning the steering wheel.

Build and upload:
  pio run -e lws1_monitor -t upload
*/

#include <Arduino.h>
#include <CANSAME5x.h>

CANSAME5x CAN;
#include <mk60.h>
#include <project.h>

// ---------------------------------------------------------------------------
// Keepalive payloads (same as main translator)
// ---------------------------------------------------------------------------
// ICL1 VIN bytes: MK60 does not validate -- zeros used for clarity.
static const uint8_t icl1_data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t icl2_data[] = { 0x64, 0x0A, 0x39, 0x05, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t icl3_data[] = { 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme1_data[] = { 0x01, 0x00, 0xD9, 0x1E, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme2_data[] = { 0x11, 0x5B, 0xC9, 0x08, 0x01, 0x00, 0x00, 0x00 };

static const unsigned long KEEPALIVE_INTERPACKET_MS = 5;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static uint8_t g_last_bytes[MK60_PACKET_LEN];
static bool    g_have_data = false;
static uint32_t g_message_count = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void send_packet(uint16_t id, const uint8_t *data) {
    CAN.beginPacket(id);
    CAN.write(data, MK60_PACKET_LEN);
    CAN.endPacket();
}

static void send_keepalive_burst() {
    send_packet(MK60_ICL1_ID, icl1_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_ICL2_ID, icl2_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_ICL3_ID, icl3_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_DME1_ID, dme1_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_DME2_ID, dme2_data);
}

static void print_lws1(const uint8_t *data) {
    // Raw hex bytes
    Serial.print("0x1F5 [");
    for (int i = 0; i < MK60_PACKET_LEN; i++) {
        if (data[i] < 0x10) Serial.print('0');
        Serial.print(data[i], HEX);
        if (i < MK60_PACKET_LEN - 1) Serial.print(' ');
    }
    Serial.print("]  ");

    // Decode bytes 0-1 as a 15-bit magnitude + sign bit
    // per the documented MK60 LWS1 layout:
    //   bits  0-14 = magnitude, 0.045 deg/LSB
    //   bit  15    = sign (1 = negative)
    uint16_t raw16   = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    uint16_t mag_raw = raw16 & 0x7FFF;
    bool     negative = (raw16 & 0x8000) != 0;
    float    angle    = (float)mag_raw * 0.045f;
    if (negative) angle = -angle;

    Serial.print("angle=");
    Serial.print(angle, 2);
    Serial.print(" deg  ");

    // Decode bytes 2-3 as angle velocity, same encoding
    uint16_t raw16v   = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    uint16_t vel_raw  = raw16v & 0x7FFF;
    bool     vel_neg  = (raw16v & 0x8000) != 0;
    float    vel      = (float)vel_raw * 0.045f;
    if (vel_neg) vel = -vel;

    Serial.print("vel=");
    Serial.print(vel, 2);
    Serial.print(" deg/s  ");

    // Remaining bytes raw decimal
    Serial.print("b4=");   Serial.print(data[4]);
    Serial.print(" b5=");  Serial.print(data[5]);
    Serial.print(" b6=");  Serial.print(data[6]);
    Serial.print(" b7=");  Serial.print(data[7]);

    Serial.print("  (#");
    Serial.print(g_message_count);
    Serial.println(")");
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
        Serial.println("CAN init failed - halting");
        while (1) {}
    }

    memset(g_last_bytes, 0, MK60_PACKET_LEN);

    Serial.println("MK60 LWS1 monitor running");
    Serial.print("Listening on 0x");
    Serial.print(MK60_LWS1_ID, HEX);
    Serial.println(" -- output only when data changes");
    Serial.println("Turn the steering wheel to generate data.");
    Serial.println();
}

void loop() {
    int packetSize = CAN.parsePacket();
    if (!packetSize) return;

    long id = CAN.packetId();

    if (CAN.packetRtr()) {
        if (id == MK60_ICL1_ID) {
            send_keepalive_burst();
        }
        return;
    }

    // Read all bytes regardless of ID so the CAN buffer stays clear
    uint8_t data[MK60_PACKET_LEN];
    memset(data, 0, MK60_PACKET_LEN);
    for (int i = 0; i < MK60_PACKET_LEN && CAN.available(); i++) {
        data[i] = (uint8_t)CAN.read();
    }

    if (id != MK60_LWS1_ID) return;

    g_message_count++;

    // Only print when data has changed
    if (g_have_data && memcmp(data, g_last_bytes, MK60_PACKET_LEN) == 0) return;

    memcpy(g_last_bytes, data, MK60_PACKET_LEN);
    g_have_data = true;
    print_lws1(data);
}

/*
MIT License
Copyright (c) 2023 Peter Kimball
(See LICENSE.txt for full text)

MK60 to Haltech CAN Translator
Targets: Adafruit Feather M4 CAN Express (ATSAMD51, CANSAME5x)

Translates BMW E46 MK60 ABS CAN messages into Subaru BRZ / Toyota 86 gen1
Vehicle CAN format for the Haltech Elite 1500. Configure the Haltech as:
  Devices -> Vehicle -> Subaru BRZ (or Toyota 86)

MK60 input messages decoded:
  0x1F0 (496)  Four wheel speeds, 12-bit, 0.0625 km/h/LSB
  0x1F5 (501)  Steering angle, 15-bit + sign, 0.045 deg/LSB
  0x153 (339)  Brake light switch, byte 0 bit 4
  0x610 (1552) RTR keepalive trigger

BRZ output messages transmitted (50 Hz):
  0xD4   Wheel speeds, little-endian, 1/28 mph/LSB       -- always
  0x18   Steering angle, big-endian, 0.1 deg/LSB          -- when 0x1F5 seen
  0x152  Brake light switch, byte 6 bit 4                 -- always

Serial diagnostics (USB serial, if connected):
  Printed every 5 seconds -- message counts, rates, last wheel speeds,
  steering angle, and brake light switch state.
*/

#include <Arduino.h>
#include <CANSAME5x.h>

CANSAME5x CAN;
#include <project.h>
#include <mk60.h>
#include <mk60_decode.h>
#include <brz.h>

// ---------------------------------------------------------------------------
// Serial diagnostics
// ---------------------------------------------------------------------------
#define SERIAL_REPORT_INTERVAL_MS  (5000)

// On SAMD51, Serial is a USBSerial object -- bool() is true only when a host
// has opened the port.
static inline bool serial_connected() {
    return (bool)Serial;
}

// Per-ID receive counters, reset each reporting period
static uint32_t g_count_asc2  = 0;   // 0x1F0 wheel speeds
static uint32_t g_count_lws1  = 0;   // 0x1F5 steering angle
static uint32_t g_count_asc1  = 0;   // 0x153 brake light switch
static uint32_t g_count_rtr   = 0;   // 0x610 RTR keepalive trigger
static uint32_t g_count_other = 0;   // all other IDs

static unsigned long g_last_report_ms = 0;

// ---------------------------------------------------------------------------
// Keepalive payload data
// ---------------------------------------------------------------------------
// ICL1 carries the VIN in bytes 0-4. Bench testing confirmed the MK60 does
// not validate the VIN against its stored value -- it responds to any payload.
// Zeros are used here for clarity.
static const uint8_t icl1_data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t icl2_data[] = { 0x64, 0x0A, 0x39, 0x05, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t icl3_data[] = { 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme1_data[] = { 0x01, 0x00, 0xD9, 0x1E, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme2_data[] = { 0x11, 0x5B, 0xC9, 0x08, 0x01, 0x00, 0x00, 0x00 };

static const unsigned long KEEPALIVE_INTERPACKET_MS = 5;

// ---------------------------------------------------------------------------
// Translator state
// ---------------------------------------------------------------------------
static float g_fl_kmh = 0.0f;
static float g_fr_kmh = 0.0f;
static float g_rl_kmh = 0.0f;
static float g_rr_kmh = 0.0f;
static float g_steering_deg = 0.0f;

static bool  g_brake_light = false;

// Last transmitted wheel speeds, kept for serial reporting
static float g_sent_fl_kmh = 0.0f;
static float g_sent_fr_kmh = 0.0f;
static float g_sent_rl_kmh = 0.0f;
static float g_sent_rr_kmh = 0.0f;

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

static void read_packet_data(uint8_t *buf) {
    memset(buf, 0, MK60_PACKET_LEN);
    for (int i = 0; i < MK60_PACKET_LEN && CAN.available(); i++) {
        buf[i] = (uint8_t)CAN.read();
    }
}

// ---------------------------------------------------------------------------
// Serial diagnostics report
// ---------------------------------------------------------------------------
static void print_report(unsigned long now) {
    if (!serial_connected()) return;

    uint32_t elapsed_ms = (uint32_t)(now - g_last_report_ms);

    Serial.println("---- MK60 translator report ----");
    Serial.print("Uptime: ");
    Serial.print(now / 1000);
    Serial.println(" s");

    Serial.println("MK60 messages received:");
    Serial.print("  0x1F0 wheel speeds : "); Serial.println(g_count_asc2);
    Serial.print("  0x1F5 steering     : "); Serial.println(g_count_lws1);
    Serial.print("  0x153 brake switch : "); Serial.println(g_count_asc1);
    Serial.print("  0x610 RTR keepalive: "); Serial.println(g_count_rtr);
    Serial.print("  other              : "); Serial.println(g_count_other);

    if (elapsed_ms > 0) {
        Serial.print("  0x1F0 rate ~");
        Serial.print((g_count_asc2 * 1000UL) / elapsed_ms);
        Serial.println(" Hz");
    }

    Serial.println("Last MK60 wheel speeds (km/h):");
    Serial.print("  FL="); Serial.print(g_fl_kmh, 2);
    Serial.print("  FR="); Serial.print(g_fr_kmh, 2);
    Serial.print("  RL="); Serial.print(g_rl_kmh, 2);
    Serial.print("  RR="); Serial.println(g_rr_kmh, 2);

    Serial.println("Last Haltech wheel speeds sent (km/h):");
    Serial.print("  FL="); Serial.print(g_sent_fl_kmh, 2);
    Serial.print("  FR="); Serial.print(g_sent_fr_kmh, 2);
    Serial.print("  RL="); Serial.print(g_sent_rl_kmh, 2);
    Serial.print("  RR="); Serial.println(g_sent_rr_kmh, 2);

    Serial.print("Steering angle: ");
    if (g_count_lws1 > 0 || g_steering_deg != 0.0f) {
        Serial.print(g_steering_deg, 1);
        Serial.println(" deg");
    } else {
        Serial.println("(no data)");
    }

    Serial.print("Brake light switch: ");
    Serial.println(g_brake_light ? "ON" : "off");

    Serial.println();

    g_count_asc2  = 0;
    g_count_lws1  = 0;
    g_count_asc1  = 0;
    g_count_rtr   = 0;
    g_count_other = 0;
}

// ---------------------------------------------------------------------------
// BRZ CAN output
// ---------------------------------------------------------------------------

// Transmitted immediately on receipt of each 0x1F0 frame.
static void transmit_wheel_speeds() {
    uint8_t buf[8];
    brz_encode_wheel_speeds(buf, g_fl_kmh, g_fr_kmh, g_rl_kmh, g_rr_kmh);
    send_packet(BRZ_WHEEL_SPEEDS_ID, buf);
    g_sent_fl_kmh = g_fl_kmh;
    g_sent_fr_kmh = g_fr_kmh;
    g_sent_rl_kmh = g_rl_kmh;
    g_sent_rr_kmh = g_rr_kmh;
}

// Transmitted immediately on receipt of each 0x1F5 frame.
static void transmit_steering() {
    uint8_t buf[8];
    brz_encode_dynamics(buf, g_steering_deg, 0.0f);
    send_packet(BRZ_DYNAMICS_ID, buf);
}

// Transmitted immediately on receipt of each 0x153 frame.
static void transmit_bls() {
    uint8_t buf[8];
    brz_encode_bls(buf, g_brake_light);
    send_packet(BRZ_BLS_ID, buf);
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
        Serial.println("MK60 -> BRZ CAN translator running");
        Serial.println("0xD4 wheel speeds | 0x18 steering | 0x152 BLS");
    }

    g_last_report_ms = millis();
}

void loop() {
    int packetSize = CAN.parsePacket();

    if (packetSize) {
        long id = CAN.packetId();

        if (CAN.packetRtr()) {
            if (id == MK60_ICL1_ID) {
                g_count_rtr++;
                send_keepalive_burst();
            }
        } else {
            uint8_t data[MK60_PACKET_LEN];
            read_packet_data(data);

            if (id == MK60_ASC2_ID) {
                g_count_asc2++;
                mk60_decode_wheel_speeds(
                    data, &g_fl_kmh, &g_fr_kmh, &g_rl_kmh, &g_rr_kmh);
                transmit_wheel_speeds();

            } else if (id == MK60_LWS1_ID) {
                g_count_lws1++;
                g_steering_deg = mk60_decode_steering_angle(data);
                transmit_steering();

            } else if (id == MK60_ASC1_ID) {
                g_count_asc1++;
                g_brake_light = mk60_decode_brake_light_switch(data);
                transmit_bls();

            } else {
                g_count_other++;
            }
        }
    }

    unsigned long now = millis();

    if (now - g_last_report_ms >= SERIAL_REPORT_INTERVAL_MS) {
        print_report(now);
        g_last_report_ms = now;
    }
}

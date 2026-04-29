/*
MIT License
Copyright (c) 2023-2026 Peter Kimball
(See LICENSE.txt for full text)

vin_test.cpp -- MK60 VIN handshake experiment

Tests whether the MK60 validates the VIN in the 0x610 ICL1 response against
its internally stored value, or whether it accepts any "VIN-like" payload.

On each RTR from the MK60 on 0x610, this program responds with the normal
keepalive burst but substitutes a different VIN in the ICL1 payload, cycling
through a predefined set of candidates. The 0x1F0 wheel speed message is
monitored to determine whether the MK60 continues broadcasting after each
handshake.

VIN encoding (per ms4x.net/index.php?title=CAN_Bus_ID_0x610_ICL1):
  The last 7 digits of the VIN are encoded as BCD pairs across bytes 1-4:
    byte 0 bits 7-4: last VIN digit (units)        e.g. VIN ...X = 0xX0
    byte 0 bits 3-0: (lower nibble, fixed 0x00)
    byte 1:          3rd and 2nd digits from end    e.g. VIN ...YZ = 0xYZ  (wait, see below)
  The ms4x wiki states:
    Byte 0: last digit, drop trailing zero -> stored in upper nibble
    Byte 1: 3rd and 2nd digits from end (BCD pair)
    Byte 2: 5th and 4th digits from end (BCD pair)
    Bytes 3-4: remaining digits

  The working payload in the translator is:
    { 0x20, 0x08, 0x29, 0x54, 0x4A, 0x00, 0x00, 0x00 }
  which encodes VIN suffix JT29082:
    byte 4: 0x4A = ASCII 'J'
    byte 3: 0x54 = ASCII 'T'
    byte 2: 0x29 = BCD digits 2,9
    byte 1: 0x08 = BCD digits 0,8
    byte 0: 0x20 = last digit 2, lower nibble 0 (structural zero)

Strategy
--------
We cycle through CANDIDATE_COUNT different ICL1 payloads on successive RTRs:
  0: The known-working VIN payload (baseline -- should always work)
  1: All-zero VIN digits (000 0000) -- tests whether any VIN is accepted
  2: All-nine VIN digits (999 9999) -- different invalid value
  3: A randomly constructed plausible VIN -- tests partial match tolerance

After each response, we watch for 0x1F0 messages for RESPONSE_WINDOW_MS.
If wheel speed messages arrive, the MK60 accepted that VIN. If not, it
rejected it. Results are printed to serial.

Build and upload:
  pio run -e vin_test -t upload
*/

#include <Arduino.h>
#include <CANSAME5x.h>
#include <mk60.h>
#include <project.h>

CANSAME5x CAN;

// ---------------------------------------------------------------------------
// Candidate ICL1 payloads to test
// Bytes 0-4 carry the VIN encoding; bytes 5-7 are always 0x00.
// ---------------------------------------------------------------------------
struct VinCandidate {
    const char *label;
    uint8_t     icl1[8];
};

// VIN suffix stored in MK60: JT29082
// Encoding:
//   byte 4: 0x4A = ASCII 'J'
//   byte 3: 0x54 = ASCII 'T'
//   byte 2: 0x29 = BCD digits 2,9
//   byte 1: 0x08 = BCD digits 0,8
//   byte 0: 0x20 = last digit 2, lower nibble 0 (structural zero, not a digit)
static const VinCandidate CANDIDATES[] = {
    {
        "WORKING  -- JT29082 (known good)",
        { 0x20, 0x08, 0x29, 0x54, 0x4A, 0x00, 0x00, 0x00 }
    },
    {
        "ALL-ZERO -- 0000000 (tests whether any value is accepted)",
        { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
    },
    {
        "NUMERIC OK, ALPHA WRONG -- ZZ29082 (tests alpha validation)",
        { 0x20, 0x08, 0x29, 0x5A, 0x5A, 0x00, 0x00, 0x00 }  // 0x5A = 'Z'
    },
    {
        "ALPHA OK, NUMERIC WRONG -- JT99999 (tests numeric validation)",
        { 0x90, 0x99, 0x99, 0x54, 0x4A, 0x00, 0x00, 0x00 }
    },
};
static const int CANDIDATE_COUNT = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

// ---------------------------------------------------------------------------
// Keepalive payloads (ICL2, ICL3, DME1, DME2 are unchanged throughout)
// ---------------------------------------------------------------------------
static const uint8_t icl2_data[] = { 0x64, 0x0A, 0x39, 0x05, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t icl3_data[] = { 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme1_data[] = { 0x01, 0x00, 0xD9, 0x1E, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme2_data[] = { 0x11, 0x5B, 0xC9, 0x08, 0x01, 0x00, 0x00, 0x00 };

static const unsigned long KEEPALIVE_INTERPACKET_MS = 5;

// How long to wait for a 0x1F0 response after each handshake (ms).
// MK60 broadcasts 0x1F0 at ~100 Hz so 200 ms gives ~20 chances to respond.
static const unsigned long RESPONSE_WINDOW_MS = 200;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static int g_candidate_index = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline bool serial_connected() { return (bool)Serial; }

static void send_packet(uint16_t id, const uint8_t *data) {
    CAN.beginPacket(id);
    CAN.write(data, MK60_PACKET_LEN);
    CAN.endPacket();
}

// Send the keepalive burst with the given ICL1 payload, then wait
// RESPONSE_WINDOW_MS listening for 0x1F0. Returns true if 0x1F0 was seen.
static bool send_and_listen(const uint8_t *icl1) {
    send_packet(MK60_ICL1_ID, icl1);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_ICL2_ID, icl2_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_ICL3_ID, icl3_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_DME1_ID, dme1_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_DME2_ID, dme2_data);

    unsigned long deadline = millis() + RESPONSE_WINDOW_MS;
    while (millis() < deadline) {
        int sz = CAN.parsePacket();
        if (sz && CAN.packetId() == MK60_ASC2_ID && !CAN.packetRtr()) {
            // Drain the packet
            while (CAN.available()) CAN.read();
            return true;
        }
    }
    return false;
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
        Serial.println("MK60 VIN handshake test");
        Serial.println("Waiting for RTR on 0x610...");
        Serial.println("Each RTR advances to the next VIN candidate.");
        Serial.println();
        Serial.print("Candidates: ");
        Serial.println(CANDIDATE_COUNT);
        for (int i = 0; i < CANDIDATE_COUNT; i++) {
            Serial.print("  [");
            Serial.print(i);
            Serial.print("] ");
            Serial.println(CANDIDATES[i].label);
        }
        Serial.println();
    }
}

void loop() {
    int packetSize = CAN.parsePacket();
    if (!packetSize) return;

    long id = CAN.packetId();

    if (!CAN.packetRtr() || id != MK60_ICL1_ID) {
        // Drain non-RTR or irrelevant packets
        while (CAN.available()) CAN.read();
        return;
    }

    // RTR received -- respond with current candidate
    const VinCandidate &c = CANDIDATES[g_candidate_index];

    if (serial_connected()) {
        Serial.print("RTR received -> sending candidate [");
        Serial.print(g_candidate_index);
        Serial.print("] ");
        Serial.print(c.label);
        Serial.print(" ... ");
    }

    bool responded = send_and_listen(c.icl1);

    if (serial_connected()) {
        Serial.println(responded ? "MK60 RESPONDED (0x1F0 seen)" : "NO RESPONSE (0x1F0 absent)");
    }

    // Advance to next candidate, wrapping around
    g_candidate_index = (g_candidate_index + 1) % CANDIDATE_COUNT;

    if (serial_connected() && g_candidate_index == 0) {
        Serial.println("-- cycle complete --");
        Serial.println();
    }
}

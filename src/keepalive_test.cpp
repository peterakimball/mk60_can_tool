/*
MIT License
Copyright (c) 2023-2026 Peter Kimball
(See LICENSE.txt for full text)

keepalive_test.cpp -- Manual MK60 keepalive sequencing test

Investigates the MK60 boot timing problem: the MK60 may send its 0x610 RTR
before the Feather is ready to respond, leaving the MK60 in a silent state
until the next RTR cycle.

Two commands are available via the serial console:

  'r' -- Send the 0x610 ICL1 response burst immediately, as if an RTR had
         been received. Use this to manually trigger the handshake after the
         MK60 has already booted and missed the automatic response.

  's' -- Send the non-0x610 keepalive messages (ICL2, ICL3, DME1, DME2)
         repeatedly at their appropriate intervals for SUSTAINED_DURATION_S
         seconds. This tests whether the MK60 can be coaxed into a listening
         state by sustained keepalive traffic without needing the ICL1
         handshake first.

After either command, the program listens for 0x1F0 wheel speed messages and
reports whether the MK60 has started broadcasting.

Build and upload:
  pio run -e keepalive_test -t upload
*/

#include <Arduino.h>
#include <CANSAME5x.h>
#include <mk60.h>
#include <project.h>

CANSAME5x CAN;

// ---------------------------------------------------------------------------
// Keepalive payloads
// ---------------------------------------------------------------------------
static const uint8_t icl1_data[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t icl2_data[] = { 0x64, 0x0A, 0x39, 0x05, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t icl3_data[] = { 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme1_data[] = { 0x01, 0x00, 0xD9, 0x1E, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t dme2_data[] = { 0x11, 0x5B, 0xC9, 0x08, 0x01, 0x00, 0x00, 0x00 };

static const unsigned long KEEPALIVE_INTERPACKET_MS = 5;

// How long 's' mode sends sustained keepalives
static const unsigned long SUSTAINED_DURATION_S = 10;

// Interval between sustained keepalive bursts (ms).
// The MK60 expects these messages periodically; 100 ms is conservative.
static const unsigned long SUSTAINED_INTERVAL_MS = 100;

// How long to listen for 0x1F0 after a command before reporting result (ms)
static const unsigned long RESPONSE_WINDOW_MS = 500;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void send_packet(uint16_t id, const uint8_t *data) {
    CAN.beginPacket(id);
    CAN.write(data, MK60_PACKET_LEN);
    CAN.endPacket();
}

// Send the full ICL1 + ICL2 + ICL3 + DME1 + DME2 burst
static void send_icl1_burst() {
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

// Send only the non-ICL1 keepalive messages
static void send_sustained_burst() {
    send_packet(MK60_ICL2_ID, icl2_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_ICL3_ID, icl3_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_DME1_ID, dme1_data);
    delay(KEEPALIVE_INTERPACKET_MS);
    send_packet(MK60_DME2_ID, dme2_data);
}

// Listen for RESPONSE_WINDOW_MS and return the count of 0x1F0 frames seen.
// Also handles any inbound RTRs during the window.
static int listen_for_wheel_speeds() {
    int count = 0;
    unsigned long deadline = millis() + RESPONSE_WINDOW_MS;
    while (millis() < deadline) {
        int sz = CAN.parsePacket();
        if (!sz) continue;
        long id = CAN.packetId();
        if (CAN.packetRtr()) {
            if (id == MK60_ICL1_ID) {
                Serial.println("  (RTR received during window -- sending burst)");
                send_icl1_burst();
            }
        } else {
            while (CAN.available()) CAN.read();
            if (id == MK60_ASC2_ID) count++;
        }
    }
    return count;
}

static void report_result(int wheel_speed_count) {
    if (wheel_speed_count > 0) {
        Serial.print("MK60 RESPONDING -- ");
        Serial.print(wheel_speed_count);
        Serial.println(" x 0x1F0 received");
    } else {
        Serial.println("NO RESPONSE -- 0x1F0 not seen");
    }
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------
static void cmd_send_icl1() {
    Serial.println("Sending 0x610 ICL1 burst...");
    send_icl1_burst();
    Serial.print("Listening for 0x1F0... ");
    report_result(listen_for_wheel_speeds());
}

static void cmd_send_sustained() {
    unsigned long duration_ms = SUSTAINED_DURATION_S * 1000UL;
    Serial.print("Sending sustained keepalives for ");
    Serial.print(SUSTAINED_DURATION_S);
    Serial.println(" s (ICL2, ICL3, DME1, DME2 only -- no ICL1)...");

    unsigned long start      = millis();
    unsigned long last_burst = 0;
    int           rtr_count  = 0;
    int           asc2_count = 0;

    while (millis() - start < duration_ms) {
        unsigned long now = millis();

        if (now - last_burst >= SUSTAINED_INTERVAL_MS) {
            last_burst = now;
            send_sustained_burst();
        }

        int sz = CAN.parsePacket();
        if (!sz) continue;
        long id = CAN.packetId();
        if (CAN.packetRtr()) {
            if (id == MK60_ICL1_ID) {
                rtr_count++;
                Serial.print("  RTR at t=");
                Serial.print(now - start);
                Serial.println(" ms");
            }
        } else {
            while (CAN.available()) CAN.read();
            if (id == MK60_ASC2_ID) asc2_count++;
        }
    }

    Serial.println("Sustained burst complete.");
    Serial.print("  RTRs received: ");   Serial.println(rtr_count);
    Serial.print("  0x1F0 received: ");  Serial.println(asc2_count);
    if (asc2_count > 0) {
        Serial.println("  MK60 RESPONDED during sustained keepalives");
    } else if (rtr_count > 0) {
        Serial.println("  MK60 sent RTR but did not broadcast 0x1F0");
        Serial.println("  Try command 'r' to respond to the RTR now");
    } else {
        Serial.println("  No RTR and no 0x1F0 -- MK60 may not be powered");
    }
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
        Serial.println("CAN init failed - halting");
        while (1) {}
    }

    Serial.println("MK60 keepalive test");
    Serial.println("Commands:");
    Serial.println("  r -- send 0x610 ICL1 burst (as if responding to RTR)");
    Serial.print(  "  s -- send sustained keepalives (ICL2/ICL3/DME1/DME2) for ");
    Serial.print(SUSTAINED_DURATION_S);
    Serial.println(" s");
    Serial.println();
    Serial.println("Monitoring for inbound RTR on 0x610...");
    Serial.println("(RTRs will be handled automatically while awaiting a command)");
    Serial.println();
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        // Drain any remaining input (e.g. newline)
        while (Serial.available()) Serial.read();

        switch (cmd) {
            case 'r': cmd_send_icl1();     break;
            case 's': cmd_send_sustained(); break;
            default:
                Serial.println("Unknown command. Use 'r' or 's'.");
                break;
        }
        Serial.println();
    }

    // Passively handle inbound RTRs while waiting for a command
    int sz = CAN.parsePacket();
    if (!sz) return;

    long id = CAN.packetId();
    if (CAN.packetRtr() && id == MK60_ICL1_ID) {
        Serial.println("RTR received -- sending ICL1 burst automatically");
        send_icl1_burst();
        Serial.print("Listening for 0x1F0... ");
        report_result(listen_for_wheel_speeds());
        Serial.println();
    } else {
        while (CAN.available()) CAN.read();
    }
}

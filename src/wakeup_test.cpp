/*
MIT License
Copyright (c) 2023-2026 Peter Kimball
(See LICENSE.txt for full text)

wakeup_test.cpp -- MK60 wakeup message requirement test

Determines which keepalive messages are required to wake the MK60 after
receiving its 0x610 RTR. The MK60 is rebooted between each test.

Four response modes are available via the serial console. All modes respond
only to an inbound RTR on 0x610 and then listen for 0x1F0 to determine
whether the MK60 started broadcasting.

Commands:
  '1' -- ICL1 only          (0x610)
  '2' -- ICL1 + DME         (0x610, 0x316, 0x329)
  '3' -- ICL1 + IKE         (0x610, 0x613, 0x615)
  '4' -- All five messages  (0x610, 0x316, 0x329, 0x613, 0x615)  [known working]

Workflow:
  1. Enter the desired command in the serial monitor
  2. Reboot the MK60
  3. The program will respond to the first RTR using the selected mode
  4. Result is printed immediately

Build and upload:
  pio run -e wakeup_test -t upload
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

static const unsigned long INTERPACKET_MS   = 5;
static const unsigned long RESPONSE_WINDOW_MS = 500;

// ---------------------------------------------------------------------------
// Selected response mode (set by serial command, persists until changed)
// ---------------------------------------------------------------------------
static int g_mode = 0;   // 0 = no mode selected yet

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static inline bool serial_connected() { return (bool)Serial; }

static void send_packet(uint16_t id, const uint8_t *data) {
    CAN.beginPacket(id);
    CAN.write(data, MK60_PACKET_LEN);
    CAN.endPacket();
}

static void print_menu() {
    Serial.println("Wakeup response mode:");
    Serial.println("  1 -- ICL1 only          (0x610)");
    Serial.println("  2 -- ICL1 + DME         (0x610, 0x316, 0x329)");
    Serial.println("  3 -- ICL1 + IKE         (0x610, 0x613, 0x615)");
    Serial.println("  4 -- All five messages  (0x610, 0x316, 0x329, 0x613, 0x615)");
    Serial.println();
    Serial.println("Select a mode, then reboot the MK60.");
    Serial.println();
}

static void print_active_mode() {
    Serial.print("Active mode: ");
    switch (g_mode) {
        case 1: Serial.println("1 -- ICL1 only (0x610)");                         break;
        case 2: Serial.println("2 -- ICL1 + DME (0x610, 0x316, 0x329)");          break;
        case 3: Serial.println("3 -- ICL1 + IKE (0x610, 0x613, 0x615)");          break;
        case 4: Serial.println("4 -- All five messages");                          break;
        default: Serial.println("none -- select a mode before rebooting MK60");   break;
    }
}

// Send the response for the currently selected mode
static void send_response() {
    switch (g_mode) {
        case 1:
            send_packet(MK60_ICL1_ID, icl1_data);
            break;

        case 2:
            send_packet(MK60_ICL1_ID, icl1_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_DME1_ID, dme1_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_DME2_ID, dme2_data);
            break;

        case 3:
            send_packet(MK60_ICL1_ID, icl1_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_ICL2_ID, icl2_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_ICL3_ID, icl3_data);
            break;

        case 4:
            send_packet(MK60_ICL1_ID, icl1_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_DME1_ID, dme1_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_DME2_ID, dme2_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_ICL2_ID, icl2_data);
            delay(INTERPACKET_MS);
            send_packet(MK60_ICL3_ID, icl3_data);
            break;
    }
}

// Listen for RESPONSE_WINDOW_MS and return count of 0x1F0 frames seen
static int listen_for_wheel_speeds() {
    int count = 0;
    unsigned long deadline = millis() + RESPONSE_WINDOW_MS;
    while (millis() < deadline) {
        int sz = CAN.parsePacket();
        if (!sz) continue;
        if (!CAN.packetRtr() && CAN.packetId() == MK60_ASC2_ID) {
            count++;
        }
        while (CAN.available()) CAN.read();
    }
    return count;
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
        Serial.println("MK60 wakeup message test");
        Serial.println();
        print_menu();
    }
}

void loop() {
    // Handle serial commands
    if (Serial.available()) {
        char cmd = Serial.read();
        while (Serial.available()) Serial.read();

        if (cmd >= '1' && cmd <= '4') {
            g_mode = cmd - '0';
            print_active_mode();
            Serial.println();
        } else {
            print_menu();
        }
        return;
    }

    // Wait for RTR
    int sz = CAN.parsePacket();
    if (!sz) return;

    long id = CAN.packetId();
    while (CAN.available()) CAN.read();

    if (!CAN.packetRtr() || id != MK60_ICL1_ID) return;

    // RTR received
    if (serial_connected()) {
        Serial.print("RTR received -- ");
    }

    if (g_mode == 0) {
        if (serial_connected()) {
            Serial.println("no mode selected, ignoring.");
            Serial.println("Select a mode (1-4) and reboot the MK60.");
            Serial.println();
        }
        return;
    }

    if (serial_connected()) {
        Serial.print("responding with mode ");
        Serial.print(g_mode);
        Serial.print("... ");
    }

    send_response();

    int count = listen_for_wheel_speeds();
    if (serial_connected()) {
        if (count > 0) {
            Serial.print("MK60 RESPONDED (");
            Serial.print(count);
            Serial.println(" x 0x1F0 received)");
        } else {
            Serial.println("NO RESPONSE (0x1F0 not seen)");
        }
        Serial.println();
    }
}

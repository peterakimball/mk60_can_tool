/*
MIT License

Copyright (c) 2023 Peter Kimball

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <Arduino.h>
#include <CAN.h>
#include <mk60.h>
#include <project.h>

unsigned long interpacket_delay = 10;

// Trust me for now, these are just some values that worked, nothing evil, pinky-swear
static const uint8_t dme1_data[] = { 0x01, 0, 0xD9, 0x1E, 0, 0, 0, 0};
static const uint8_t dme2_data[] = { 0x11, 0x5B, 0xC9, 0x08, 0x01, 0, 0, 0};
static const uint8_t icl1_data[] = { 0x20, 0x08, 0x29, 0x54, 0x4a, 0, 0, 0};
static const uint8_t icl2_data[] = { 0x64, 0x0A, 0x39, 0x05, 0, 0, 0, 0};
static const uint8_t icl3_data[] = { 0, 0, 0x14, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(PROJECT_SERIAL_BAUD);

  // Board setup for CAN comms
  pinMode(PIN_CAN_STANDBY, OUTPUT);
  digitalWrite(PIN_CAN_STANDBY, false); // turn off STANDBY
  pinMode(PIN_CAN_BOOSTEN, OUTPUT);
  digitalWrite(PIN_CAN_BOOSTEN, true); // turn on booster
  
  // start the CAN bus at appropriate baud rate
  if (!CAN.begin(MK60_BUS_SPEED)) {
    while (1);
  }
}

void send_packet_with_id_and_data(u_int16_t packetId, const uint8_t *data) {
  CAN.beginPacket(packetId);
  CAN.write(data, MK60_PACKET_LEN);
  CAN.endPacket();
}

void loop() {
  int packetSize = CAN.parsePacket();

  if (packetSize) {
    if (CAN.packetRtr()) {
      if (CAN.packetId() == MK60_ICL1_ID) {
        send_packet_with_id_and_data(MK60_ICL1_ID, icl1_data);
        delay(interpacket_delay);
        send_packet_with_id_and_data(MK60_ICL2_ID, icl2_data);
        delay(interpacket_delay);
        send_packet_with_id_and_data(MK60_ICL3_ID, icl3_data);
        delay(interpacket_delay);
        send_packet_with_id_and_data(MK60_DME1_ID, dme1_data);
        delay(interpacket_delay);
        send_packet_with_id_and_data(MK60_DME2_ID, dme2_data);
        delay(interpacket_delay);
      }
    }
  }
}
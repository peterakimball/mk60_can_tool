/*
MIT License
Copyright (c) 2023 Peter Kimball
(See LICENSE.txt for full text)
*/

// ---------------------------------------------------------------------------
// CAN bus parameters
// ---------------------------------------------------------------------------
#define MK60_BUS_SPEED  (500000)
#define MK60_PACKET_LEN (8)

// ---------------------------------------------------------------------------
// MK60 input message IDs (messages WE send to keep the MK60 talking)
// ---------------------------------------------------------------------------
#define MK60_DME1_ID  (790)   // 0x316  Engine data keepalive
#define MK60_DME2_ID  (809)   // 0x329  Engine data keepalive
#define MK60_ICL1_ID  (1552)  // 0x610  VIN handshake / IKE keepalive (RTR trigger)
#define MK60_ICL2_ID  (1555)  // 0x613  IKE keepalive
#define MK60_ICL3_ID  (1557)  // 0x615  IKE keepalive

// ---------------------------------------------------------------------------
// MK60 output message IDs (messages the MK60 sends that WE receive)
// ---------------------------------------------------------------------------

// 0x1F0 - four wheel speeds, 12-bit each, Intel byte order, 0.0625 km/h/LSB
//   bits  0-11  = FL (byte0 | (byte1 & 0x0F) << 8)
//   bits 16-27  = FR (byte2 | (byte3 & 0x0F) << 8)
//   bits 32-43  = RL (byte4 | (byte5 & 0x0F) << 8)
//   bits 48-59  = RR (byte6 | (byte7 & 0x0F) << 8)
//   Upper nibble of each odd byte = unknown (see mk60_decode.h) -- must be masked
#define MK60_ASC2_ID  (496)   // 0x1F0  Wheel speeds (4 x 12-bit)

// 0x1F5 - steering angle sensor (LWS)
//   bits  0-14  = angle magnitude, 0.045 deg/LSB, sign at bit 15
//   bits 16-30  = angle velocity,  0.045 deg/s/LSB, sign at bit 31
#define MK60_LWS1_ID  (501)   // 0x1F5  Steering angle sensor

// 0x153 - vehicle speed and brake light switch (ASC/DSC status)
//   byte 0 bit 4 = brake light switch (BLS): 1 = brake pedal pressed
#define MK60_ASC1_ID  (339)   // 0x153  Vehicle speed + brake light switch

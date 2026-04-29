/*
MIT License
Copyright (c) 2023-2026 Peter Kimball
(See LICENSE.txt for full text)

mk60_decode.h -- Decode raw MK60 CAN message bytes into physical values.

All decode functions accept an 8-byte buffer matching a received CAN frame.
Physical units:
  Wheel speed    : km/h (float)
  Steering angle : degrees (float, signed, positive = right)
  Brake light    : bool
*/

#pragma once
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// 0x1F0 (MK60_ASC2_ID = 496) -- Wheel speeds
//
// Four 12-bit fields packed into 8 bytes, Intel byte order, 0.0625 km/h/LSB.
// Upper nibble of each odd byte must be masked when extracting the speed field.
//
//   FL: byte0 | ((byte1 & 0x0F) << 8)
//   FR: byte2 | ((byte3 & 0x0F) << 8)
//   RL: byte4 | ((byte5 & 0x0F) << 8)
//   RR: byte6 | ((byte7 & 0x0F) << 8)
//
// Upper nibble of each odd byte:
//   Community documentation describes this as a qualifier bitfield (signal
//   valid, standstill, ABS active, direction). However, bench testing showed
//   the FL nibble cycling 0x0->0x2->0x4->0x6 in a fixed 4-step pattern
//   independent of wheel state, including during confirmed reverse rotation
//   where the direction bit did not appear. This is consistent with a rolling
//   2-bit message counter rather than a status bitfield. The meaning of this
//   nibble is therefore treated as unknown and is not used by the translator.
// ---------------------------------------------------------------------------
static inline float mk60_decode_wheel_speed_field(const uint8_t *data, int byte_offset) {
    uint16_t raw = (uint16_t)data[byte_offset]
                 | ((uint16_t)(data[byte_offset + 1] & 0x0F) << 8);
    return (float)raw * 0.0625f;
}

// Populates four wheel speeds in km/h. Always returns true -- the upper nibble
// is not used for validity gating (see note above).
static inline void mk60_decode_wheel_speeds(
    const uint8_t *data,
    float *fl_kmh, float *fr_kmh, float *rl_kmh, float *rr_kmh)
{
    *fl_kmh = mk60_decode_wheel_speed_field(data, 0);
    *fr_kmh = mk60_decode_wheel_speed_field(data, 2);
    *rl_kmh = mk60_decode_wheel_speed_field(data, 4);
    *rr_kmh = mk60_decode_wheel_speed_field(data, 6);
}

// ---------------------------------------------------------------------------
// 0x1F5 (MK60_LWS1_ID = 501) -- Steering angle sensor
//
// 15-bit magnitude + separate sign bit, 0.045 deg/LSB.
// Returns signed angle in degrees (positive = right).
// ---------------------------------------------------------------------------
static inline float mk60_decode_steering_angle(const uint8_t *data) {
    uint16_t raw       = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    uint16_t magnitude = raw & 0x7FFF;
    bool     negative  = (raw & 0x8000) != 0;
    float    angle     = (float)magnitude * 0.045f;
    return negative ? -angle : angle;
}

// ---------------------------------------------------------------------------
// 0x153 (MK60_ASC1_ID = 339) -- Brake light switch
//
// Byte 0, bit 4: brake light switch state.
//   1 = brake pedal pressed
//   0 = brake pedal released
// ---------------------------------------------------------------------------
static inline bool mk60_decode_brake_light_switch(const uint8_t *data) {
    return (data[0] & (1 << 4)) != 0;
}

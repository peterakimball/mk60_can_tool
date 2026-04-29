/*
MIT License
Copyright (c) 2023-2026 Peter Kimball
(See LICENSE.txt for full text)

brz.h -- Encode MK60-derived values into Subaru BRZ / Toyota 86 / Scion FR-S
         gen1 (2013-2020) Vehicle CAN format for the Haltech Elite 1500.

Protocol source: timurrrr/ft86 gen1.md and timurrrr/RaceChronoDiyBleDevice
ft86.md (both MIT licence), cross-referenced against Autosport Labs forum
thread #5319. Bus speed: 500 kbit/s.

Frames produced:
  0xD4   All four wheel speeds  -- always transmitted
  0x18   Steering angle         -- transmitted when 0x1F5 available
  0x152  Brake light switch     -- always transmitted

Signal encoding (from the canonical ft86 CAN database and bench-verified):
  Wheel speed   : 0xD4, 16-bit signed Intel, 1/28 mph/LSB (0.057476 km/h/LSB) -- Haltech verified
                  FL bytes 0-1, FR bytes 2-3, RL bytes 4-5, RR bytes 6-7
  Steering angle: 0x18, bytes 0-1, 16-bit signed Motorola, 0.1 deg/LSB, +ve=right -- bench verified
  Yaw rate      : 0x18, bytes 2-3, 16-bit signed Motorola, 0.286478897 deg/s/LSB -- unverified
  BLS           : 0x152, byte 6 bit 4 -- bench verified
*/

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ---------------------------------------------------------------------------
// BRZ/86 output message IDs
// ---------------------------------------------------------------------------
#define BRZ_WHEEL_SPEEDS_ID  (0xD4)   // FL, FR, RL, RR wheel speeds
#define BRZ_DYNAMICS_ID      (0x18)   // Steering angle -- bench verified
#define BRZ_BLS_ID           (0x152)  // Brake light switch

// Transmit rate -- BRZ OEM transmits at 50-100 Hz; we use 50 Hz.
#define BRZ_TRANSMIT_INTERVAL_MS  (20)

// ---------------------------------------------------------------------------
// Scale factors
// ---------------------------------------------------------------------------

// Wheel speed: 1/28 mph/LSB = 0.057476 km/h/LSB, bench verified.
// Note: the ft86 CAN database documents 0.015694 km/h/LSB (the OEM encoding).
// The Haltech BRZ template interprets the field at 1/28 mph/LSB -- empirically
// confirmed across raw values 1000/2000/3000/4000 to within display rounding.
#define BRZ_WHEEL_SPEED_FACTOR    (1.60934f / 28.0f)   // 0.057476 km/h/LSB

// Steering: big-endian (Motorola) signed int16, 0.1 deg/LSB, positive = right.
// Bench verified: Haltech reads 0x18 bytes 0-1 as Motorola byte order.
// Note: yaw rate byte order and factor are unverified -- 0x18 layout TBD.
#define BRZ_STEERING_FACTOR       (0.1f)             // deg/LSB, Motorola
#define BRZ_YAW_FACTOR            (0.286478897f)     // deg/s/LSB, Motorola, unverified

// ---------------------------------------------------------------------------
// 0x152 BLS byte/bit position -- bench verified
// Source: timurrrr/ft86 gen1.md "Brake pedal pressed: (G & 16) / 16"
//   G = byte 6, 16 = 1 << 4
// ---------------------------------------------------------------------------
#define BRZ_BLS_BYTE  (6)
#define BRZ_BLS_BIT   (4)

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

// Little-endian (Intel) pack -- used by 0xD4 wheel speeds
static inline void brz_pack_s16_le(uint8_t *dest, int16_t value) {
    dest[0] = (uint8_t)((uint16_t)value & 0xFF);
    dest[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFF);
}

// Big-endian (Motorola) pack -- used by 0x18 steering/yaw, bench verified
static inline void brz_pack_s16_be(uint8_t *dest, int16_t value) {
    dest[0] = (uint8_t)(((uint16_t)value >> 8) & 0xFF);
    dest[1] = (uint8_t)((uint16_t)value & 0xFF);
}

static inline int16_t brz_clamp_s16(float v) {
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

// ---------------------------------------------------------------------------
// 0xD4 -- All four wheel speeds
// Input km/h from mk60_decode; encoded as signed int16, 1/28 mph/LSB (0.057476 km/h/LSB).
// ---------------------------------------------------------------------------
static inline void brz_encode_wheel_speeds(
    uint8_t *buf,
    float fl_kmh, float fr_kmh, float rl_kmh, float rr_kmh)
{
    memset(buf, 0, 8);
    brz_pack_s16_le(buf + 0, brz_clamp_s16(fl_kmh / BRZ_WHEEL_SPEED_FACTOR));
    brz_pack_s16_le(buf + 2, brz_clamp_s16(fr_kmh / BRZ_WHEEL_SPEED_FACTOR));
    brz_pack_s16_le(buf + 4, brz_clamp_s16(rl_kmh / BRZ_WHEEL_SPEED_FACTOR));
    brz_pack_s16_le(buf + 6, brz_clamp_s16(rr_kmh / BRZ_WHEEL_SPEED_FACTOR));
}

// ---------------------------------------------------------------------------
// 0x18 -- Steering angle and yaw rate
// Byte order: big-endian (Motorola), bench verified.
// steering_deg: positive = right, factor 0.1 deg/LSB.
// yaw_deg_s: not available from MK60 CAN; pass 0.0.
// ---------------------------------------------------------------------------
static inline void brz_encode_dynamics(
    uint8_t *buf,
    float steering_deg,
    float yaw_deg_s)
{
    memset(buf, 0, 8);
    brz_pack_s16_be(buf + 0, brz_clamp_s16(steering_deg / BRZ_STEERING_FACTOR));
    brz_pack_s16_be(buf + 2, brz_clamp_s16(yaw_deg_s   / BRZ_YAW_FACTOR));
}

// ---------------------------------------------------------------------------
// 0x152 -- Brake light switch
//
// Byte 6 bit 4, per ft86 gen1 CAN database and bench verification.
// All other bytes in the frame are zeroed.
// ---------------------------------------------------------------------------
static inline void brz_encode_bls(uint8_t *buf, bool brake_active) {
    memset(buf, 0, 8);
    if (brake_active) {
        buf[BRZ_BLS_BYTE] |= (1 << BRZ_BLS_BIT);
    }
}

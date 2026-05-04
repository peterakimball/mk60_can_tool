/*
MIT License
Copyright (c) 2023 Peter Kimball
(See LICENSE.txt for full text)

bosch_m5.h -- Encode physical values into Bosch Motorsport M5 ABS CAN frames.

Protocol: V19, "customer from 0700 onwards"
Source: Bosch Motorsport ABS M5 Kit Manual v1.3, Chapter 12

The Haltech Elite 1500 is configured to read the Bosch M5 ABS CAN stream.
This file builds the specific frame formats that the Haltech expects.

Frames produced:
  0x24A  All four wheel speeds                  -- always transmitted
  0x342  Accumulator fill levels + wheel quality -- always transmitted
  0x5C0  Status, brake pressures, diagnostics   -- always transmitted

Scale factors (from the official Bosch DBC / manual):
  Wheel speed : 0.015625 m/s/LSB  (= 1/64),  16-bit unsigned Intel
  Brake pressure: 0.01526 bar/LSB,            16-bit SIGNED Intel
  (Input from MK60 decode is in km/h for speed and bar for pressure)
*/

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Bosch M5 ABS CAN Protocol V19 message IDs
// Source: Bosch Motorsport ABS M5 Kit Manual v1.3, Chapter 12
// "CAN Protocol V19 customer from 0700 onwards"
// ---------------------------------------------------------------------------

// 0x24A - All four wheel speeds in a single 8-byte message
//   All fields: 16-bit Intel unsigned, factor 0.015625 m/s/LSB, offset 0
//   bits  0-15  = RG_VL_Bremse2  (FL wheel speed direct)
//   bits 16-31  = RG_VR_Bremse2  (FR wheel speed direct)
//   bits 32-47  = RG_HL_Bremse2  (RL wheel speed direct)
//   bits 48-63  = RG_HR_Bremse2  (RR wheel speed direct)
#define M5_WHEEL_SPEEDS_ID  (0x24A)

// 0x342 - Accumulator fill levels and per-wheel signal quality
//   bits  0-7   = acc_FA  (front axle reservoir, 0.05 cm3/LSB)
//   bits  8-15  = acc_RA  (rear axle reservoir, 0.05 cm3/LSB)
//   bits 32-39  = WheelQuality_FL (0=ok, >1=disturbance)
//   bits 40-47  = WheelQuality_FR
//   bits 48-55  = WheelQuality_RL
//   bits 56-63  = WheelQuality_RR
#define M5_QUALITY_ID       (0x342)

// 0x5C0 - Status, brake pressures, switch position, and diagnostic bits
//   bits  0-7   = SwitchPosition (ABS map switch, 1-12)
//   bits  8-23  = P_FA  (front brake pressure, signed int16, 0.01526 bar/LSB)
//   bit  24     = BLS   (brake light switch)
//   bit  28     = ABS_Malfunction
//   bit  29     = ABS_Active
//   bit  30     = EBD_Lamp
//   bit  31     = ABS_Lamp
//   bits 32-33  = Diag_FL (0=ok, 1=wiring fault, 2=signal fault)
//   bits 34-35  = Diag_FR
//   bits 36-37  = Diag_RL
//   bits 38-39  = Diag_RR
//   bits 40-47  = further diagnostic bits (ABSUnit, FuseValve, FusePump, P_FA, P_RA, YRS, fault_info)
//   bits 48-63  = P_RA  (rear brake pressure, signed int16, 0.01526 bar/LSB)
#define M5_STATUS_ID        (0x5C0)

// Transmit rate for M5 output messages.
// A real M5 unit transmits 0x24A at 100 Hz; we target 50 Hz as a conservative
// rate that is well within the Haltech's expected update window.
#define M5_TRANSMIT_INTERVAL_MS  (20)   // 50 Hz

// ---------------------------------------------------------------------------
// Scale factors matching the official Bosch V19 protocol
#define M5_WHEEL_SPEED_FACTOR   (0.015625f)  // m/s per LSB
#define M5_BRAKE_PRESSURE_FACTOR (0.01526f)  // bar per LSB

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
static inline void m5_pack_u16_le(uint8_t *dest, uint16_t value) {
    dest[0] = (uint8_t)(value & 0xFF);
    dest[1] = (uint8_t)((value >> 8) & 0xFF);
}

static inline void m5_pack_s16_le(uint8_t *dest, int16_t value) {
    dest[0] = (uint8_t)((uint16_t)value & 0xFF);
    dest[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFF);
}

// Convert km/h (from MK60 decode) to M5 raw wheel speed units (m/s / 0.015625)
static inline uint16_t m5_kmh_to_raw(float kmh) {
    float ms = kmh / 3.6f;
    return (uint16_t)(ms / M5_WHEEL_SPEED_FACTOR);
}

// Convert bar (from MK60 decode) to M5 raw brake pressure units
static inline int16_t m5_bar_to_raw(float bar) {
    return (int16_t)(bar / M5_BRAKE_PRESSURE_FACTOR);
}

// ---------------------------------------------------------------------------
// 0x24A -- All four wheel speeds
//
//   bits  0-15  = FL (RG_VL_Bremse2)
//   bits 16-31  = FR (RG_VR_Bremse2)
//   bits 32-47  = RL (RG_HL_Bremse2)
//   bits 48-63  = RR (RG_HR_Bremse2)
//
// All fields: 16-bit Intel unsigned, 0.015625 m/s/LSB
// Speed inputs are in km/h from mk60_decode; conversion is applied here.
// ---------------------------------------------------------------------------
static inline void m5_encode_wheel_speeds(
    uint8_t *buf,
    float fl_kmh, float fr_kmh, float rl_kmh, float rr_kmh)
{
    m5_pack_u16_le(buf + 0, m5_kmh_to_raw(fl_kmh));
    m5_pack_u16_le(buf + 2, m5_kmh_to_raw(fr_kmh));
    m5_pack_u16_le(buf + 4, m5_kmh_to_raw(rl_kmh));
    m5_pack_u16_le(buf + 6, m5_kmh_to_raw(rr_kmh));
}

// ---------------------------------------------------------------------------
// 0x342 -- Accumulator fill levels and wheel quality flags
//
//   bits  0-7   = acc_FA (front axle accumulator, 0.05 cm3/LSB)
//   bits  8-15  = acc_RA (rear axle accumulator, 0.05 cm3/LSB)
//   bits 16-31  = reserved (zero)
//   bits 32-39  = WheelQuality_FL
//   bits 40-47  = WheelQuality_FR
//   bits 48-55  = WheelQuality_RL
//   bits 56-63  = WheelQuality_RR
//
// WheelQuality values: 0 = signal ok, >1 = disturbance detected.
// We set quality to 0 (ok) when wheel speeds are valid, 1 when not.
// Accumulator fill levels are not available from the MK60; set to 0.
// ---------------------------------------------------------------------------
static inline void m5_encode_quality(
    uint8_t *buf,
    bool wheels_valid)
{
    memset(buf, 0, 8);
    uint8_t q = wheels_valid ? 0 : 1;
    buf[4] = q;  // WheelQuality_FL  (bit 32)
    buf[5] = q;  // WheelQuality_FR  (bit 40)
    buf[6] = q;  // WheelQuality_RL  (bit 48)
    buf[7] = q;  // WheelQuality_RR  (bit 56)
}

// ---------------------------------------------------------------------------
// 0x5C0 -- Status, brake pressures, switch position, diagnostics
//
//   byte 0       bits  0-7   SwitchPosition (1-12; use 6 = mid-range default)
//   bytes 1-2    bits  8-23  P_FA  (front brake pressure, signed int16)
//   byte 3       bit  24     BLS   (brake light switch)
//                bit  25     reserved
//                bits 26-27  Bremse_53_cnt (counter, leave 0)
//                bit  28     ABS_Malfunction
//                bit  29     ABS_Active
//                bit  30     EBD_Lamp
//                bit  31     ABS_Lamp
//   bytes 4-5    bits 32-33  Diag_FL (2-bit: 0=ok, 1=wiring, 2=signal)
//                bits 34-35  Diag_FR
//                bits 36-37  Diag_RL
//                bits 38-39  Diag_RR
//                bits 40-47  further diag bits (leave 0)
//   bytes 6-7    bits 48-63  P_RA  (rear brake pressure, signed int16)
// ---------------------------------------------------------------------------
static inline void m5_encode_status(
    uint8_t *buf,
    bool abs_active,
    bool abs_malfunction,
    bool ebd_lamp,
    bool abs_lamp,
    bool brake_light_switch,
    float front_bar,        // 0.0 if not available
    float rear_bar,         // 0.0 if not available
    bool pressure_valid)
{
    memset(buf, 0, 8);

    // byte 0: switch position -- report mid-range (6); not available from MK60
    buf[0] = 6;

    // bytes 1-2: P_FA (front brake pressure, signed int16, 0.01526 bar/LSB)
    if (pressure_valid) {
        m5_pack_s16_le(buf + 1, m5_bar_to_raw(front_bar));
    }

    // byte 3: status bits
    uint8_t status = 0;
    if (brake_light_switch) status |= (1 << 0);  // bit 24
    // bits 1-2 (Bremse_53_cnt): leave 0
    if (abs_malfunction)   status |= (1 << 4);  // bit 28
    if (abs_active)        status |= (1 << 5);  // bit 29
    if (ebd_lamp)          status |= (1 << 6);  // bit 30
    if (abs_lamp)          status |= (1 << 7);  // bit 31
    buf[3] = status;

    // bytes 4-5: Diag bits -- all 0 (no per-wheel diagnostics available from MK60)

    // bytes 6-7: P_RA (rear brake pressure, signed int16, 0.01526 bar/LSB)
    if (pressure_valid) {
        m5_pack_s16_le(buf + 6, m5_bar_to_raw(rear_bar));
    }
}

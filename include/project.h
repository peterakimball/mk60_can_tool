/*
MIT License
Copyright (c) 2023-2026 Peter Kimball
(See LICENSE.txt for full text)
*/

#define PROJECT_SERIAL_BAUD 115200

// ---------------------------------------------------------------------------
// Wheel speed correction
//
// The MK60 computes wheel speed from the ABS tone ring tooth count and the
// tire rolling diameter it was calibrated for. If your tone ring tooth count
// or tire diameter differs from the stock E46 values, the MK60 will report
// incorrect wheel speeds and a correction factor must be applied.
//
// Choose ONE of the two options below.
// ---------------------------------------------------------------------------

// E46 reference values -- do not change
#define WHEEL_SPEED_E46_TONE_RING_TEETH  (48)    // stock E46 tone ring
#define WHEEL_SPEED_E46_TIRE_DIAMETER_MM (634)   // stock E46 225/45R17

// Option A: specify your tone ring tooth count and tire diameter.
// The correction factor is computed automatically from the ratio to the E46
// reference values above.
//
// To find your tire diameter in mm from a tyre size code (e.g. 225/45R17):
//   diameter = (rim_inches * 25.4) + 2 * (width_mm * aspect_ratio / 100)
//   e.g. 225/45R17: (17 * 25.4) + 2 * (225 * 0.45) = 431.8 + 202.5 = 634mm
#define WHEEL_SPEED_TONE_RING_TEETH   (48)    // set to your tone ring tooth count
#define WHEEL_SPEED_TIRE_DIAMETER_MM  (634)   // set to your tire diameter in mm

#define WHEEL_SPEED_CORRECTION_FACTOR \
    ((float)WHEEL_SPEED_TONE_RING_TEETH  / WHEEL_SPEED_E46_TONE_RING_TEETH \
   * (float)WHEEL_SPEED_E46_TIRE_DIAMETER_MM / WHEEL_SPEED_TIRE_DIAMETER_MM)

// Option B: specify a correction factor directly.
// Uncomment this line and comment out Option A above to use a fixed factor.
// A value of 1.0 applies no correction.
// #define WHEEL_SPEED_CORRECTION_FACTOR (1.0f)

# MK60 CAN Tool

## Intent

This application runs on an [Adafruit Feather M4 CAN Express](https://www.adafruit.com/product/4759)
and acts as a CAN bus translator between a BMW E46 MK60 ABS module and a
[Haltech Elite 1500](https://www.haltech.com/product/ht-150990-elite-1500/) ECU.

The BMW E46 MK60 is a capable ABS/DSC unit commonly retained in amateur
motorsport builds, but it speaks a proprietary BMW CAN protocol that the
Haltech does not understand natively. This translator sits on the shared
500 kbit/s CAN bus between the two devices and performs two roles:

1. **Keepalive** — The MK60 will not broadcast any data unless it receives
   a periodic handshake sequence on CAN ID `0x610`. This translator generates
   that handshake automatically, unlocking the MK60's CAN output.

2. **Translation** — Wheel speed, steering angle, and brake light switch data
   from the MK60 are decoded and re-encoded in Subaru BRZ / Toyota 86 gen1
   Vehicle CAN format, which the Haltech Elite 1500 understands natively.
   Configure the Haltech as: `Devices -> Vehicle -> Subaru BRZ` (or Toyota 86).

### MK60 messages decoded

| CAN ID | Content | Notes |
|--------|---------|-------|
| `0x610` | RTR keepalive trigger | MK60 sends an RTR; translator responds with handshake burst |
| `0x1F0` | Four wheel speeds | 12-bit fields, 0.0625 km/h/LSB |
| `0x1F5` | Steering angle | 15-bit + sign, 0.045 deg/LSB; requires LWS calibration |
| `0x153` | Brake light switch | Byte 0, bit 4 |

### BRZ-format messages transmitted to Haltech

| CAN ID | Content | Encoding | Notes |
|--------|---------|----------|-------|
| `0xD4` | Wheel speeds | Little-endian, 1/28 mph/LSB | Transmitted on each received `0x1F0` |
| `0x18` | Steering angle | Big-endian, 0.1 deg/LSB, +ve = right | Transmitted on each received `0x1F5` |
| `0x152` | Brake light switch | Byte 6, bit 4 | Transmitted on each received `0x153` |

> **Note:** The BRZ encoding values above were determined empirically by bench
> testing against the Haltech and differ in some cases from community CAN
> database documentation for the BRZ/86 platform.

## Hardware

* [Adafruit Feather M4 CAN Express](https://www.adafruit.com/product/4759) (ATSAMD51, CANSAME5x)
* The Feather M4, MK60, and Haltech all share a single 500 kbit/s CAN bus
* Ensure exactly two 120Ω termination resistors are present, one at each physical end of the bus
* The Haltech must have **Vehicle CAN receive-only mode disabled** to acknowledge frames

### CAN bus termination

A properly terminated CAN bus requires exactly two 120Ω resistors, one at each
physical end of the bus. With the bus unpowered, measuring resistance between
CAN_H and CAN_L should read **60Ω** (two 120Ω resistors in parallel).

| Measured resistance | Diagnosis |
|---------------------|----------|
| 60Ω | Correct — two terminators present |
| 120Ω | Only one terminator present |
| 40Ω or less | More than two terminators present |
| Open circuit | No terminators connected |

> **Warning:** The Adafruit Feather M4 CAN Express has an onboard 120Ω
> termination resistor **enabled by default** via a solder jumper. Depending
> on where the Feather sits in your physical bus layout, this may be one of
> your two required terminators, or it may be an unwanted additional terminator.
> Check the jumper and your wiring carefully before powering the bus.
> See the [Feather M4 CAN Express schematic](https://learn.adafruit.com/adafruit-feather-m4-can-express/downloads)
> for the jumper location.

## Safety notice

This software is provided for use in motorsport and experimental vehicle
applications. By using this software you acknowledge the following:

- **No warranty.** This software is provided "as is", without warranty of
  any kind. See [LICENSE.txt](LICENSE.txt) for the full terms.
- **No validation.** The signal translations implemented here were determined
  by empirical bench testing against one specific vehicle and ECU. They have
  not been independently verified and may be incorrect or incomplete.
- **Brakes and safety systems.** This software interfaces with ABS and
  related vehicle safety systems. Incorrect operation could affect braking
  performance in ways that are not immediately apparent and could result in
  loss of vehicle control, serious injury, or death.
- **Your responsibility.** You are solely responsible for validating that
  this software functions correctly in your specific installation before
  operating the vehicle at speed or in any situation where loss of control
  could cause harm.

Always test thoroughly at low speed in a controlled environment before
competitive or high-speed use.

## Project setup

This project uses [PlatformIO](https://platformio.org/) with the Arduino
framework. The recommended setup is PlatformIO IDE as a Visual Studio Code
extension.

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install the PlatformIO IDE extension — see the
   [official setup guide](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
3. Clone this repository **with submodules** to pull in the Adafruit_CAN library:
   ```
   git clone --recurse-submodules <repo-url>
   ```
   If you have already cloned without `--recurse-submodules`, run:
   ```
   git submodule update --init
   ```
4. Open the cloned folder in VS Code. PlatformIO will detect `platformio.ini`
   automatically.

## Build environments

The project defines several build environments in `platformio.ini`. Build and
upload a specific environment with:

```
pio run -e <environment> -t upload
```

| Environment | Source file | Purpose |
|-------------|-------------|---------|
| `adafruit_feather_m4_can` | `main.cpp` | **Production translator firmware** |
| `brz_wheel_sweep_test` | `brz_wheel_sweep_test.cpp` | Sweeps all four wheel speed channels to verify Haltech reception |
| `brz_steering_test` | `brz_steering_test.cpp` | Steps through left/center/right steering positions |
| `lws1_monitor` | `lws1_monitor.cpp` | Prints raw MK60 `0x1F5` bytes to serial; useful for LWS diagnosis |
| `vin_test` | `vin_test.cpp` | Tests whether MK60 validates the VIN in the `0x610` handshake |

## Serial diagnostics

When a USB serial host is connected, the production firmware prints a
diagnostic report every 5 seconds showing message counts, receive rates,
last wheel speeds, steering angle, and brake light switch state. The serial
monitor baud rate is 115200, matching `monitor_speed` in `platformio.ini`:

```
pio device monitor
```

## Resources

* [MS4X Wiki — E46 CAN Bus](https://www.ms4x.net/index.php?title=Siemens_MS43_CAN_Bus)
* [MiataTurbo Forum — MK60 ABS Installation Guide](https://www.miataturbo.net/suspension-brakes-drivetrain-49/mk60-abs-installation-guide-100731/page15/#post1612591)
* [timurrrr/ft86 — BRZ/86 gen1 CAN database](https://github.com/timurrrr/ft86)
* [PlatformIO IDE for VSCode](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
* [Adafruit Feather M4 CAN Express](https://learn.adafruit.com/adafruit-feather-m4-can-express)

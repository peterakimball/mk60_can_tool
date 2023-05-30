# MK60 CAN Tool

## Intent

This application communicates with an ABS control module, typically referred to as _MK60_, which is found on many 2000s-era BMWs and commonly used in amateur automobile racing.

At this time, the only function provided is to respond to CAN bus transmission requests from the ABS module and provide a few data messages (simulating other CAN devices), which triggers the ABS module to begin transmitting wheel speed data via the CAN bus.

## Usage

At this time, installation and usage is left as an exercise to the reader.

## Compatible Hardware

* [Adafruit Feather M4 CAN Express](https://www.digikey.com/catalog/en/partgroup/sam-d5x-e5x/70620) 

## Resources

Invaluable information came from the following sources:

* [MS4X Wiki](https://www.ms4x.net/index.php?title=Siemens_MS43_CAN_Bus)
* [MiataTurbo Forum](https://www.miataturbo.net/suspension-brakes-drivetrain-49/mk60-abs-installation-guide-100731/page15/#post1612591)
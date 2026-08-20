# Firmware

This folder contains the embedded firmware for the Gustave rocket avionics and its CanSats, both running on Teensy 4.1 boards.

## Structure

```
Firmware/
├── Rocket_main.ino
├── CanSat_main.ino
└── README.md
```

## Rocket vs CanSat

The rocket and the CanSats run separate firmware, since they don't do exactly the same job. All CanSats share the same code, each physical CanSat just gets a different `CANSAT_ID` flashed in, used both for UWB ranging and to tag its data on the SD card.

## What the firmware does

**Rocket:**
- Reads GPS, IMU, and barometer data
- Acts as the UWB anchor, ranging with up to 6 CanSats (asymmetric two-way ranging, compensated for clock drift)
- Sends telemetry over LoRa
- Logs everything to two SD files in parallel (main + backup) so we don't lose data if one write fails
- Detects launch via a physical jack pin removed at liftoff. In the current test version, CanSat ejection motors are triggered by a manual "ready" switch; the final logic will fire the motors after a precise delay computed from the rocket's internal timer, following acceleration-based launch detection.
- Detects landing from a combination of low acceleration + low altitude sustained for 10 seconds, then stops logging

**CanSat:**
- Detects its own launch by watching for a sustained acceleration spike (not a fixed altitude/time — it reacts to the actual liftoff), then waits a set delay before triggering separation. The delay is currently a test value and will be tuned later based on actual flight timing.
- Once the delay has elapsed, triggers a servo to physically separate from its release mechanism
- Reads GPS, IMU, and barometer at 10 Hz
- Responds to the rocket's UWB ranging requests and reports back its own ID
- Logs its own data to the SD card and sends it over LoRa

## Notes on hardware interfacing

- **I2C**: BMP390 (barometer) and LSM6DSOX (IMU)
- **SPI**: DW1000 (UWB), LoRa module, SD card
- **UART**: GNSS/GPS module
- **PWM**: rocket-side ejection motors (`analogWrite`) and CanSat separation servo (`Servo.write`)

## Status

The code is written and each part has been tested individually (sensors, UWB ranging, SD logging, etc.), but it hasn't been through a full integration test yet with everything running together on the assembled hardware. That's one of the next steps before flight preparation, along with LoRa range testing planned for February.

## Notes

Built using the Arduino framework. No RTOS, just a straightforward loop-based structure adapted to what a Teensy 4.1 can handle for this kind of mission.

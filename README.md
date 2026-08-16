# Gustave Avionics & Telemetry System

Experimental rocket avionics platform develop within AeroIPSA.


## Overview

Gustave is an experimental rocket developed within AeroIPSA, IPSA's student aerospace association, with a planned in march 2027.

The avionics system is responsible for:
- Flight telemetry
- CanSat deployment
- UWB-Based positioning between rocket and CanSats logged for post-flight trajectory reconstruction
- Sensor Acquisition
- Onboard data logging

## Features

- 4-layer avionics PCB (GND-5V-5V-GND stack up) plus a dedicated PCB for each CansSat
- ARM Cortex-M7 firmware (Teensy 4.1)
- LoRa wireless telemetry
- DW1000 UWB ranging
- GPS positioning
- IMU integration
- Barometric altitude measurement
- SD card data logging
- PWM-based CanSat release at a precise time

## My contributions

### PCB Design

Designed and routed the complete 4-layer avionics PCB plus designed witch another member a dedicated PCB for each deployed CansSat using EasyEDA

### Component Selection & Integration

Selected and integrated LoRa, DW1000 UWB, GPS, IMU and barometric sensors for real-time flight telemetry and CanSat Localization.

### Embedded Firmware















# Hardware Architecture & PCB Design
 
This directory contains the hardware design files, schematics, and bill of materials (BOM) for the Gustave rocket avionics board.
 
## PCB Specifications
 
| Parameter | Specification |
| :--- | :--- |
| **EDA Tool** | EasyEDA |
| **Layer Count** | 4-layer stack-up (GND – 5V – 5V – GND) |
| **Board Dimensions** | 120.65 mm x 79.248 mm |
 
---
 
## Key Components
 
- **MCU**: Teensy 4.1 (ARM Cortex-M7)
- **Wireless**: LoRa module, DW1000 UWB transceiver
- **Sensors**: GPS module, IMU (LSM6DSOX), barometric sensor
- **Storage**: SD card module for onboard data logging
## Communication Interfaces
 
The board uses UART, SPI, and I2C to interface the Teensy 4.1 with onboard peripherals (sensors, UWB module, SD card).
 
## UWB Design Consideration
 
The DW1000 UWB module has a manufacturer-specified antenna keepout zone that had to be respected during component placement and routing this constrained the layout and required rethinking the placement of nearby components.
 
The UWB ranging link is configured on Channel 5, 110 kbps, PRF 64 MHz with the TX power register set per Qorvo/Decawave's certified reference values (Table 20, DW1000 User Manual) to ensure regulatory spectral compliance.
 
---
 
## File Manifest
 
- `schematic.pdf` — Complete circuit schematic, exported from EasyEDA.
- `BOM.csv` — Bill of materials with component references and footprints.

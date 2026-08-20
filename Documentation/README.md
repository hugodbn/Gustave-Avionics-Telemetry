# Documentation

This folder gathers design notes and reference material for the Gustave avionics system. It's still fairly light for now — it will grow as the project moves toward flight testing (photos of the assembled hardware, test notes, etc. are expected after February).

---

## Component Selection & Design Rationale

### Why the Decawave DW1000 for UWB?
The DW1000 was selected primarily because it had already been used within AéroIPSA on previous projects. Reusing a component the team already had experience with helped:

- Reduce bring-up risk, since the module and its basic driver behavior were already familiar to the team.
- Focus engineering effort on the ranging implementation (asymmetric two-way ranging) rather than debugging an unfamiliar transceiver from scratch.

> **Note on RF Configuration:** While the hardware selection leveraged association heritage, the RF parameters (**Channel 5**, **110 kbps**, **PRF 64 MHz**, and manual TX power calibration per **Table 20 of the DW1000 User Manual**) were derived directly from official Qorvo/Decawave specifications to ensure strict regulatory spectral compliance.
---

## Official Reference Documentation

To keep this repository lightweight, reference datasheets are linked directly to official manufacturer sources rather than redistributed here:

- **UWB Transceiver:** [Qorvo / Decawave DW1000 User Manual](https://www.qorvo.com/products/p/DW1000)
- **Microcontroller:** [Teensy 4.1 Technical Specifications (PJRC)](https://www.pjrc.com/store/teensy41.html)
- **Barometric Altimeter:** [Bosch Sensortec BMP390 Datasheet](https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/bmp390/)
- **Inertial Measurement Unit:** [STMicroelectronics LSM6DSOX Datasheet](https://www.st.com/en/mems-and-sensors/lsm6dsox.html)

---

## Coming later

- Bench test notes from sensor calibration and UWB ranging validation
- LoRa range test results (scheduled February)
- Photos of the assembled hardware / test bench

# ESP32 Joint Controller Firmware

## Overview

Arduino-framework firmware for ESP32 that:

* Receives CAN commands from the Raspberry Pi base station
* Reads an AS5600 absolute encoder (I2C) on the output shaft
* Drives a TMC2209 stepper driver via STEP/DIR/EN
* Closes a position loop on the **output joint angle** through a 21:1 cycloidal gear
* Sends telemetry (50 Hz) and heartbeat (10 Hz) frames back over CAN

## Wiring Summary

### Power
| Signal | Connection |
|---|---|
| Backbone +12 V | → LM2596 IN+, TMC2209 VM |
| Backbone GND | → common GND |
| LM2596 OUT+ | → ESP32 VIN |

### CAN
| ESP32 Pin | CAN Transceiver |
|---|---|
| GPIO 5 | TXD |
| GPIO 4 | RXD |

### TMC2209 (STEP/DIR/EN)
| ESP32 Pin | TMC2209 |
|---|---|
| GPIO 25 | STEP |
| GPIO 26 | DIR |
| GPIO 27 | EN |
| 3.3 V | VDD |

### AS5600 (I2C)
| ESP32 Pin | AS5600 |
|---|---|
| GPIO 21 | SDA |
| GPIO 22 | SCL |
| 3.3 V | VCC |

## Build & Flash

Install [PlatformIO CLI](https://platformio.org/install/cli) or use the VS Code extension.

```bash
cd esp_joint
pio run                   # compile
pio run -t upload         # flash
pio device monitor        # serial monitor @ 115200
```

## Build Flags

Set in `platformio.ini`:

| Flag | Default | Description |
|---|---|---|
| `NODE_ID` | 1 | CAN node ID for this joint |
| `CAN_TERMINATION` | 1 | Enable 120 Ω CAN termination (bench test) |
| `SERIAL_DEBUG` | 1 | Enable Serial debug output |

## Architecture

```
include/
  config.h         ← pin definitions, constants
  protocol.h       ← CAN frame pack/unpack
  encoder_as5600.h ← AS5600 I2C driver
  stepgen.h        ← hardware-timer step generator
  controller.h     ← position control loop
src/
  main.cpp         ← setup/loop, CAN RX dispatch
  protocol.cpp
  encoder_as5600.cpp
  stepgen.cpp
  controller.cpp
```

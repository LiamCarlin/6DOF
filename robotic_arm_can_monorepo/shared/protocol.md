# CAN Protocol Specification — 6-DOF Robotic Arm

## Overview

Each joint module is an ESP32 node on a shared CAN 2.0A bus (500 kbps, 11-bit IDs).
The Raspberry Pi 5 acts as the single bus master.

All multi-byte integers are **little-endian**.
Angle unit everywhere: **millidegrees (mdeg)** — 1 degree = 1000 mdeg.
All CAN payloads are exactly **8 bytes**.

---

## CAN Settings

| Parameter | Value |
|---|---|
| Bitrate | 500 000 bps |
| Standard | CAN 2.0A, 11-bit IDs |
| Payload | 8 bytes (always) |
| Endianness | Little-endian |

## CAN ID Mapping

| Direction | Formula | Example (Joint 1) |
|---|---|---|
| Base → Node command | `0x100 + node_id` | `0x101` |
| Node → Base telemetry | `0x200 + node_id` | `0x201` |
| Node → Base heartbeat | `0x300 + node_id` | `0x301` |

Node IDs: Joint 1 = 1, Joint 2 = 2, … Joint 6 = 6.

---

## COMMAND Frame (Base → Node)

CAN ID = `0x100 + node_id`

| Byte(s) | Field | Type | Description |
|---|---|---|---|
| 0 | opcode | uint8 | Operation code (see below) |
| 1 | flags | uint8 | Reserved; set to 0 |
| 2–5 | angle_mdeg | int32 LE | Meaning depends on opcode; 0 if unused |
| 6–7 | param | uint16 LE | Meaning depends on opcode; 0 if unused |

### Opcodes

| Code | Name | angle_mdeg | param | Description |
|---|---|---|---|---|
| `0x00` | KEEPALIVE | 0 | 0 | Refresh watchdog; no motion |
| `0x01` | ENABLE | 0 | 0 | Enable motor driver outputs |
| `0x02` | DISABLE | 0 | 0 | Disable motor driver outputs |
| `0x03` | STOP | 0 | 0 | Immediate stop step generation |
| `0x04` | SET_ZERO | 0 | 0 | Store current encoder reading as zero reference |
| `0x10` | SET_POS | target_output_mdeg (int32) | speed_limit (uint16 mdeg/s, 0 = firmware default) | Command target output joint angle |

---

## TELEMETRY Frame (Node → Base)

CAN ID = `0x200 + node_id`, sent at **50 Hz**

| Byte(s) | Field | Type | Description |
|---|---|---|---|
| 0–1 | status_flags | uint16 LE | Bitfield (see below) |
| 2 | fault_code | uint8 | Fault enum |
| 3 | reserved | uint8 | 0 |
| 4–7 | current_angle_mdeg | int32 LE | Current output angle |

---

## HEARTBEAT Frame (Node → Base)

CAN ID = `0x300 + node_id`, sent at **10 Hz**

| Byte(s) | Field | Type | Description |
|---|---|---|---|
| 0–3 | uptime_ms | uint32 LE | Milliseconds since boot |
| 4–5 | status_flags | uint16 LE | Same bitfield |
| 6 | fault_code | uint8 | Fault enum |
| 7 | reserved | uint8 | 0 |

---

## Status Flags Bitfield (uint16)

| Bit | Name | Description |
|---|---|---|
| 0 | ENABLED | Motor driver outputs are enabled |
| 1 | MOVING | Step generation is active |
| 2 | ZEROED | Zero reference has been set |
| 3 | AT_TARGET | Within position tolerance of target |
| 4 | ENCODER_OK | AS5600 I2C communication healthy |
| 5 | CAN_OK | CAN peripheral initialised |
| 6 | WATCHDOG_TIMEOUT | Watchdog tripped (no cmd > 300 ms) |
| 7 | LIMIT_FAULT | Soft limit exceeded |
| 8–15 | reserved | 0 |

## Fault Codes (uint8)

| Code | Name |
|---|---|
| 0 | OK |
| 1 | ENCODER_I2C_FAIL |
| 2 | CAN_INIT_FAIL |
| 3 | COMMAND_OUT_OF_RANGE |
| 4 | WATCHDOG_STOP |
| 5 | INTERNAL_ERROR |

---

## Safety & Limits

| Parameter | Value |
|---|---|
| Soft limit min | −180 000 mdeg (−180°) |
| Soft limit max | +180 000 mdeg (+180°) |
| Position tolerance | ±300 mdeg (±0.3°) |
| Watchdog timeout | 300 ms |
| Keepalive rate (Pi) | 20 Hz |

## Mechanical Parameters

| Parameter | Value |
|---|---|
| Motor full steps/rev | 200 |
| Microsteps | 16 |
| Gear ratio (output:input) | 21:1 |
| AS5600 raw range | 0–4095 (12-bit) |
| Encoder placement | Output shaft |

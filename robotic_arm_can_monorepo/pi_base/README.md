# Raspberry Pi 5 — Base Station for 6-DOF Robotic Arm

## Prerequisites

* Raspberry Pi 5 with Raspberry Pi OS (64-bit Bookworm or later)
* RS485/CAN HAT Rev 2.1 installed and overlay enabled
* Python 3.9+

## Hardware Setup

1. Attach the RS485/CAN HAT Rev 2.1 to the Pi 5 GPIO header.
2. Enable the CAN overlay — add the following to `/boot/firmware/config.txt`:

```
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25,spimaxfrequency=2000000
```

> **Note:** Check your HAT documentation for the correct overlay name and oscillator
> frequency. Some HATs use `seeed-can-fd-hat-v2` or similar. Adjust accordingly.

3. Reboot.

## Bring up CAN interface

```bash
cd scripts
chmod +x can_up.sh
sudo ./can_up.sh
```

This configures `can0` at 500 kbps.

Verify:

```bash
ip -d link show can0
```

## Install Python dependencies

```bash
cd pi_base
pip install -r requirements.txt
# or
pip install -e .
```

## Run the Tkinter UI

```bash
python -m arm_pi.ui_tk
```

## CLI usage

```bash
# Enable Joint 1
python -m arm_pi.cli enable --node 1

# Set position to 30° at 60000 mdeg/s
python -m arm_pi.cli setpos --node 1 --deg 30 --speed 60000

# Set zero reference
python -m arm_pi.cli zero --node 1

# Stop
python -m arm_pi.cli stop --node 1

# Disable
python -m arm_pi.cli disable --node 1

# Read latest status
python -m arm_pi.cli status --node 1
```

## Architecture

```
arm_pi/
  protocol.py      ← CAN frame pack/unpack; loads shared/protocol_constants.json
  can_bus.py        ← python-can SocketCAN wrapper
  node_client.py    ← High-level joint client (send cmds, receive telemetry)
  ui_tk.py          ← Tkinter GUI
  cli.py            ← CLI entry-point
```

#!/usr/bin/env bash
# can_up.sh — bring up SocketCAN interface can0 at 500 kbps
# Run with:  sudo ./can_up.sh

set -euo pipefail

IFACE="can0"
BITRATE=500000

echo "Bringing down $IFACE (ignore errors if already down)…"
sudo ip link set "$IFACE" down 2>/dev/null || true

echo "Configuring $IFACE  bitrate=$BITRATE"
sudo ip link set "$IFACE" type can bitrate "$BITRATE"

echo "Bringing up $IFACE"
sudo ip link set "$IFACE" up

echo "Done.  Verify with:  ip -d link show $IFACE"
ip -d link show "$IFACE"

"""
CLI for controlling joint nodes over CAN.

Usage:
    python -m arm_pi.cli enable   --node 1
    python -m arm_pi.cli disable  --node 1
    python -m arm_pi.cli stop     --node 1
    python -m arm_pi.cli zero     --node 1
    python -m arm_pi.cli setpos   --node 1 --deg 30 --speed 60000
    python -m arm_pi.cli status   --node 1
"""

from __future__ import annotations

import argparse
import sys
import time

from . import protocol as proto
from .can_bus import CANBus
from .node_client import JointNodeClient


def _make_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="arm_pi.cli",
        description="CAN joint-node command-line interface",
    )
    p.add_argument("--channel", default="can0", help="SocketCAN interface (default: can0)")
    sub = p.add_subparsers(dest="command", required=True)

    # enable
    s = sub.add_parser("enable", help="Enable motor driver")
    s.add_argument("--node", type=int, default=1, help="Node ID (default: 1)")

    # disable
    s = sub.add_parser("disable", help="Disable motor driver")
    s.add_argument("--node", type=int, default=1)

    # stop
    s = sub.add_parser("stop", help="Immediate stop")
    s.add_argument("--node", type=int, default=1)

    # zero
    s = sub.add_parser("zero", help="Set current position as zero")
    s.add_argument("--node", type=int, default=1)

    # setpos
    s = sub.add_parser("setpos", help="Set target position")
    s.add_argument("--node", type=int, default=1)
    s.add_argument("--deg", type=float, required=True, help="Target angle in degrees")
    s.add_argument("--speed", type=int, default=0, help="Speed limit mdeg/s (0=default)")

    # status
    s = sub.add_parser("status", help="Read latest telemetry / heartbeat")
    s.add_argument("--node", type=int, default=1)
    s.add_argument("--wait", type=float, default=0.5, help="Seconds to listen for frames")

    return p


def main(argv: list[str] | None = None) -> None:
    args = _make_parser().parse_args(argv)

    with CANBus(channel=args.channel) as bus:
        node = JointNodeClient(bus, node_id=args.node)

        cmd = args.command

        if cmd == "enable":
            node.enable()
            print(f"→ ENABLE sent to node {args.node}")

        elif cmd == "disable":
            node.disable()
            print(f"→ DISABLE sent to node {args.node}")

        elif cmd == "stop":
            node.stop()
            print(f"→ STOP sent to node {args.node}")

        elif cmd == "zero":
            node.set_zero()
            print(f"→ SET_ZERO sent to node {args.node}")

        elif cmd == "setpos":
            node.set_pos(args.deg, args.speed)
            mdeg = proto.deg_to_mdeg(args.deg)
            print(f"→ SET_POS {args.deg}° ({mdeg} mdeg), speed={args.speed} sent to node {args.node}")

        elif cmd == "status":
            print(f"Listening for telemetry/heartbeat from node {args.node} for {args.wait}s …")
            node.start(keepalive=False)
            time.sleep(args.wait)
            node.stop_threads()

            state = node.state
            if state.telemetry:
                t = state.telemetry
                print(f"  Angle  : {t.current_angle_deg:.2f}° ({t.current_angle_mdeg} mdeg)")
                print(f"  Flags  : {proto.format_status_flags(t.status_flags)}")
                print(f"  Fault  : {t.fault_name}")
            else:
                print("  (no telemetry received)")

            if state.heartbeat:
                hb = state.heartbeat
                print(f"  Uptime : {hb.uptime_ms / 1000:.1f}s")
                print(f"  HB Flags: {proto.format_status_flags(hb.status_flags)}")
                print(f"  HB Fault: {hb.fault_name}")
            else:
                print("  (no heartbeat received)")

        else:
            print(f"Unknown command: {cmd}", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    main()

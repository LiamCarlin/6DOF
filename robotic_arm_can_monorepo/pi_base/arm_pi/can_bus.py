"""
Thin wrapper around python-can for SocketCAN on the Raspberry Pi.
"""

from __future__ import annotations

import can
from typing import Optional

from . import protocol as proto


DEFAULT_CHANNEL = "can0"
DEFAULT_BITRATE = proto.CAN_BITRATE  # 500 000


class CANBus:
    """Manage a SocketCAN connection."""

    def __init__(self, channel: str = DEFAULT_CHANNEL, bitrate: int = DEFAULT_BITRATE):
        self.channel = channel
        self.bitrate = bitrate
        self._bus: Optional[can.Bus] = None

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        """Open the CAN bus (SocketCAN — must already be 'up')."""
        self._bus = can.Bus(
            channel=self.channel,
            interface="socketcan",
            bitrate=self.bitrate,
        )

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    @property
    def bus(self) -> can.Bus:
        if self._bus is None:
            raise RuntimeError("CAN bus is not open — call open() first")
        return self._bus

    # ------------------------------------------------------------------
    # send / receive
    # ------------------------------------------------------------------

    def send(self, arb_id: int, data: bytes) -> None:
        msg = can.Message(
            arbitration_id=arb_id,
            data=data,
            is_extended_id=False,
        )
        self.bus.send(msg)

    def recv(self, timeout: float = 0.1) -> Optional[can.Message]:
        """Receive one CAN frame (blocking up to *timeout* seconds)."""
        return self.bus.recv(timeout=timeout)

    # ------------------------------------------------------------------
    # context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "CANBus":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

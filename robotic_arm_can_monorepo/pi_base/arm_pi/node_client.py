"""
High-level client for a single joint node.

Provides threaded CAN receive, keepalive, and convenient command methods.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from . import protocol as proto
from .can_bus import CANBus


@dataclass
class NodeState:
    """Last-known state of a joint node, updated from telemetry + heartbeat."""
    telemetry: Optional[proto.TelemetryFrame] = None
    heartbeat: Optional[proto.HeartbeatFrame] = None
    last_telemetry_time: float = 0.0
    last_heartbeat_time: float = 0.0

    @property
    def online(self) -> bool:
        """Consider node online if heartbeat received within 0.25 s."""
        return (time.time() - self.last_heartbeat_time) < 0.25


class JointNodeClient:
    """Manage communication with one joint node."""

    def __init__(
        self,
        bus: CANBus,
        node_id: int = 1,
        on_state_update: Optional[Callable[[NodeState], None]] = None,
    ):
        self.bus = bus
        self.node_id = node_id
        self.state = NodeState()
        self._on_state_update = on_state_update

        self._cmd_id = proto.command_id(node_id)
        self._telem_id = proto.telemetry_id(node_id)
        self._hb_id = proto.heartbeat_id(node_id)

        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._ka_thread: Optional[threading.Thread] = None
        self._keepalive_enabled = False
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # send helpers
    # ------------------------------------------------------------------

    def send_command(self, data: bytes) -> None:
        self.bus.send(self._cmd_id, data)

    def enable(self) -> None:
        self.send_command(proto.pack_enable())

    def disable(self) -> None:
        self.send_command(proto.pack_disable())

    def stop(self) -> None:
        self.send_command(proto.pack_stop())

    def set_zero(self) -> None:
        self.send_command(proto.pack_set_zero())

    def set_pos(self, deg: float, speed_mdeg_s: int = 0) -> None:
        mdeg = proto.deg_to_mdeg(deg)
        self.send_command(proto.pack_set_pos(mdeg, speed_mdeg_s))

    def keepalive(self) -> None:
        self.send_command(proto.pack_keepalive())

    # ------------------------------------------------------------------
    # receive loop
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        while self._running:
            msg = self.bus.recv(timeout=0.05)
            if msg is None:
                continue
            now = time.time()
            try:
                if msg.arbitration_id == self._telem_id and len(msg.data) == 8:
                    telem = proto.unpack_telemetry(bytes(msg.data))
                    with self._lock:
                        self.state.telemetry = telem
                        self.state.last_telemetry_time = now
                    if self._on_state_update:
                        self._on_state_update(self.state)

                elif msg.arbitration_id == self._hb_id and len(msg.data) == 8:
                    hb = proto.unpack_heartbeat(bytes(msg.data))
                    with self._lock:
                        self.state.heartbeat = hb
                        self.state.last_heartbeat_time = now
                    if self._on_state_update:
                        self._on_state_update(self.state)
            except Exception:
                pass  # robust against malformed frames

    # ------------------------------------------------------------------
    # keepalive loop
    # ------------------------------------------------------------------

    def _ka_loop(self) -> None:
        interval = 1.0 / proto.CONSTANTS["limits"]["keepalive_hz"]  # 50 ms
        while self._running:
            if self._keepalive_enabled:
                try:
                    self.keepalive()
                except Exception:
                    pass
            time.sleep(interval)

    # ------------------------------------------------------------------
    # start / stop background threads
    # ------------------------------------------------------------------

    def start(self, keepalive: bool = True) -> None:
        self._running = True
        self._keepalive_enabled = keepalive
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self._ka_thread = threading.Thread(target=self._ka_loop, daemon=True)
        self._ka_thread.start()

    def stop_threads(self) -> None:
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        if self._ka_thread:
            self._ka_thread.join(timeout=1.0)

    def set_keepalive(self, enabled: bool) -> None:
        self._keepalive_enabled = enabled

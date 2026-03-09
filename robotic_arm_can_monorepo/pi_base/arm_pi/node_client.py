"""
High-level client for joint nodes on the CAN bus.

Supports multiple joints simultaneously with a single shared RX thread.
Provides threaded CAN receive, per-node keepalive, and convenient command methods.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

from . import protocol as proto
from .can_bus import CANBus


NUM_JOINTS = 6


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
        on_state_update: Optional[Callable[[int, NodeState], None]] = None,
    ):
        self.bus = bus
        self.node_id = node_id
        self.state = NodeState()
        self._on_state_update = on_state_update

        self._cmd_id = proto.command_id(node_id)
        self._telem_id = proto.telemetry_id(node_id)
        self._hb_id = proto.heartbeat_id(node_id)

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
    # process a received CAN message (called by ArmBusManager)
    # ------------------------------------------------------------------

    def process_message(self, arb_id: int, data: bytes, now: float) -> None:
        try:
            if arb_id == self._telem_id and len(data) == 8:
                telem = proto.unpack_telemetry(data)
                with self._lock:
                    self.state.telemetry = telem
                    self.state.last_telemetry_time = now
                if self._on_state_update:
                    self._on_state_update(self.node_id, self.state)

            elif arb_id == self._hb_id and len(data) == 8:
                hb = proto.unpack_heartbeat(data)
                with self._lock:
                    self.state.heartbeat = hb
                    self.state.last_heartbeat_time = now
                if self._on_state_update:
                    self._on_state_update(self.node_id, self.state)
        except Exception:
            pass  # robust against malformed frames

    # ------------------------------------------------------------------
    # standalone mode (for CLI one-shot usage without ArmBusManager)
    # ------------------------------------------------------------------

    def start(self, keepalive: bool = True) -> None:
        """Start a private RX thread (and optional keepalive) for this
        single node.  Use this only in the CLI — the Tk UI should use
        ArmBusManager instead."""
        self._standalone_running = True
        if keepalive:
            self._keepalive_enabled = True

        self._standalone_rx = threading.Thread(target=self._standalone_rx_loop, daemon=True)
        self._standalone_rx.start()

        if keepalive:
            self._standalone_ka = threading.Thread(target=self._standalone_ka_loop, daemon=True)
            self._standalone_ka.start()

    def stop_threads(self) -> None:
        self._standalone_running = False
        if hasattr(self, "_standalone_rx"):
            self._standalone_rx.join(timeout=1.0)
        if hasattr(self, "_standalone_ka"):
            self._standalone_ka.join(timeout=1.0)

    def _standalone_rx_loop(self) -> None:
        while self._standalone_running:
            msg = self.bus.recv(timeout=0.05)
            if msg is None:
                continue
            self.process_message(msg.arbitration_id, bytes(msg.data), time.time())

    def _standalone_ka_loop(self) -> None:
        interval = 1.0 / proto.CONSTANTS["limits"]["keepalive_hz"]
        while self._standalone_running:
            try:
                self.keepalive()
            except Exception:
                pass
            time.sleep(interval)


class ArmBusManager:
    """Manage all joint nodes on a single CAN bus.

    One shared RX thread listens for all telemetry/heartbeat frames and
    dispatches them to the correct JointNodeClient.
    One shared keepalive thread sends keepalive to all enabled nodes.
    """

    def __init__(
        self,
        bus: CANBus,
        node_ids: list[int] | None = None,
        on_state_update: Optional[Callable[[int, NodeState], None]] = None,
    ):
        self.bus = bus
        if node_ids is None:
            node_ids = list(range(1, NUM_JOINTS + 1))

        self.nodes: Dict[int, JointNodeClient] = {}
        for nid in node_ids:
            self.nodes[nid] = JointNodeClient(bus, nid, on_state_update=on_state_update)

        self._running = False
        self._rx_thread: Optional[threading.Thread] = None
        self._ka_thread: Optional[threading.Thread] = None

        # Build lookup: arb_id → (node_id, client)
        self._id_map: Dict[int, JointNodeClient] = {}
        for nid, client in self.nodes.items():
            self._id_map[client._telem_id] = client
            self._id_map[client._hb_id] = client

    def __getitem__(self, node_id: int) -> JointNodeClient:
        return self.nodes[node_id]

    # ------------------------------------------------------------------
    # receive loop (shared for all nodes)
    # ------------------------------------------------------------------

    def _rx_loop(self) -> None:
        while self._running:
            msg = self.bus.recv(timeout=0.05)
            if msg is None:
                continue
            now = time.time()
            client = self._id_map.get(msg.arbitration_id)
            if client is not None:
                client.process_message(msg.arbitration_id, bytes(msg.data), now)

    # ------------------------------------------------------------------
    # keepalive loop (sends to all nodes that have keepalive enabled)
    # ------------------------------------------------------------------

    def _ka_loop(self) -> None:
        interval = 1.0 / proto.CONSTANTS["limits"]["keepalive_hz"]  # 50 ms
        while self._running:
            for client in self.nodes.values():
                if client._keepalive_enabled:
                    try:
                        client.keepalive()
                    except Exception:
                        pass
            time.sleep(interval)

    # ------------------------------------------------------------------
    # start / stop
    # ------------------------------------------------------------------

    def start(self, keepalive_nodes: list[int] | None = None) -> None:
        """Start RX and keepalive threads.

        Args:
            keepalive_nodes: Node IDs to send keepalive to. None = all.
        """
        self._running = True

        if keepalive_nodes is None:
            keepalive_nodes = list(self.nodes.keys())
        for nid in keepalive_nodes:
            if nid in self.nodes:
                self.nodes[nid]._keepalive_enabled = True

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

    def set_keepalive(self, node_id: int, enabled: bool) -> None:
        if node_id in self.nodes:
            self.nodes[node_id]._keepalive_enabled = enabled

"""
CAN protocol helpers — pack / unpack command, telemetry, and heartbeat frames.

Constants are loaded from shared/protocol_constants.json so there is one source
of truth shared with the ESP32 firmware documentation.
"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# ---------------------------------------------------------------------------
# Load shared constants
# ---------------------------------------------------------------------------
_CONSTANTS_PATH = Path(__file__).resolve().parents[2] / "shared" / "protocol_constants.json"
with open(_CONSTANTS_PATH, "r") as _f:
    CONSTANTS: dict = json.load(_f)

# CAN ID bases (stored as hex strings in JSON)
COMMAND_BASE   = int(CONSTANTS["can_id"]["command_base"], 16)    # 0x100
TELEMETRY_BASE = int(CONSTANTS["can_id"]["telemetry_base"], 16)  # 0x200
HEARTBEAT_BASE = int(CONSTANTS["can_id"]["heartbeat_base"], 16)  # 0x300

# Opcodes
OP_KEEPALIVE = int(CONSTANTS["opcodes"]["KEEPALIVE"], 16)
OP_ENABLE    = int(CONSTANTS["opcodes"]["ENABLE"], 16)
OP_DISABLE   = int(CONSTANTS["opcodes"]["DISABLE"], 16)
OP_STOP      = int(CONSTANTS["opcodes"]["STOP"], 16)
OP_SET_ZERO  = int(CONSTANTS["opcodes"]["SET_ZERO"], 16)
OP_SET_POS   = int(CONSTANTS["opcodes"]["SET_POS"], 16)

OPCODE_NAMES = {v: k for k, v in {
    "KEEPALIVE": OP_KEEPALIVE,
    "ENABLE":    OP_ENABLE,
    "DISABLE":   OP_DISABLE,
    "STOP":      OP_STOP,
    "SET_ZERO":  OP_SET_ZERO,
    "SET_POS":   OP_SET_POS,
}.items()}

# Status flag bit indices
STATUS_BITS = CONSTANTS["status_bits"]

# Fault codes
FAULT_CODES = {v: k for k, v in CONSTANTS["fault_codes"].items()}

# Limits  (no soft angle limits — unlimited multi-turn)
MDEG_PER_DEG        = CONSTANTS["units"]["mdeg_per_degree"]

CAN_BITRATE = CONSTANTS["can"]["bitrate"]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def command_id(node_id: int) -> int:
    return COMMAND_BASE + node_id

def telemetry_id(node_id: int) -> int:
    return TELEMETRY_BASE + node_id

def heartbeat_id(node_id: int) -> int:
    return HEARTBEAT_BASE + node_id


def deg_to_mdeg(deg: float) -> int:
    return int(round(deg * MDEG_PER_DEG))

def mdeg_to_deg(mdeg: int) -> float:
    return mdeg / MDEG_PER_DEG


# ---------------------------------------------------------------------------
# COMMAND packing
# ---------------------------------------------------------------------------

def pack_command(opcode: int, angle_mdeg: int = 0, param: int = 0, flags: int = 0) -> bytes:
    """Pack an 8-byte command payload.

    Layout (all LE):
      byte 0   : opcode   (uint8)
      byte 1   : flags    (uint8)
      byte 2-5 : angle_mdeg (int32)
      byte 6-7 : param    (uint16)
    """
    return struct.pack("<BBiH", opcode, flags, angle_mdeg, param)


def pack_keepalive() -> bytes:
    return pack_command(OP_KEEPALIVE)

def pack_enable() -> bytes:
    return pack_command(OP_ENABLE)

def pack_disable() -> bytes:
    return pack_command(OP_DISABLE)

def pack_stop() -> bytes:
    return pack_command(OP_STOP)

def pack_set_zero() -> bytes:
    return pack_command(OP_SET_ZERO)

def pack_set_pos(target_mdeg: int, speed_mdeg_s: int = 0) -> bytes:
    """Pack a SET_POS command.

    Args:
        target_mdeg: Target output angle in millidegrees.
        speed_mdeg_s: Speed limit in mdeg/s (uint16, 0 = firmware default).
    """
    return pack_command(OP_SET_POS, angle_mdeg=target_mdeg, param=speed_mdeg_s & 0xFFFF)


# ---------------------------------------------------------------------------
# TELEMETRY unpacking
# ---------------------------------------------------------------------------

@dataclass
class TelemetryFrame:
    status_flags: int      # uint16
    fault_code: int        # uint8
    current_angle_mdeg: int  # int32

    @property
    def current_angle_deg(self) -> float:
        return mdeg_to_deg(self.current_angle_mdeg)

    # convenience flag helpers
    def flag(self, name: str) -> bool:
        bit = STATUS_BITS[name]
        return bool(self.status_flags & (1 << bit))

    @property
    def enabled(self) -> bool:     return self.flag("ENABLED")
    @property
    def moving(self) -> bool:      return self.flag("MOVING")
    @property
    def zeroed(self) -> bool:      return self.flag("ZEROED")
    @property
    def at_target(self) -> bool:   return self.flag("AT_TARGET")
    @property
    def encoder_ok(self) -> bool:  return self.flag("ENCODER_OK")
    @property
    def can_ok(self) -> bool:      return self.flag("CAN_OK")
    @property
    def watchdog_timeout(self) -> bool: return self.flag("WATCHDOG_TIMEOUT")
    @property
    def limit_fault(self) -> bool: return self.flag("LIMIT_FAULT")

    @property
    def fault_name(self) -> str:
        return FAULT_CODES.get(self.fault_code, f"UNKNOWN({self.fault_code})")


def unpack_telemetry(data: bytes) -> TelemetryFrame:
    """Unpack an 8-byte telemetry payload."""
    if len(data) != 8:
        raise ValueError(f"Telemetry frame must be 8 bytes, got {len(data)}")
    status_flags, fault_code, _reserved, current_angle_mdeg = struct.unpack("<HBBi", data)
    return TelemetryFrame(
        status_flags=status_flags,
        fault_code=fault_code,
        current_angle_mdeg=current_angle_mdeg,
    )


# ---------------------------------------------------------------------------
# HEARTBEAT unpacking
# ---------------------------------------------------------------------------

@dataclass
class HeartbeatFrame:
    uptime_ms: int       # uint32
    status_flags: int    # uint16
    fault_code: int      # uint8

    def flag(self, name: str) -> bool:
        bit = STATUS_BITS[name]
        return bool(self.status_flags & (1 << bit))

    @property
    def fault_name(self) -> str:
        return FAULT_CODES.get(self.fault_code, f"UNKNOWN({self.fault_code})")


def unpack_heartbeat(data: bytes) -> HeartbeatFrame:
    """Unpack an 8-byte heartbeat payload."""
    if len(data) != 8:
        raise ValueError(f"Heartbeat frame must be 8 bytes, got {len(data)}")
    uptime_ms, status_flags, fault_code, _reserved = struct.unpack("<IHBB", data)
    return HeartbeatFrame(
        uptime_ms=uptime_ms,
        status_flags=status_flags,
        fault_code=fault_code,
    )


# ---------------------------------------------------------------------------
# ID classification
# ---------------------------------------------------------------------------

def classify_id(can_id: int) -> Optional[tuple[str, int]]:
    """Return (frame_type, node_id) or None if not recognised.

    frame_type is one of 'command', 'telemetry', 'heartbeat'.
    """
    if COMMAND_BASE < can_id <= COMMAND_BASE + 6:
        return ("command", can_id - COMMAND_BASE)
    if TELEMETRY_BASE < can_id <= TELEMETRY_BASE + 6:
        return ("telemetry", can_id - TELEMETRY_BASE)
    if HEARTBEAT_BASE < can_id <= HEARTBEAT_BASE + 6:
        return ("heartbeat", can_id - HEARTBEAT_BASE)
    return None


def format_status_flags(flags: int) -> str:
    """Human-readable status flag string."""
    parts = []
    for name, bit in STATUS_BITS.items():
        if flags & (1 << bit):
            parts.append(name)
    return " | ".join(parts) if parts else "NONE"

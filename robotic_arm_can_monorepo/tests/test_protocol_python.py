"""
Test suite for the Python CAN protocol pack / unpack functions.

Run:  pytest tests/test_protocol_python.py -v
      (from the monorepo root, with pi_base on PYTHONPATH or installed)
"""

import struct
import sys
from pathlib import Path

# Ensure pi_base is importable
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "pi_base"))

from arm_pi import protocol as proto


# =====================================================================
# Constants sanity checks
# =====================================================================

class TestConstants:
    def test_can_bitrate(self):
        assert proto.CAN_BITRATE == 500_000

    def test_command_base(self):
        assert proto.COMMAND_BASE == 0x100

    def test_telemetry_base(self):
        assert proto.TELEMETRY_BASE == 0x200

    def test_heartbeat_base(self):
        assert proto.HEARTBEAT_BASE == 0x300

    def test_opcodes(self):
        assert proto.OP_KEEPALIVE == 0x00
        assert proto.OP_ENABLE    == 0x01
        assert proto.OP_DISABLE   == 0x02
        assert proto.OP_STOP      == 0x03
        assert proto.OP_SET_ZERO  == 0x04
        assert proto.OP_SET_POS   == 0x10


# =====================================================================
# ID mapping
# =====================================================================

class TestIDMapping:
    def test_command_id_joint1(self):
        assert proto.command_id(1) == 0x101

    def test_telemetry_id_joint1(self):
        assert proto.telemetry_id(1) == 0x201

    def test_heartbeat_id_joint1(self):
        assert proto.heartbeat_id(1) == 0x301

    def test_command_id_joint6(self):
        assert proto.command_id(6) == 0x106


# =====================================================================
# Angle conversion
# =====================================================================

class TestAngleConversion:
    def test_deg_to_mdeg(self):
        assert proto.deg_to_mdeg(45.0) == 45000

    def test_mdeg_to_deg(self):
        assert proto.mdeg_to_deg(90000) == 90.0

    def test_negative(self):
        assert proto.deg_to_mdeg(-180.0) == -180000

    def test_roundtrip(self):
        for deg in [-180, -90, 0, 45, 90, 179.5]:
            assert abs(proto.mdeg_to_deg(proto.deg_to_mdeg(deg)) - deg) < 0.01


# =====================================================================
# Command packing
# =====================================================================

class TestCommandPacking:
    def test_keepalive_is_8_bytes(self):
        data = proto.pack_keepalive()
        assert len(data) == 8

    def test_keepalive_opcode(self):
        data = proto.pack_keepalive()
        assert data[0] == proto.OP_KEEPALIVE
        assert data[1] == 0  # flags

    def test_enable(self):
        data = proto.pack_enable()
        assert data[0] == proto.OP_ENABLE
        assert len(data) == 8

    def test_disable(self):
        data = proto.pack_disable()
        assert data[0] == proto.OP_DISABLE

    def test_stop(self):
        data = proto.pack_stop()
        assert data[0] == proto.OP_STOP

    def test_set_zero(self):
        data = proto.pack_set_zero()
        assert data[0] == proto.OP_SET_ZERO

    def test_set_pos_positive(self):
        data = proto.pack_set_pos(45000, 60000)
        assert len(data) == 8
        assert data[0] == proto.OP_SET_POS
        assert data[1] == 0  # flags
        # bytes 2-5: angle_mdeg = 45000 LE
        angle = struct.unpack_from("<i", data, 2)[0]
        assert angle == 45000
        # bytes 6-7: param = 60000 LE
        param = struct.unpack_from("<H", data, 6)[0]
        assert param == 60000

    def test_set_pos_negative(self):
        data = proto.pack_set_pos(-90000, 0)
        angle = struct.unpack_from("<i", data, 2)[0]
        assert angle == -90000

    def test_all_commands_8_bytes(self):
        for fn in [
            proto.pack_keepalive,
            proto.pack_enable,
            proto.pack_disable,
            proto.pack_stop,
            proto.pack_set_zero,
        ]:
            assert len(fn()) == 8


# =====================================================================
# Telemetry unpacking
# =====================================================================

class TestTelemetryUnpacking:
    def _make_telem(self, flags=0, fault=0, angle_mdeg=0):
        return struct.pack("<HBBi", flags, fault, 0, angle_mdeg)

    def test_basic(self):
        data = self._make_telem(flags=0x0011, fault=0, angle_mdeg=30000)
        t = proto.unpack_telemetry(data)
        assert t.status_flags == 0x0011
        assert t.fault_code == 0
        assert t.current_angle_mdeg == 30000
        assert abs(t.current_angle_deg - 30.0) < 0.01

    def test_flags(self):
        flags = (1 << proto.STATUS_BITS["ENABLED"]) | (1 << proto.STATUS_BITS["AT_TARGET"])
        data = self._make_telem(flags=flags)
        t = proto.unpack_telemetry(data)
        assert t.enabled is True
        assert t.at_target is True
        assert t.moving is False

    def test_fault_name(self):
        data = self._make_telem(fault=4)
        t = proto.unpack_telemetry(data)
        assert t.fault_name == "WATCHDOG_STOP"

    def test_negative_angle(self):
        data = self._make_telem(angle_mdeg=-45000)
        t = proto.unpack_telemetry(data)
        assert t.current_angle_mdeg == -45000

    def test_wrong_length_raises(self):
        import pytest
        with pytest.raises(ValueError):
            proto.unpack_telemetry(b"\x00" * 7)


# =====================================================================
# Heartbeat unpacking
# =====================================================================

class TestHeartbeatUnpacking:
    def _make_hb(self, uptime=12345, flags=0, fault=0):
        return struct.pack("<IHBB", uptime, flags, fault, 0)

    def test_basic(self):
        data = self._make_hb(uptime=9999, flags=0x05, fault=0)
        hb = proto.unpack_heartbeat(data)
        assert hb.uptime_ms == 9999
        assert hb.status_flags == 0x05
        assert hb.fault_code == 0

    def test_fault_name(self):
        data = self._make_hb(fault=2)
        hb = proto.unpack_heartbeat(data)
        assert hb.fault_name == "CAN_INIT_FAIL"

    def test_wrong_length(self):
        import pytest
        with pytest.raises(ValueError):
            proto.unpack_heartbeat(b"\x00" * 5)


# =====================================================================
# ID classification
# =====================================================================

class TestClassify:
    def test_command(self):
        assert proto.classify_id(0x101) == ("command", 1)

    def test_telemetry(self):
        assert proto.classify_id(0x203) == ("telemetry", 3)

    def test_heartbeat(self):
        assert proto.classify_id(0x305) == ("heartbeat", 5)

    def test_unknown(self):
        assert proto.classify_id(0x400) is None
        assert proto.classify_id(0x100) is None  # base without node_id


# =====================================================================
# Cross-check: Python pack ↔ C struct layout
# =====================================================================

class TestCrossCheck:
    """Verify that Python packing matches the byte layout expected by the
    ESP32 C code (memcpy from the same byte offsets)."""

    def test_command_layout(self):
        # Build a SET_POS command manually and verify field positions
        data = proto.pack_set_pos(target_mdeg=-123456, speed_mdeg_s=5000)
        # byte 0: opcode
        assert data[0] == 0x10
        # byte 1: flags
        assert data[1] == 0x00
        # bytes 2-5: int32 LE
        assert struct.unpack_from("<i", data, 2)[0] == -123456
        # bytes 6-7: uint16 LE
        assert struct.unpack_from("<H", data, 6)[0] == 5000

    def test_telemetry_layout(self):
        # Manually build bytes and verify unpack
        status = 0x001F   # first 5 bits set
        fault  = 3
        angle  = -180000
        raw = struct.pack("<HBBi", status, fault, 0, angle)
        t = proto.unpack_telemetry(raw)
        assert t.status_flags == status
        assert t.fault_code == fault
        assert t.current_angle_mdeg == angle

    def test_heartbeat_layout(self):
        uptime = 4000000000  # near uint32 max
        status = 0x0030
        fault  = 5
        raw = struct.pack("<IHBB", uptime, status, fault, 0)
        hb = proto.unpack_heartbeat(raw)
        assert hb.uptime_ms == uptime
        assert hb.status_flags == status
        assert hb.fault_code == fault

# Validating ESP32 C++ Protocol Packing

The C++ `protocol.cpp` uses `memcpy` to pack/unpack structs into byte buffers,
which is correct for little-endian architectures (ESP32 is LE).

## Quick validation approach

### 1. Add a serial-dump test in firmware

Add this to `setup()` temporarily:

```cpp
// --- Pack/unpack self-test ---
{
    // Test telemetry packing
    TelemetryPayload t;
    t.status_flags       = 0x001F;
    t.fault_code         = 3;
    t.reserved           = 0;
    t.current_angle_mdeg = -180000;

    uint8_t buf[8];
    packTelemetry(t, buf);
    Serial.print("[TEST] Telem bytes: ");
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();
    // Expected: 1F 00 03 00 20 21 FD FF
    //   status  = 0x001F → 1F 00
    //   fault   = 0x03
    //   reserved= 0x00
    //   angle   = -180000 = 0xFFFD2120 → 20 21 FD FF

    // Test command unpacking
    uint8_t cmd_data[8] = {0x10, 0x00, 0xC0, 0x1E, 0xFE, 0xFF, 0x88, 0x13};
    // opcode=0x10(SET_POS), flags=0, angle=-123456 (0xFFFE1EC0), param=5000 (0x1388)
    CommandFrame cmd;
    unpackCommand(cmd_data, cmd);
    Serial.printf("[TEST] CMD: op=%02X angle=%ld param=%u\n",
                  cmd.opcode, (long)cmd.angle_mdeg, cmd.param);
    // Expected: op=10 angle=-123456 param=5000

    // Test heartbeat packing
    HeartbeatPayload h;
    h.uptime_ms    = 4000000000UL;
    h.status_flags = 0x0030;
    h.fault_code   = 5;
    h.reserved     = 0;

    packHeartbeat(h, buf);
    Serial.print("[TEST] HB bytes: ");
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", buf[i]);
    Serial.println();
    // Expected: 00 28 6B EE 30 00 05 00
}
```

### 2. Compare with Python

Run on the Pi:

```python
import struct
from arm_pi.protocol import *

# Telemetry
data = pack_set_pos(-123456, 5000)
print("SET_POS bytes:", data.hex())
# Expected: 10 00 c0 1e fe ff 88 13

# Heartbeat
hb_bytes = struct.pack("<IHBB", 4000000000, 0x0030, 5, 0)
print("HB bytes:", hb_bytes.hex())
# Expected: 00 28 6b ee 30 00 05 00
```

Both sides should produce identical byte sequences. If they match,
the protocol implementation is consistent.

### 3. Over-the-wire test

1. Flash the ESP32, ensure CAN is up on both sides.
2. Send a known command from the Pi:
   ```
   python -m arm_pi.cli enable --node 1
   python -m arm_pi.cli status --node 1
   ```
3. Check that the telemetry/heartbeat fields decode correctly on the Pi.
4. Check Serial output on the ESP32 to confirm the command decoded correctly.

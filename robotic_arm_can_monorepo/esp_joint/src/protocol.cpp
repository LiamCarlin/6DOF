/**
 * protocol.cpp — CAN frame pack / unpack implementations.
 */

#include "protocol.h"

// =====================================================================
// Command unpacking
// =====================================================================

bool unpackCommand(const uint8_t data[8], CommandFrame &out) {
    out.opcode = data[0];
    out.flags  = data[1];

    // bytes 2-5: int32 little-endian
    memcpy(&out.angle_mdeg, &data[2], sizeof(int32_t));

    // bytes 6-7: uint16 little-endian
    memcpy(&out.param, &data[6], sizeof(uint16_t));

    return true;
}

// =====================================================================
// Telemetry packing
// =====================================================================

void packTelemetry(const TelemetryPayload &t, uint8_t out[8]) {
    // bytes 0-1: status_flags (uint16 LE)
    memcpy(&out[0], &t.status_flags, sizeof(uint16_t));

    // byte 2: fault_code
    out[2] = t.fault_code;

    // byte 3: reserved
    out[3] = 0;

    // bytes 4-7: current_angle_mdeg (int32 LE)
    memcpy(&out[4], &t.current_angle_mdeg, sizeof(int32_t));
}

// =====================================================================
// Heartbeat packing
// =====================================================================

void packHeartbeat(const HeartbeatPayload &h, uint8_t out[8]) {
    // bytes 0-3: uptime_ms (uint32 LE)
    memcpy(&out[0], &h.uptime_ms, sizeof(uint32_t));

    // bytes 4-5: status_flags (uint16 LE)
    memcpy(&out[4], &h.status_flags, sizeof(uint16_t));

    // byte 6: fault_code
    out[6] = h.fault_code;

    // byte 7: reserved
    out[7] = 0;
}

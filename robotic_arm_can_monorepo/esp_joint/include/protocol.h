/**
 * protocol.h — CAN protocol opcodes, frame structures, pack/unpack helpers.
 *
 * All multi-byte integers are little-endian.
 * All CAN payloads are exactly 8 bytes.
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <cstdint>
#include <cstring>

// =====================================================================
// Opcodes (command frame byte 0)
// =====================================================================
enum Opcode : uint8_t {
    OP_KEEPALIVE = 0x00,
    OP_ENABLE    = 0x01,
    OP_DISABLE   = 0x02,
    OP_STOP      = 0x03,
    OP_SET_ZERO  = 0x04,
    OP_SET_POS   = 0x10,
};

// =====================================================================
// Status flag bits (uint16)
// =====================================================================
enum StatusBit : uint8_t {
    BIT_ENABLED          = 0,
    BIT_MOVING           = 1,
    BIT_ZEROED           = 2,
    BIT_AT_TARGET        = 3,
    BIT_ENCODER_OK       = 4,
    BIT_CAN_OK           = 5,
    BIT_WATCHDOG_TIMEOUT = 6,
    BIT_LIMIT_FAULT      = 7,
};

inline void setFlag(uint16_t &flags, StatusBit bit)   { flags |=  (1u << bit); }
inline void clearFlag(uint16_t &flags, StatusBit bit)  { flags &= ~(1u << bit); }
inline bool getFlag(uint16_t flags, StatusBit bit)     { return (flags >> bit) & 1u; }

// =====================================================================
// Fault codes
// =====================================================================
enum FaultCode : uint8_t {
    FAULT_OK                   = 0,
    FAULT_ENCODER_I2C_FAIL     = 1,
    FAULT_CAN_INIT_FAIL        = 2,
    FAULT_COMMAND_OUT_OF_RANGE = 3,
    FAULT_WATCHDOG_STOP        = 4,
    FAULT_INTERNAL_ERROR       = 5,
};

// =====================================================================
// Parsed command
// =====================================================================
struct CommandFrame {
    uint8_t  opcode;
    uint8_t  flags;
    int32_t  angle_mdeg;
    uint16_t param;
};

/**
 * Unpack an 8-byte command payload into CommandFrame.
 * Returns true on success.
 */
bool unpackCommand(const uint8_t data[8], CommandFrame &out);

// =====================================================================
// Telemetry frame (node → base), 8 bytes
// =====================================================================
struct TelemetryPayload {
    uint16_t status_flags;
    uint8_t  fault_code;
    uint8_t  reserved;       // must be 0
    int32_t  current_angle_mdeg;
};

/**
 * Pack telemetry into an 8-byte buffer.
 */
void packTelemetry(const TelemetryPayload &t, uint8_t out[8]);

// =====================================================================
// Heartbeat frame (node → base), 8 bytes
// =====================================================================
struct HeartbeatPayload {
    uint32_t uptime_ms;
    uint16_t status_flags;
    uint8_t  fault_code;
    uint8_t  reserved;       // must be 0
};

/**
 * Pack heartbeat into an 8-byte buffer.
 */
void packHeartbeat(const HeartbeatPayload &h, uint8_t out[8]);

#endif // PROTOCOL_H

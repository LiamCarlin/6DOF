/**
 * config.h — Hardware pin definitions and compile-time constants.
 *
 * All values here MUST match the wiring spec in shared/protocol.md.
 * NODE_ID and CAN_TERMINATION can be overridden via platformio.ini build_flags.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// =====================================================================
// Node identity (override in platformio.ini: -DNODE_ID=1)
// =====================================================================
#ifndef NODE_ID
#define NODE_ID 1
#endif

// =====================================================================
// CAN transceiver pins (ESP32 TWAI)
// =====================================================================
#define CAN_TX_PIN  GPIO_NUM_5
#define CAN_RX_PIN  GPIO_NUM_4

// CAN termination flag (1 = this node is at a bus end → 120 Ω enabled externally)
#ifndef CAN_TERMINATION
#define CAN_TERMINATION 1
#endif

// =====================================================================
// TMC2209 STEP / DIR / EN
// =====================================================================
#define PIN_STEP    25
#define PIN_DIR     26
#define PIN_ENABLE  27

// TMC2209 EN polarity: active-LOW means LOW = enabled, HIGH = disabled
#define EN_ACTIVE_LOW  true  // set false if your board uses active-high

// =====================================================================
// AS5600 encoder (I2C)
// =====================================================================
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define AS5600_I2C_ADDR   0x36
#define AS5600_RAW_ANGLE_REG 0x0C   // 2-byte big-endian raw angle register

// =====================================================================
// CAN bus settings (must match Pi)
// =====================================================================
static constexpr uint32_t CAN_BITRATE = 500000;

// CAN ID bases
static constexpr uint16_t CAN_CMD_BASE   = 0x100;
static constexpr uint16_t CAN_TELEM_BASE = 0x200;
static constexpr uint16_t CAN_HB_BASE    = 0x300;

// Derived IDs for this node
static constexpr uint16_t MY_CMD_ID   = CAN_CMD_BASE   + NODE_ID;
static constexpr uint16_t MY_TELEM_ID = CAN_TELEM_BASE + NODE_ID;
static constexpr uint16_t MY_HB_ID    = CAN_HB_BASE    + NODE_ID;

// =====================================================================
// Mechanical / encoder
// =====================================================================
static constexpr uint16_t ENCODER_COUNTS       = 4096;
static constexpr float    ENCODER_DEG_PER_COUNT = 360.0f / 4096.0f;
static constexpr uint16_t MOTOR_FULL_STEPS_REV  = 200;
static constexpr uint16_t MICROSTEPS            = 16;
static constexpr float    GEAR_RATIO            = 21.0f;  // output:input
static constexpr uint32_t MOTOR_USTEPS_REV      = MOTOR_FULL_STEPS_REV * MICROSTEPS;      // 3200
static constexpr float    OUTPUT_USTEPS_REV     = MOTOR_USTEPS_REV * GEAR_RATIO;           // 67200

// =====================================================================
// Control limits  (NO soft limits — unlimited multi-turn rotation)
// =====================================================================
static constexpr int32_t  POSITION_TOLERANCE_MDEG = 300;      // ±0.3°
static constexpr int32_t  MDEG_PER_DEG           = 1000;
static constexpr int32_t  FULL_TURN_MDEG         = 360000;    // one output revolution
// Wrap detection: if single-turn reading jumps more than this, count a wrap
static constexpr int32_t  WRAP_THRESHOLD_MDEG    = 180000;    // half turn

// =====================================================================
// Timing
// =====================================================================
static constexpr uint32_t WATCHDOG_TIMEOUT_MS   = 300;
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 20;   // 50 Hz
static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 100;  // 10 Hz

// =====================================================================
// Step generator limits
// =====================================================================
static constexpr uint32_t DEFAULT_SPEED_MDEG_S  = 30000;  // 30°/s default
static constexpr uint32_t MAX_STEP_HZ           = 8000;   // max pulse rate
static constexpr uint32_t MIN_STEP_HZ           = 100;    // avoid stalling

// =====================================================================
// Serial debug
// =====================================================================
#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG 1
#endif

#if SERIAL_DEBUG
  #define DBG(fmt, ...) Serial.printf(fmt "\n", ##__VA_ARGS__)
#else
  #define DBG(fmt, ...) ((void)0)
#endif

#endif // CONFIG_H

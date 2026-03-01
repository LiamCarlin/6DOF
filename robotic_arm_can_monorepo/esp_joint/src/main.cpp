/**
 * main.cpp — ESP32 joint controller entry point.
 *
 * - Initialises CAN (TWAI), encoder, step generator, controller.
 * - Receives command frames, dispatches opcodes.
 * - Sends telemetry at 50 Hz and heartbeat at 10 Hz.
 * - Implements watchdog: STOP if no command received for >300 ms while enabled.
 * - Runs control loop at ~1 kHz from loop().
 */

#include <Arduino.h>
#include "driver/twai.h"

#include "config.h"
#include "protocol.h"
#include "encoder_as5600.h"
#include "stepgen.h"
#include "controller.h"

// =====================================================================
// Module state
// =====================================================================

static uint16_t  gStatusFlags = 0;
static uint8_t   gFaultCode   = FAULT_OK;
static bool      gEnabled     = false;    // driver enabled state
static uint32_t  gLastCmdTime = 0;        // millis() of last valid command
static bool      gCanOk       = false;

// =====================================================================
// CAN (TWAI) helpers
// =====================================================================

static bool canInit() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.tx_queue_len  = 10;
    g_config.rx_queue_len  = 10;

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // Acceptance filter: accept only our command ID
    // Standard frame: ID is in bits [31:21] of the filter register
    uint32_t id_shifted = ((uint32_t)MY_CMD_ID) << 21;
    twai_filter_config_t f_config;
    f_config.acceptance_code = id_shifted;
    f_config.acceptance_mask = ~(0x7FFu << 21);  // match all 11 bits
    f_config.single_filter   = true;

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        DBG("[CAN] driver install failed: 0x%x", err);
        return false;
    }

    err = twai_start();
    if (err != ESP_OK) {
        DBG("[CAN] start failed: 0x%x", err);
        return false;
    }

    DBG("[CAN] TWAI started, cmd ID=0x%03X", MY_CMD_ID);
    return true;
}

static bool canSend(uint32_t id, const uint8_t data[8]) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.data_length_code = 8;
    msg.extd             = 0;  // standard frame
    msg.rtr              = 0;
    memcpy(msg.data, data, 8);

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(5));
    return (err == ESP_OK);
}

// =====================================================================
// Build and send telemetry / heartbeat
// =====================================================================

static void buildStatusFlags() {
    gStatusFlags = 0;
    if (gEnabled)                setFlag(gStatusFlags, BIT_ENABLED);
    if (controllerIsMoving())    setFlag(gStatusFlags, BIT_MOVING);
    if (controllerIsZeroed())    setFlag(gStatusFlags, BIT_ZEROED);
    if (controllerAtTarget())    setFlag(gStatusFlags, BIT_AT_TARGET);
    if (controllerEncoderOk())   setFlag(gStatusFlags, BIT_ENCODER_OK);
    if (gCanOk)                  setFlag(gStatusFlags, BIT_CAN_OK);
}

static void sendTelemetry() {
    buildStatusFlags();
    TelemetryPayload t;
    t.status_flags       = gStatusFlags;
    t.fault_code         = gFaultCode;
    t.reserved           = 0;
    t.current_angle_mdeg = controllerCurrentMdeg();

    uint8_t buf[8];
    packTelemetry(t, buf);
    canSend(MY_TELEM_ID, buf);
}

static void sendHeartbeat() {
    buildStatusFlags();
    HeartbeatPayload h;
    h.uptime_ms    = millis();
    h.status_flags = gStatusFlags;
    h.fault_code   = gFaultCode;
    h.reserved     = 0;

    uint8_t buf[8];
    packHeartbeat(h, buf);
    canSend(MY_HB_ID, buf);
}

// =====================================================================
// Command dispatch
// =====================================================================

static void handleCommand(const CommandFrame &cmd) {
    gLastCmdTime = millis();

    // Clear watchdog fault on any valid command (if not a deeper fault)
    if (gFaultCode == FAULT_WATCHDOG_STOP) {
        gFaultCode = FAULT_OK;
        clearFlag(gStatusFlags, BIT_WATCHDOG_TIMEOUT);
    }

    switch (cmd.opcode) {

        case OP_KEEPALIVE:
            // Just refreshes watchdog timer (gLastCmdTime updated above)
            break;

        case OP_ENABLE:
            gEnabled = true;
            stepgenSetEnable(true);
            gFaultCode = FAULT_OK;
            clearFlag(gStatusFlags, BIT_WATCHDOG_TIMEOUT);
            clearFlag(gStatusFlags, BIT_LIMIT_FAULT);
            DBG("[CMD] ENABLE");
            break;

        case OP_DISABLE:
            gEnabled = false;
            stepgenSetEnable(false);
            controllerStop();
            DBG("[CMD] DISABLE");
            break;

        case OP_STOP:
            controllerStop();
            DBG("[CMD] STOP");
            break;

        case OP_SET_ZERO:
            controllerSetZero();
            DBG("[CMD] SET_ZERO");
            break;

        case OP_SET_POS: {
            if (!gEnabled) {
                DBG("[CMD] SET_POS ignored — not enabled");
                break;
            }
            bool ok = controllerSetTarget(cmd.angle_mdeg, cmd.param);
            if (!ok) {
                gFaultCode = FAULT_COMMAND_OUT_OF_RANGE;
                setFlag(gStatusFlags, BIT_LIMIT_FAULT);
                DBG("[CMD] SET_POS rejected — out of range (%ld)", (long)cmd.angle_mdeg);
            } else {
                // Clear limit fault on a good command
                clearFlag(gStatusFlags, BIT_LIMIT_FAULT);
                if (gFaultCode == FAULT_COMMAND_OUT_OF_RANGE) {
                    gFaultCode = FAULT_OK;
                }
                DBG("[CMD] SET_POS %ld mdeg, speed=%u", (long)cmd.angle_mdeg, cmd.param);
            }
            break;
        }

        default:
            DBG("[CMD] unknown opcode 0x%02X", cmd.opcode);
            break;
    }
}

// =====================================================================
// Process incoming CAN frames
// =====================================================================

static void processCanRx() {
    twai_message_t rxMsg;
    // Drain the RX queue (non-blocking)
    while (twai_receive(&rxMsg, 0) == ESP_OK) {
        if (rxMsg.extd || rxMsg.rtr) continue;           // ignore extended / RTR
        if (rxMsg.identifier != MY_CMD_ID) continue;     // shouldn't happen (filter)
        if (rxMsg.data_length_code != 8) continue;        // protocol requires 8 bytes

        CommandFrame cmd;
        if (unpackCommand(rxMsg.data, cmd)) {
            handleCommand(cmd);
        }
    }
}

// =====================================================================
// Watchdog
// =====================================================================

static void checkWatchdog() {
    if (!gEnabled) return;
    if (gFaultCode == FAULT_WATCHDOG_STOP) return;  // already tripped

    uint32_t elapsed = millis() - gLastCmdTime;
    if (elapsed > WATCHDOG_TIMEOUT_MS) {
        controllerStop();
        gFaultCode = FAULT_WATCHDOG_STOP;
        setFlag(gStatusFlags, BIT_WATCHDOG_TIMEOUT);
        DBG("[WDG] watchdog timeout (%lu ms)", elapsed);
    }
}

// =====================================================================
// Arduino setup() / loop()
// =====================================================================

void setup() {
#if SERIAL_DEBUG
    Serial.begin(115200);
    delay(200);
    DBG("=== Joint %d firmware starting ===", NODE_ID);
#endif

    // Init subsystems
    stepgenInit();

    bool encOk = encoderInit();
    if (!encOk) {
        gFaultCode = FAULT_ENCODER_I2C_FAIL;
    }

    controllerInit();

    gCanOk = canInit();
    if (!gCanOk) {
        gFaultCode = FAULT_CAN_INIT_FAIL;
    }

    gLastCmdTime = millis();

    DBG("=== Init complete, entering loop ===");
}

static uint32_t sLastTelem = 0;
static uint32_t sLastHb    = 0;
static uint32_t sLastCtrl  = 0;

void loop() {
    uint32_t now = millis();

    // --- CAN receive ----------------------------------------------------
    processCanRx();

    // --- Control loop at ~1 kHz -----------------------------------------
    if (now - sLastCtrl >= 1) {
        sLastCtrl = now;
        controllerUpdate();
    }

    // --- Watchdog check --------------------------------------------------
    checkWatchdog();

    // --- Telemetry at 50 Hz ---------------------------------------------
    if (now - sLastTelem >= TELEMETRY_INTERVAL_MS) {
        sLastTelem = now;
        sendTelemetry();
    }

    // --- Heartbeat at 10 Hz ---------------------------------------------
    if (now - sLastHb >= HEARTBEAT_INTERVAL_MS) {
        sLastHb = now;
        sendHeartbeat();
    }
}

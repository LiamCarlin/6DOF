/**
 * controller.cpp — Output-angle position controller with multi-turn tracking.
 *
 * Strategy (P-to-rate with deceleration):
 *   1. Read AS5600 single-turn angle (0–359.999°).
 *   2. Detect wraparound (crossing 0°↔360°) to maintain a turn counter.
 *   3. Compute absolute multi-turn position = turns * 360° + single-turn.
 *   4. Apply zero offset → zero-referenced continuous position.
 *   5. Compute linear error (target − current) — NO wrapping, direct difference.
 *   6. Map |error| to a step frequency (proportional with clamp).
 *   7. When |error| ≤ tolerance, stop and set AT_TARGET.
 *
 * No soft limits — the arm can rotate any number of turns in either direction.
 * The 21:1 cycloidal gearbox means 21 motor revs = 1 output rev.
 */

#include "controller.h"
#include "config.h"
#include "encoder_as5600.h"
#include "stepgen.h"
#include <Arduino.h>
#include <cmath>

// =====================================================================
// Internal state
// =====================================================================

static int32_t  sTargetMdeg     = 0;       // commanded output angle (mdeg, multi-turn)
static int32_t  sCurrentMdeg    = 0;       // multi-turn zero-referenced position (mdeg)
static int32_t  sZeroOffsetMdeg = 0;       // stored at SET_ZERO (absolute multi-turn)
static bool     sMoving         = false;
static bool     sAtTarget       = false;
static bool     sEncoderOk      = false;
static bool     sZeroed         = false;
static uint16_t sSpeedLimit     = 0;       // 0 = use default

// Multi-turn tracking
static int32_t  sTurnCount      = 0;       // number of full output turns
static int32_t  sPrevSingleMdeg = 0;       // previous single-turn reading (0–359999)
static bool     sFirstReading   = true;    // first reading after init/zero

// =====================================================================
// Helpers
// =====================================================================

/**
 * Map an absolute error (mdeg) to a step frequency (Hz).
 *
 * Linear ramp: freq = Kp * |error|, clamped to [MIN_STEP_HZ, maxHz].
 * maxHz is derived from the speed limit.
 */
static uint32_t errorToStepHz(int32_t absErrorMdeg, uint16_t speedLimitMdeg) {
    if (speedLimitMdeg == 0) speedLimitMdeg = DEFAULT_SPEED_MDEG_S;

    // Convert speed limit (mdeg/s at output) → step Hz at motor
    // step_hz = speedLimit * gear_ratio * motor_usteps_rev / (1000 * 360)
    float maxHz_f = (float)speedLimitMdeg * GEAR_RATIO * MOTOR_USTEPS_REV / 360000.0f;
    uint32_t maxHz = (uint32_t)maxHz_f;
    if (maxHz > MAX_STEP_HZ) maxHz = MAX_STEP_HZ;
    if (maxHz < MIN_STEP_HZ) maxHz = MIN_STEP_HZ;

    // Proportional gain: full speed when |error| >= 30° (30000 mdeg)
    // scale linearly below that
    const int32_t FULL_SPEED_ERROR = 30000;  // mdeg

    uint32_t hz;
    if (absErrorMdeg >= FULL_SPEED_ERROR) {
        hz = maxHz;
    } else {
        hz = (uint32_t)((float)maxHz * absErrorMdeg / FULL_SPEED_ERROR);
    }

    if (hz < MIN_STEP_HZ) hz = MIN_STEP_HZ;
    if (hz > maxHz) hz = maxHz;

    return hz;
}

// =====================================================================
// Public API
// =====================================================================

void controllerInit() {
    sTargetMdeg     = 0;
    sCurrentMdeg    = 0;
    sZeroOffsetMdeg = 0;
    sMoving         = false;
    sAtTarget       = false;
    sEncoderOk      = false;
    sZeroed         = false;
    sSpeedLimit     = 0;
    sTurnCount      = 0;
    sPrevSingleMdeg = 0;
    sFirstReading   = true;

    DBG("[CTRL] controller initialised (multi-turn, no limits)");
}

bool controllerSetTarget(int32_t target_mdeg, uint16_t speed_limit_mdeg_s) {
    // No soft limits — accept any int32 value
    sTargetMdeg = target_mdeg;
    sSpeedLimit = speed_limit_mdeg_s;
    sMoving     = true;
    sAtTarget   = false;
    DBG("[CTRL] target=%ld mdeg (%.2f turns), speed=%u",
        (long)target_mdeg, (float)target_mdeg / 360000.0f, speed_limit_mdeg_s);
    return true;
}

void controllerSetZero() {
    // Read current raw encoder angle
    int32_t rawMdeg = 0;
    if (encoderReadMdeg(rawMdeg)) {
        // Reset turn counter and store absolute position as zero offset
        sTurnCount      = 0;
        sPrevSingleMdeg = rawMdeg;
        sFirstReading   = false;
        sZeroOffsetMdeg = rawMdeg;  // single-turn reading at zero
        sZeroed         = true;
        sCurrentMdeg    = 0;
        sTargetMdeg     = 0;
        sAtTarget       = false;
        sMoving         = false;
        stepgenStop();
        DBG("[CTRL] zero set at raw=%ld mdeg, turn counter reset", (long)rawMdeg);
    }
}

void controllerStop() {
    sMoving = false;
    sAtTarget = false;
    stepgenStop();
    DBG("[CTRL] STOP");
}

void controllerUpdate() {
    // --- 1. Read single-turn encoder (0–359999 mdeg) --------------------
    int32_t singleMdeg = 0;
    sEncoderOk = encoderReadMdeg(singleMdeg);
    if (!sEncoderOk) {
        // If encoder fails, stop for safety
        stepgenStop();
        sMoving = false;
        return;
    }

    // --- 2. Multi-turn wraparound detection -----------------------------
    if (sFirstReading) {
        sPrevSingleMdeg = singleMdeg;
        sFirstReading = false;
    } else {
        int32_t delta = singleMdeg - sPrevSingleMdeg;

        // If the single-turn reading jumped by more than half a turn,
        // it wrapped around
        if (delta > WRAP_THRESHOLD_MDEG) {
            // Went from high to low → wrapped backwards (e.g. 350° → 10°)
            // Actually this means single went UP by a lot, which means
            // it wrapped from low to high... let's think carefully:
            // prev=10000 (10°), new=350000 (350°) → delta=+340000 → wrapped backwards
            sTurnCount--;
        } else if (delta < -WRAP_THRESHOLD_MDEG) {
            // Went from high to low → wrapped forwards (e.g. 350° → 10°)
            // prev=350000, new=10000 → delta=-340000 → wrapped forwards
            sTurnCount++;
        }
        sPrevSingleMdeg = singleMdeg;
    }

    // --- 3. Compute absolute multi-turn position ------------------------
    int32_t absoluteMdeg = sTurnCount * FULL_TURN_MDEG + singleMdeg;

    // --- 4. Apply zero offset → zero-referenced position ----------------
    sCurrentMdeg = absoluteMdeg - sZeroOffsetMdeg;

    // --- 5. If not moving, nothing to do --------------------------------
    if (!sMoving) {
        return;
    }

    // --- 6. Compute LINEAR error (no wrapping — multi-turn) -------------
    int32_t error = sTargetMdeg - sCurrentMdeg;
    int32_t absError = abs(error);

    // --- 7. Check tolerance ---------------------------------------------
    if (absError <= POSITION_TOLERANCE_MDEG) {
        stepgenStop();
        sMoving   = false;
        sAtTarget = true;
        return;
    }

    // --- 8. Determine direction and speed --------------------------------
    bool forward = (error > 0);
    stepgenSetDirection(forward);

    uint32_t hz = errorToStepHz(absError, sSpeedLimit);
    stepgenSetFrequency(hz);
}

// =====================================================================
// State accessors
// =====================================================================

int32_t  controllerCurrentMdeg()  { return sCurrentMdeg; }
int32_t  controllerTargetMdeg()   { return sTargetMdeg; }
bool     controllerIsMoving()     { return sMoving; }
bool     controllerAtTarget()     { return sAtTarget; }
bool     controllerEncoderOk()    { return sEncoderOk; }
bool     controllerIsZeroed()     { return sZeroed; }
uint16_t controllerSpeedLimit()   { return sSpeedLimit; }

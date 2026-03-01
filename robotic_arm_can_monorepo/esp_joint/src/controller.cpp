/**
 * controller.cpp — Output-angle position controller.
 *
 * Strategy (P-to-rate with deceleration):
 *   1. Read AS5600 output angle.
 *   2. Apply zero offset → zero-referenced output angle.
 *   3. Compute wrap-aware signed error  (target − current)  in mdeg.
 *   4. Map |error| to a step frequency (proportional with clamp).
 *   5. When |error| ≤ tolerance, stop and set AT_TARGET.
 *
 * The encoder is on the OUTPUT shaft, so the control loop works directly
 * in output-angle space — no gear conversion needed in the loop itself.
 * The step generator drives the motor, and the gear converts motor rotation
 * to output rotation; the encoder closes the loop.
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

static int32_t  sTargetMdeg     = 0;       // commanded output angle (mdeg)
static int32_t  sCurrentMdeg    = 0;       // latest encoder reading (zero-ref'd, mdeg)
static int32_t  sZeroOffsetMdeg = 0;       // stored at SET_ZERO
static bool     sMoving         = false;
static bool     sAtTarget       = false;
static bool     sEncoderOk      = false;
static bool     sZeroed         = false;
static uint16_t sSpeedLimit     = 0;       // 0 = use default

// =====================================================================
// Helpers
// =====================================================================

/**
 * Shortest signed angle difference in millidegrees, wrapping at 360 000.
 * Result is in range (−180000, +180000].
 */
static int32_t shortestDiffMdeg(int32_t target, int32_t current) {
    int32_t diff = target - current;

    // Normalise into (−180000, +180000]
    while (diff >  180000) diff -= 360000;
    while (diff <= -180000) diff += 360000;

    return diff;
}

/**
 * Map an absolute error (mdeg) to a step frequency (Hz).
 *
 * Linear ramp: freq = Kp * |error|, clamped to [MIN_STEP_HZ, maxHz].
 * maxHz is derived from the speed limit.
 */
static uint32_t errorToStepHz(int32_t absErrorMdeg, uint16_t speedLimitMdeg) {
    if (speedLimitMdeg == 0) speedLimitMdeg = DEFAULT_SPEED_MDEG_S;

    // Convert speed limit (mdeg/s at output) → step Hz at motor
    // output_deg_per_s = speedLimit / 1000
    // motor_rev_per_s  = output_deg_per_s / 360 * gear_ratio
    // step_hz          = motor_rev_per_s * motor_usteps_rev
    //
    // step_hz = speedLimit * gear_ratio * motor_usteps_rev / (1000 * 360)
    // Simplify: speedLimit * 21 * 3200 / 360000
    //         = speedLimit * 67200 / 360000
    //         ≈ speedLimit * 0.18667

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

    DBG("[CTRL] controller initialised");
}

bool controllerSetTarget(int32_t target_mdeg, uint16_t speed_limit_mdeg_s) {
    // Validate soft limits
    if (target_mdeg < SOFT_LIMIT_MIN_MDEG || target_mdeg > SOFT_LIMIT_MAX_MDEG) {
        return false;
    }

    sTargetMdeg = target_mdeg;
    sSpeedLimit = speed_limit_mdeg_s;
    sMoving     = true;
    sAtTarget   = false;
    DBG("[CTRL] target=%ld mdeg, speed=%u", (long)target_mdeg, speed_limit_mdeg_s);
    return true;
}

void controllerSetZero() {
    // Read current raw encoder angle
    int32_t rawMdeg = 0;
    if (encoderReadMdeg(rawMdeg)) {
        sZeroOffsetMdeg = rawMdeg;
        sZeroed = true;
        sTargetMdeg = 0;
        sAtTarget   = false;
        sMoving     = false;
        stepgenStop();
        DBG("[CTRL] zero set at raw=%ld mdeg", (long)rawMdeg);
    }
}

void controllerStop() {
    sMoving = false;
    sAtTarget = false;
    stepgenStop();
    DBG("[CTRL] STOP");
}

void controllerUpdate() {
    // --- 1. Read encoder ------------------------------------------------
    int32_t rawMdeg = 0;
    sEncoderOk = encoderReadMdeg(rawMdeg);
    if (!sEncoderOk) {
        // If encoder fails, stop for safety
        stepgenStop();
        sMoving = false;
        return;
    }

    // --- 2. Apply zero offset → zero-referenced angle ------------------
    int32_t zeroed = rawMdeg - sZeroOffsetMdeg;
    // Normalise to (−180000, +180000]
    while (zeroed >  180000) zeroed -= 360000;
    while (zeroed <= -180000) zeroed += 360000;
    sCurrentMdeg = zeroed;

    // --- 3. If not moving, nothing to do --------------------------------
    if (!sMoving) {
        return;
    }

    // --- 4. Compute wrap-aware error ------------------------------------
    int32_t error = shortestDiffMdeg(sTargetMdeg, sCurrentMdeg);
    int32_t absError = abs(error);

    // --- 5. Check tolerance ---------------------------------------------
    if (absError <= POSITION_TOLERANCE_MDEG) {
        stepgenStop();
        sMoving   = false;
        sAtTarget = true;
        return;
    }

    // --- 6. Determine direction and speed --------------------------------
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

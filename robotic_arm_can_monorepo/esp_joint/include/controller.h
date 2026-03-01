/**
 * controller.h — Output-angle position controller with multi-turn tracking.
 *
 * Runs periodically from the main loop (or a FreeRTOS task).
 * Reads the encoder, tracks multi-turn position via wraparound detection,
 * computes linear error, and drives the step generator.
 * No soft limits — unlimited rotation in either direction.
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cstdint>

/**
 * Initialise controller state.  Call after encoderInit() and stepgenInit().
 */
void controllerInit();

/**
 * Set the target output angle (in millidegrees, zero-referenced).
 * Unlimited range — any int32 value is accepted (multi-turn).
 * @param target_mdeg  Desired output angle in mdeg (unlimited).
 * @param speed_limit_mdeg_s  Max speed in mdeg/s (0 = use default).
 * @return always true (no soft limits).
 */
bool controllerSetTarget(int32_t target_mdeg, uint16_t speed_limit_mdeg_s);

/**
 * Store current encoder reading as the zero reference.
 */
void controllerSetZero();

/**
 * Immediately stop motion.
 */
void controllerStop();

/**
 * One iteration of the control loop.  Call at ~1 kHz from loop() / task.
 * Reads encoder, computes error, adjusts step generator.
 */
void controllerUpdate();

// ---------------------------------------------------------------------------
// State accessors (for telemetry)
// ---------------------------------------------------------------------------

/** Current zero-referenced output angle in mdeg. */
int32_t controllerCurrentMdeg();

/** Target angle in mdeg. */
int32_t controllerTargetMdeg();

/** True if controller is actively stepping toward target. */
bool controllerIsMoving();

/** True if within tolerance of target. */
bool controllerAtTarget();

/** True if encoder reads OK this cycle. */
bool controllerEncoderOk();

/** True if the zero offset has been set at least once. */
bool controllerIsZeroed();

/** Speed limit currently in effect (mdeg/s). */
uint16_t controllerSpeedLimit();

#endif // CONTROLLER_H

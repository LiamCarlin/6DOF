/**
 * stepgen.h — Hardware-timer based step pulse generator.
 *
 * Uses ESP32 hardware timer 0 to produce a clean STEP signal at a
 * commanded frequency.  Direction and enable are set before starting.
 */

#ifndef STEPGEN_H
#define STEPGEN_H

#include <cstdint>

/**
 * Initialise STEP, DIR, EN pins and the hardware timer.
 */
void stepgenInit();

/**
 * Set the motor enable pin.
 * @param en  true = driver enabled (respects EN_ACTIVE_LOW).
 */
void stepgenSetEnable(bool en);

/**
 * Set stepping direction.
 * @param forward  true = positive (CW) direction.
 */
void stepgenSetDirection(bool forward);

/**
 * Start or update step generation at the given frequency.
 * @param hz  Step pulse frequency in Hz (clamped to MIN/MAX).
 *            Pass 0 to stop.
 */
void stepgenSetFrequency(uint32_t hz);

/**
 * Immediately stop pulse generation (motor coasts to stop).
 */
void stepgenStop();

/**
 * Return the total number of steps generated since last reset.
 * (Useful for diagnostics; not used in the position loop since the
 *  encoder provides ground truth.)
 */
uint32_t stepgenTotalSteps();

/**
 * Reset the step counter to zero.
 */
void stepgenResetCount();

#endif // STEPGEN_H

/**
 * stepgen.cpp — Hardware-timer step pulse generator for ESP32.
 *
 * Uses hw_timer 0 to toggle the STEP pin at a precise frequency.
 * The ISR toggles the pin: one alarm period = half a step cycle,
 * so the timer fires at 2× the desired step frequency.
 */

#include "stepgen.h"
#include "config.h"
#include <Arduino.h>

// =====================================================================
// Module state
// =====================================================================
static hw_timer_t *sTimer = nullptr;
static volatile uint32_t sStepCount = 0;
static volatile bool     sRunning   = false;
static volatile bool     sPinState  = false;

// =====================================================================
// Timer ISR
// =====================================================================

static void IRAM_ATTR onTimerISR() {
    if (!sRunning) return;

    sPinState = !sPinState;
    digitalWrite(PIN_STEP, sPinState ? HIGH : LOW);

    // Count only on rising edge (one full step = one rising + one falling)
    if (sPinState) {
        sStepCount++;
    }
}

// =====================================================================
// Public API
// =====================================================================

void stepgenInit() {
    pinMode(PIN_STEP,   OUTPUT);
    pinMode(PIN_DIR,    OUTPUT);
    pinMode(PIN_ENABLE, OUTPUT);

    digitalWrite(PIN_STEP, LOW);
    digitalWrite(PIN_DIR,  LOW);

    // Start with driver disabled
    stepgenSetEnable(false);

    // Configure hw_timer 0, prescaler = 80 → 1 µs tick (80 MHz / 80)
    sTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(sTimer, &onTimerISR, true);
    // Don't start the alarm yet
    timerAlarmWrite(sTimer, 1000000, true);  // placeholder, 1 s
    timerAlarmDisable(sTimer);

    DBG("[STEP] stepgen initialised");
}

void stepgenSetEnable(bool en) {
    if (EN_ACTIVE_LOW) {
        digitalWrite(PIN_ENABLE, en ? LOW : HIGH);
    } else {
        digitalWrite(PIN_ENABLE, en ? HIGH : LOW);
    }
}

void stepgenSetDirection(bool forward) {
#if INVERT_DIR
    digitalWrite(PIN_DIR, forward ? LOW : HIGH);
#else
    digitalWrite(PIN_DIR, forward ? HIGH : LOW);
#endif
}

void stepgenSetFrequency(uint32_t hz) {
    if (hz == 0) {
        stepgenStop();
        return;
    }

    // Clamp
    if (hz < MIN_STEP_HZ) hz = MIN_STEP_HZ;
    if (hz > MAX_STEP_HZ) hz = MAX_STEP_HZ;

    // Timer fires at 2× step freq (rising + falling edges)
    uint32_t half_period_us = 500000UL / hz;   // 1e6 / (2 * hz)
    if (half_period_us < 10) half_period_us = 10;  // safety floor

    timerAlarmWrite(sTimer, half_period_us, true);

    if (!sRunning) {
        sRunning = true;
        timerAlarmEnable(sTimer);
    }
}

void stepgenStop() {
    sRunning = false;
    timerAlarmDisable(sTimer);
    digitalWrite(PIN_STEP, LOW);
    sPinState = false;
}

uint32_t stepgenTotalSteps() {
    return sStepCount;
}

void stepgenResetCount() {
    sStepCount = 0;
}

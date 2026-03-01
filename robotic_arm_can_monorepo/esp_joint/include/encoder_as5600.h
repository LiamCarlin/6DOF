/**
 * encoder_as5600.h — AS5600 absolute magnetic encoder over I2C.
 *
 * Reads raw 12-bit angle (0–4095) from the output shaft.
 */

#ifndef ENCODER_AS5600_H
#define ENCODER_AS5600_H

#include <cstdint>

/**
 * Initialise I2C and verify the AS5600 is present.
 * @return true if encoder responds on the bus.
 */
bool encoderInit();

/**
 * Read the raw 12-bit angle from the AS5600.
 * @param[out] raw  Raw angle 0–4095.
 * @return true on successful I2C read.
 */
bool encoderReadRaw(uint16_t &raw);

/**
 * Read the raw angle and convert to millidegrees (0–359999).
 * @param[out] mdeg  Angle in millidegrees.
 * @return true on success.
 */
bool encoderReadMdeg(int32_t &mdeg);

/**
 * Convert raw 12-bit count to millidegrees.
 */
inline int32_t rawToMdeg(uint16_t raw) {
    // raw * 360000 / 4096  ≈  raw * 87.890625
    return (int32_t)((uint32_t)raw * 360000UL / 4096UL);
}

#endif // ENCODER_AS5600_H

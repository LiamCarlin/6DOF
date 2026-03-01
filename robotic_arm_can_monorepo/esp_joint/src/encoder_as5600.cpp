/**
 * encoder_as5600.cpp — AS5600 I2C absolute encoder driver.
 */

#include "encoder_as5600.h"
#include "config.h"
#include <Wire.h>

// =====================================================================
// Init
// =====================================================================

bool encoderInit() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);  // 400 kHz fast-mode

    // Probe the device
    Wire.beginTransmission(AS5600_I2C_ADDR);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        DBG("[ENC] AS5600 not found (I2C err %u)", err);
        return false;
    }

    DBG("[ENC] AS5600 detected");
    return true;
}

// =====================================================================
// Read raw 12-bit angle
// =====================================================================

bool encoderReadRaw(uint16_t &raw) {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(AS5600_RAW_ANGLE_REG);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((uint8_t)AS5600_I2C_ADDR, (uint8_t)2) != 2) {
        return false;
    }

    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    raw = ((uint16_t)(hi & 0x0F) << 8) | lo;   // 12-bit result
    return true;
}

// =====================================================================
// Read in millidegrees
// =====================================================================

bool encoderReadMdeg(int32_t &mdeg) {
    uint16_t raw;
    if (!encoderReadRaw(raw)) {
        return false;
    }
    mdeg = rawToMdeg(raw);
    return true;
}

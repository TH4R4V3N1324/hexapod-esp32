#include "CurrentSensor.h"

// Initialize the sensor
void CurrentSensor::init() {
    // Configure INA260: 16V, 400mA, continuous mode
    uint16_t config = (0b011 << 9) | (0b100 << 6) | (0b100 << 3) | 0b111;
    uint8_t buf[3];
    buf[0] = INA260_REG_CONFIG;
    buf[1] = (config >> 8) & 0xFF;
    buf[2] = config & 0xFF;
    i2cManager.write(INA260_ADDRESS, buf, 3);
}

// Read current in mA
float CurrentSensor::readCurrent() {
    uint8_t data[2];
    if (i2cManager.readRegister(INA260_ADDRESS, INA260_REG_CURRENT, data, 2)) {
        uint16_t raw = (data[0] << 8) | data[1];
        return raw * 1.25f; // mA
    } else {
        return 0.0f;
    }
}

// Read voltage in mV
float CurrentSensor::readVoltage() {
    uint8_t data[2];
    if (i2cManager.readRegister(INA260_ADDRESS, INA260_REG_BUS_VOLTAGE, data, 2)) {
        uint16_t raw = (data[0] << 8) | data[1];
        return raw * 1.25f; // mV
    } else {
        return 0.0f;
    }
}

// Read power in mW
float CurrentSensor::readPower() {
    uint8_t data[2];
    if (i2cManager.readRegister(INA260_ADDRESS, INA260_REG_POWER, data, 2)) {
        uint16_t raw = (data[0] << 8) | data[1];
        return raw * 10.0f; // mW
    } else {
        return 0.0f;
    }
}

// Set trigger mode
void CurrentSensor::setTriggerMode(TriggerMode mode) {
    uint8_t buf[2];
    buf[0] = (mode >> 8) & 0xFF;
    buf[1] = mode & 0xFF;
    i2cManager.writeRegister(INA260_ADDRESS, INA260_REG_MASK_ENABLE, buf, 2);
}

// Set trigger threshold
void CurrentSensor::setTriggerThreshold(float threshold) {
    uint16_t register_value = (uint16_t)(threshold / 1.25f);
    uint8_t buf[2];
    buf[0] = (register_value >> 8) & 0xFF;
    buf[1] = register_value & 0xFF;
    i2cManager.writeRegister(INA260_ADDRESS, INA260_REG_ALERT_LIMIT, buf, 2);
}

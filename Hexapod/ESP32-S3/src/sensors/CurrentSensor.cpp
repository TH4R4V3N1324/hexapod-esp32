#include "CurrentSensor.h"

// Read INA260 register
uint16_t CurrentSensor::readRegister(uint8_t reg) {
    uint8_t data[2];
    i2cManager.write(INA260_ADDRESS, &reg, 1); // Set register pointer
    i2cManager.read(INA260_ADDRESS, data, 2);  // Read 2 bytes
    return (data[0] << 8) | data[1];        // MSB first
}

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
    uint16_t raw = readRegister(INA260_REG_CURRENT);
    return raw * 1.25f; // mA
}

// Read voltage in mV
float CurrentSensor::readVoltage() {
    uint16_t raw = readRegister(INA260_REG_BUS_VOLTAGE);
    return raw * 1.25f; // mV
}

// Read power in mW
float CurrentSensor::readPower() {
    uint16_t raw = readRegister(INA260_REG_POWER);
    return raw * 10.0f; // mW
}

#include "IMUSensor.h"

// Write a register
void IMUSensor::BMI330_writeRegister(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2cManager.write(BMI330_ADDR, buf, 2);
}

// Read a register
void IMUSensor::BMI330_readRegister(uint8_t reg, uint8_t *data, uint8_t len) {
    i2cManager.write(BMI330_ADDR, &reg, 1); // Set register pointer
    i2cManager.read(BMI330_ADDR, data, len); // Read data
}

// Initialize the sensor
void IMUSensor::init() {
    uint8_t id;
    BMI330_readRegister(BMI330_CHIP_ID, &id, 1);
    if (id != 0x42) {return;}

    // Reset the sensor
    BMI330_writeRegister(BMI330_CMD, 0xB6);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Configure power settings
    BMI330_writeRegister(BMI330_PWR_CONF, 0x00);
    BMI330_writeRegister(BMI330_PWR_CTRL, 0x0E);

    // Configure accelerometer and gyroscope
    BMI330_writeRegister(BMI330_ACC_CONF, 0x24);
    BMI330_writeRegister(BMI330_GYR_CONF, 0x24);
}

// Read sensor data
void IMUSensor::BMI330_readData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[12];
    BMI330_readRegister(BMI330_DATA_ACC, data, 12);

    *ax = (int16_t)((data[1] << 8) | data[0]);
    *ay = (int16_t)((data[3] << 8) | data[2]);
    *az = (int16_t)((data[5] << 8) | data[4]);
    *gx = (int16_t)((data[7] << 8) | data[6]);
    *gy = (int16_t)((data[9] << 8) | data[8]);
    *gz = (int16_t)((data[11] << 8) | data[10]);
}
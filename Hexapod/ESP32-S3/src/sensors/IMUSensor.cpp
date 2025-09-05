#include "IMUSensor.h"

// Initialize the sensor
void IMUSensor::init() {
    uint8_t id;
    i2cManager.readRegister(BMI330_ADDR, BMI330_CHIP_ID, &id, 1);
    if (id != 0x42) {return;}

    // Reset the sensor
    uint8_t reset = 0xB6;
    i2cManager.writeRegister(BMI330_ADDR, BMI330_CMD, &reset, 1);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    // Configure power settings
    uint8_t pwr_conf = 0x00;
    uint8_t pwr_ctrl = 0x0E;
    i2cManager.writeRegister(BMI330_ADDR, BMI330_PWR_CONF, &pwr_conf, 1);
    i2cManager.writeRegister(BMI330_ADDR, BMI330_PWR_CTRL, &pwr_ctrl, 1);

    // Configure accelerometer and gyroscope
    uint8_t acc_conf = 0x24;
    uint8_t gyr_conf = 0x24;
    i2cManager.writeRegister(BMI330_ADDR, BMI330_ACC_CONF, &acc_conf, 1);
    i2cManager.writeRegister(BMI330_ADDR, BMI330_GYR_CONF, &gyr_conf, 1);
}

// Read sensor data
void IMUSensor::BMI330_readData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    uint8_t data[12];
    if (i2cManager.readRegister(BMI330_ADDR, BMI330_DATA_ACC, data, 12)) {
        *ax = (int16_t)((data[1] << 8) | data[0]);
        *ay = (int16_t)((data[3] << 8) | data[2]);
        *az = (int16_t)((data[5] << 8) | data[4]);
        *gx = (int16_t)((data[7] << 8) | data[6]);
        *gy = (int16_t)((data[9] << 8) | data[8]);
        *gz = (int16_t)((data[11] << 8) | data[10]);
    } else {
        // Handle error: set outputs to zero or a known error value
        *ax = *ay = *az = *gx = *gy = *gz = 0;
    }
}

// Update pitch and roll using a complementary filter
void IMUSensor::updateOrientation() {
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    BMI330_readData(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // convert raw to physical
    float accelScale = 8.0f / 32768.0f;    // g/LSB
    float gyroScale  = 1000.0f / 32768.0f; // dps/LSB
    float ax = ax_raw * accelScale;
    float ay = ay_raw * accelScale;
    float az = az_raw * accelScale;
    float gx = gx_raw * gyroScale * M_PI/180.0f; // rad/s
    float gy = gy_raw * gyroScale * M_PI/180.0f; // rad/s

    // Update pitch and roll
    pitch += gx * dt;
    roll  += gy * dt;

    // Compute pitch and roll from accelerometer data
    float pitchAcc = atan2(-ax, sqrt(ay*ay + az*az));
    float rollAcc  = atan2(ay, az);

    // Complementary filter
    const float alpha = 0.98f;
    pitch = alpha * pitch + (1.0f - alpha) * pitchAcc;
    roll  = alpha * roll  + (1.0f - alpha) * rollAcc;
}

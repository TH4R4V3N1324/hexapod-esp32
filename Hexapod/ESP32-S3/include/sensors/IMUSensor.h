#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "SensorBase.h"
#include "I2CManager.h"
#include <math.h>

#define BMI330_ADDR      0x68 << 1   // 7-bit addr shifted for HAL
#define BMI330_CHIP_ID   0x00
#define BMI330_CMD       0x7E
#define BMI330_ACC_CONF  0x20
#define BMI330_GYR_CONF  0x21
#define BMI330_PWR_CONF  0x1A
#define BMI330_PWR_CTRL  0x1B
#define BMI330_DATA_ACC  0x0C  // start of accel data
#define BMI330_DATA_GYR  0x12  // start of gyro data

class IMUSensor : public SensorBase {
private:
    void BMI330_writeRegister(uint8_t reg, uint8_t data);
    void BMI330_readRegister(uint8_t reg, uint8_t *data, uint8_t len);
    void BMI330_readData(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
    void updateOrientation();
    float pitch = 0.0f, roll = 0.0f;
    float dt = 0.01f; // 100 Hz loop
public:
    void init() override;
    float getPitch() {updateOrientation(); return pitch;}
    float getRoll() {updateOrientation(); return roll;}
};

#endif
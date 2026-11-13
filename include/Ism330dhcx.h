#ifndef _ISM330DHCX_H_
#define _ISM330DHCX_H_

#include <Wire.h>

#include <cstddef>
#include <cstdint>

/**
 * @brief One accelerometer and gyroscope reading, in the sensor's axes
*/
struct ImuReading {
    float accel[3];  // m/s²
    float gyro[3];   // rad/s
};

/**
 * @brief I2C driver for the ISM330DHCX IMU (accelerometer + gyroscope)
 * @note Also accepts the older ISM330DLC, which shares the register layout.
*/
class Ism330dhcx {
public:
    /**
     * @brief Check the sensor is there and start a software reset. Call configure() 10 ms later.
     * @param[in] bus The I2C bus, already started
     * @param[in] address The sensor's I2C address (0x6A, or 0x6B with the address jumper bridged)
     * @return true if the sensor was found; error() says why not otherwise
     * @note Split from configure() so nothing waits out the reset: the gait loop keeps running
    */
    bool startReset(TwoWire& bus, uint8_t address);
    /**
     * @brief Configure the ranges and output rate, once the reset has finished
     * @param[in] accelRangeG Accelerometer full scale: 2, 4, 8 or 16 g
     * @param[in] gyroRangeDps Gyroscope full scale: 125, 250, 500, 1000, 2000 or 4000 °/s
     * @param[in] rate How often the sensor will be read (Hz); the output rate is the next step up
     * @return true if configured; error() says why not otherwise
    */
    bool configure(int accelRangeG, int gyroRangeDps, float rate);
    bool read(ImuReading& reading);

    const char* chipName() const { return chip_name; }
    const char* error() const { return error_text; }
    float outputRate() const { return output_rate; }

private:
    TwoWire* bus = nullptr;
    uint8_t address = 0;
    const char* chip_name = "ISM330DHCX";
    bool is_dlc = false;  // the older ISM330DLC
    char error_text[80] = "";
    float output_rate = 0;
    float accel_scale = 0;
    float gyro_scale = 0;

    bool checkId();
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
};

#endif

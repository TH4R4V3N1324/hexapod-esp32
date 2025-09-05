#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "driver/i2c.h"
#include <cstring>

#define I2C_MASTER_FREQ_HZ 100000          // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS 1000         // I2C master timeout

class I2CManager {
private:
    i2c_port_t port = I2C_NUM_0;
public:
    void init(uint8_t SDA_pin, uint8_t SCL_pin, i2c_port_t i2c_port = I2C_NUM_0);
    void deinit();
    bool write(uint8_t addr, uint8_t *data, size_t len);
    bool read(uint8_t addr, uint8_t *data, size_t len);
    bool readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t* data, uint8_t len);
    bool writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t* data, uint8_t len);
    void scanBus();
};

extern I2CManager i2cManager; // Global instance

#endif
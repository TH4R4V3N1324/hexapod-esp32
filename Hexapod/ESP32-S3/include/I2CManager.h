#ifndef I2C_MANAGER_H
#define I2C_MANAGER_H

#include "driver/i2c.h"

#define I2C_MASTER_FREQ_HZ 100000          // I2C master clock frequency
#define I2C_MASTER_TIMEOUT_MS 1000         // I2C master timeout

class I2CManager {
public:
    void init(uint8_t SDA_pin, uint8_t SCL_pin);
    void deinit();
    void write(uint8_t addr, uint8_t *data, size_t len);
    void read(uint8_t addr, uint8_t *data, size_t len);
};

extern I2CManager i2cManager; // Global instance

#endif
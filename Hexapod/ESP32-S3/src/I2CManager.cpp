#include "I2CManager.h"

I2CManager i2cManager;

/*
@brief Initialize I2C with specified SDA and SCL pins
@param SDA_pin GPIO number for SDA
@param SCL_pin GPIO number for SCL
@param i2c_port I2C port number (default is I2C_NUM_0)
@return void
@note This function configures the I2C driver with standard settings
*/
void I2CManager::init(uint8_t SDA_pin, uint8_t SCL_pin, i2c_port_t i2c_port) {
    port = i2c_port;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = SCL_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(i2c_port, &conf);
    i2c_driver_install(i2c_port, conf.mode, 0, 0, 0);
}

// Deinitialize I2C
void I2CManager::deinit() {
    i2c_driver_delete(port);
}

/*
@brief Write data to an I2C device
@param addr I2C device address
@param data Pointer to the data buffer
@param len Length of the data buffer
@return true on success, false on failure
@note This function uses a timeout defined by I2C_MASTER_TIMEOUT_MS
*/
bool I2CManager::write(uint8_t addr, uint8_t *data, size_t len) {
    return i2c_master_write_to_device(port, addr, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
@brief Read data from an I2C device
@param addr I2C device address
@param data Pointer to the data buffer
@param len Length of the data buffer
@return true on success, false on failure
@note This function uses a timeout defined by I2C_MASTER_TIMEOUT_MS
*/
bool I2CManager::read(uint8_t addr, uint8_t *data, size_t len) {
    return i2c_master_read_from_device(port, addr, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
@brief Read a register from an I2C device
@param deviceAddr I2C device address
@param reg Register address to read from
@param data Pointer to the data buffer
@param len Length of the data buffer
@return true on success, false on failure
@note This function first writes the register address, then reads the data
*/
bool I2CManager::readRegister(uint8_t deviceAddr, uint8_t reg, uint8_t* data, uint8_t len) {
    if (!write(deviceAddr, &reg, 1)) return false; // Set register pointer
    return read(deviceAddr, data, len); // Read data
}

/*
@brief Write to a register of an I2C device
@param deviceAddr I2C device address
@param reg Register address to write to
@param data Pointer to the data buffer
@param len Length of the data buffer
@return true on success, false on failure
@note This function prepends the register address to the data buffer before writing
*/
bool I2CManager::writeRegister(uint8_t deviceAddr, uint8_t reg, uint8_t* data, uint8_t len) {
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return write(deviceAddr, buf, len + 1);
}

// Scan the I2C bus for devices and print their addresses
void I2CManager::scanBus() {
    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        uint8_t data = 0;
        if (i2c_master_write_to_device(port, addr, &data, 0, 100 / portTICK_PERIOD_MS) == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
        }
    }
    printf("I2C scan complete.\n");
}

#include "I2CManager.h"

void I2CManager::init(uint8_t SDA_pin, uint8_t SCL_pin ) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = SCL_pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

void I2CManager::deinit() {
    i2c_driver_delete(I2C_NUM_0);
}

void I2CManager::write(uint8_t addr, uint8_t *data, size_t len) {
    i2c_master_write_to_device(I2C_NUM_0, addr, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

void I2CManager::read(uint8_t addr, uint8_t *data, size_t len) {
    i2c_master_read_from_device(I2C_NUM_0, addr, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

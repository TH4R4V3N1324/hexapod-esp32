#include "Ism330dhcx.h"

#include <Arduino.h>

#include <cstdio>

namespace {

// ISM330DHCX registers (the older ISM330DLC shares this layout)
constexpr uint8_t WHO_AM_I = 0x0F;
constexpr uint8_t CTRL1_XL = 0x10;
constexpr uint8_t CTRL2_G = 0x11;
constexpr uint8_t CTRL3_C = 0x12;
constexpr uint8_t OUTX_L_G = 0x22;  // gyro x, y, z then accel x, y, z, 12 bytes

constexpr uint8_t ISM330DHCX_ID = 0x6B;
constexpr uint8_t ISM330DLC_ID = 0x6A;

constexpr uint8_t CTRL3_SW_RESET = 0x01;
constexpr uint8_t CTRL3_BDU_IF_INC = 0x44;  // block data update, auto-increment register address
constexpr float GRAVITY = 9.80665f;
constexpr float PI_F = 3.14159265358979f;

}  // namespace

bool Ism330dhcx::writeRegister(uint8_t reg, uint8_t value) {
    bus->beginTransmission(address);
    bus->write(reg);
    bus->write(value);
    return bus->endTransmission() == 0;
}

bool Ism330dhcx::readRegisters(uint8_t reg, uint8_t* data, size_t length) {
    bus->beginTransmission(address);
    bus->write(reg);
    if (bus->endTransmission(false) != 0) {
        return false;
    }
    if (bus->requestFrom(address, static_cast<uint8_t>(length)) != length) {
        return false;
    }
    for (size_t i = 0; i < length; ++i) {
        data[i] = static_cast<uint8_t>(bus->read());
    }
    return true;
}

bool Ism330dhcx::checkId() {
    uint8_t id = 0;
    if (!readRegisters(WHO_AM_I, &id, 1) || (id != ISM330DHCX_ID && id != ISM330DLC_ID)) {
        snprintf(error_text, sizeof(error_text), "ISM330DHCX not found at 0x%02X (WHO_AM_I 0x%02X)", address, id);
        return false;
    }
    is_dlc = id == ISM330DLC_ID;
    chip_name = is_dlc ? "ISM330DLC" : "ISM330DHCX";
    return true;
}

bool Ism330dhcx::startReset(TwoWire& i2c, uint8_t i2cAddress) {
    bus = &i2c;
    address = i2cAddress;
    // Checked first, so a missing sensor fails straight away rather than after a reset wait
    if (!checkId()) {
        return false;
    }
    if (!writeRegister(CTRL3_C, CTRL3_SW_RESET)) {
        snprintf(error_text, sizeof(error_text), "Failed to reset %s", chip_name);
        return false;
    }
    return true;
}

bool Ism330dhcx::configure(int accelRangeG, int gyroRangeDps, float rate) {
    // FS_XL bits [3:2] of CTRL1_XL, mg/LSB from the datasheet
    uint8_t accel_range_code;
    float accel_mg_per_lsb;
    switch (accelRangeG) {
        case 2: accel_range_code = 0x00; accel_mg_per_lsb = 0.061f; break;
        case 4: accel_range_code = 0x08; accel_mg_per_lsb = 0.122f; break;
        case 8: accel_range_code = 0x0C; accel_mg_per_lsb = 0.244f; break;
        case 16: accel_range_code = 0x04; accel_mg_per_lsb = 0.488f; break;
        default:
            snprintf(error_text, sizeof(error_text), "accel range must be 2, 4, 8 or 16 g");
            return false;
    }
    // FS_G bits [3:0] of CTRL2_G, mdps/LSB from the datasheet
    uint8_t gyro_range_code;
    float gyro_mdps_per_lsb;
    switch (gyroRangeDps) {
        case 125: gyro_range_code = 0x02; gyro_mdps_per_lsb = 4.375f; break;
        case 250: gyro_range_code = 0x00; gyro_mdps_per_lsb = 8.75f; break;
        case 500: gyro_range_code = 0x04; gyro_mdps_per_lsb = 17.5f; break;
        case 1000: gyro_range_code = 0x08; gyro_mdps_per_lsb = 35.0f; break;
        case 2000: gyro_range_code = 0x0C; gyro_mdps_per_lsb = 70.0f; break;
        case 4000: gyro_range_code = 0x01; gyro_mdps_per_lsb = 140.0f; break;
        default:
            snprintf(error_text, sizeof(error_text), "gyro range must be 125, 250, 500, 1000, 2000 or 4000 dps");
            return false;
    }
    // ODR bits [7:4]: 1 = 12.5 Hz, doubling up to 8 = 1666 Hz
    uint8_t odr_code = 1;
    float odr_hz = 12.5f;
    while (odr_hz < rate && odr_code < 8) {
        odr_code++;
        odr_hz *= 2.0f;
    }

    if (!checkId()) {
        return false;
    }
    if (is_dlc && gyroRangeDps == 4000) {
        snprintf(error_text, sizeof(error_text), "The ISM330DLC has no 4000 dps range");
        return false;
    }

    bool ok = writeRegister(CTRL3_C, CTRL3_BDU_IF_INC);
    ok = ok && writeRegister(CTRL1_XL, static_cast<uint8_t>(odr_code << 4) | accel_range_code);
    ok = ok && writeRegister(CTRL2_G, static_cast<uint8_t>(odr_code << 4) | gyro_range_code);
    if (!ok) {
        snprintf(error_text, sizeof(error_text), "Failed to configure %s", chip_name);
        return false;
    }

    output_rate = odr_hz;
    accel_scale = accel_mg_per_lsb / 1000.0f * GRAVITY;
    gyro_scale = gyro_mdps_per_lsb / 1000.0f * PI_F / 180.0f;
    return true;
}

bool Ism330dhcx::read(ImuReading& reading) {
    uint8_t raw[12];
    if (!readRegisters(OUTX_L_G, raw, sizeof(raw))) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        reading.gyro[i] = static_cast<int16_t>(raw[2 * i] | (raw[2 * i + 1] << 8)) * gyro_scale;
        reading.accel[i] = static_cast<int16_t>(raw[6 + 2 * i] | (raw[7 + 2 * i] << 8)) * accel_scale;
    }
    return true;
}

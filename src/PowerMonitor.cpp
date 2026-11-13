// gait_core (Eigen) before any Arduino header: Arduino defines macros such as B1 that break Eigen
#include "RobotConfig.h"

#include "PowerMonitor.h"

#include <Arduino.h>

namespace {

// INA226 registers (16-bit, big-endian)
constexpr uint8_t INA226_CONFIG = 0x00;
constexpr uint8_t INA226_SHUNT_VOLTAGE = 0x01;
constexpr uint8_t INA226_BUS_VOLTAGE = 0x02;
constexpr uint8_t INA226_MANUFACTURER_ID = 0xFE;
constexpr uint16_t INA226_MANUFACTURER_ID_VALUE = 0x5449;  // "TI"

// 16-sample averaging, 1.1 ms conversions, continuous shunt and bus: a new value every ~35 ms
constexpr uint16_t INA226_CONFIG_VALUE = 0x4527;
constexpr float SHUNT_VOLTAGE_LSB = 2.5e-6f;  // V
constexpr float BUS_VOLTAGE_LSB = 1.25e-3f;   // V

constexpr uint64_t SEARCH_PERIOD_US = 5000000;

}  // namespace

bool PowerMonitor::readRegister(uint8_t address, uint8_t reg, uint16_t& value) {
    bus.beginTransmission(address);
    bus.write(reg);
    if (bus.endTransmission(false) != 0 || bus.requestFrom(address, static_cast<uint8_t>(2)) != 2) {
        return false;
    }
    const uint8_t high = static_cast<uint8_t>(bus.read());
    const uint8_t low = static_cast<uint8_t>(bus.read());
    value = static_cast<uint16_t>((high << 8) | low);
    return true;
}

bool PowerMonitor::writeRegister(uint8_t address, uint8_t reg, uint16_t value) {
    bus.beginTransmission(address);
    bus.write(reg);
    bus.write(static_cast<uint8_t>(value >> 8));
    bus.write(static_cast<uint8_t>(value & 0xFF));
    return bus.endTransmission() == 0;
}

bool PowerMonitor::initialise(uint8_t address) {
    uint16_t id = 0;
    return readRegister(address, INA226_MANUFACTURER_ID, id) && id == INA226_MANUFACTURER_ID_VALUE &&
           writeRegister(address, INA226_CONFIG, INA226_CONFIG_VALUE);
}

bool PowerMonitor::update(uint64_t nowUs) {
    if (nowUs < nextReadUs) {
        return false;
    }
    nextReadUs = nowUs + robot::POWER_READ_PERIOD_US;

    for (int hub = 0; hub < NUM_HUBS; ++hub) {
        readNow[hub] = false;
        const uint8_t address = robot::LEG_HUB_ADDRESSES[hub];
        if (!found[hub]) {
            // A missing chip doesn't acknowledge, so this fails at once
            if (nowUs < nextSearchUs[hub]) {
                continue;
            }
            nextSearchUs[hub] = nowUs + SEARCH_PERIOD_US;
            if (!initialise(address)) {
                continue;
            }
            found[hub] = true;
            Serial.printf("[power] %s leg hub INA226 found at 0x%02X\n", robot::LEG_HUB_NAMES[hub], address);
        }

        uint16_t shuntRaw = 0, busRaw = 0;
        if (!readRegister(address, INA226_SHUNT_VOLTAGE, shuntRaw) || !readRegister(address, INA226_BUS_VOLTAGE, busRaw)) {
            Serial.printf("[power] %s leg hub INA226 stopped answering\n", robot::LEG_HUB_NAMES[hub]);
            found[hub] = false;
            nextSearchUs[hub] = nowUs + SEARCH_PERIOD_US;
            continue;
        }
        volts[hub] = busRaw * BUS_VOLTAGE_LSB;
        amps[hub] = static_cast<int16_t>(shuntRaw) * SHUNT_VOLTAGE_LSB / robot::LEG_HUB_SHUNT_OHMS;
        readNow[hub] = true;
    }
    return true;
}

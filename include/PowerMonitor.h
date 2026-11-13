#ifndef _POWER_MONITOR_H_
#define _POWER_MONITOR_H_

#include <Wire.h>

#include <array>
#include <cstdint>

/**
 * @brief The two leg hubs' INA226 power monitors: battery voltage and current on each side
 * @note Each INA226 measures everything its hub draws (its three legs and its 5 V output). A hub
 *       that doesn't answer is looked for again every 5 s, so without leg hubs nothing is read.
*/
class PowerMonitor {
public:
    static constexpr int NUM_HUBS = 2;

    explicit PowerMonitor(TwoWire& bus) : bus(bus) {}

    /**
     * @brief Read the hubs if it's time
     * @param[in] nowUs The current time (us since boot)
     * @return true if it read them (check fresh() for which)
    */
    bool update(uint64_t nowUs);

    bool present(int hub) const { return found[hub]; }
    bool fresh(int hub) const { return readNow[hub]; }    // read in the last update()
    float voltage(int hub) const { return volts[hub]; }   // V
    float current(int hub) const { return amps[hub]; }    // A, positive while drawing

private:
    TwoWire& bus;
    std::array<bool, NUM_HUBS> found{};
    std::array<bool, NUM_HUBS> readNow{};
    std::array<float, NUM_HUBS> volts{};
    std::array<float, NUM_HUBS> amps{};
    std::array<uint64_t, NUM_HUBS> nextSearchUs{};
    uint64_t nextReadUs = 0;

    bool initialise(uint8_t address);
    bool readRegister(uint8_t address, uint8_t reg, uint16_t& value);
    bool writeRegister(uint8_t address, uint8_t reg, uint16_t value);
};

#endif

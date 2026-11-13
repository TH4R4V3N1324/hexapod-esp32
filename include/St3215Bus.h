#ifndef _ST3215_BUS_H_
#define _ST3215_BUS_H_

#include <Arduino.h>

#include <cstddef>
#include <cstdint>

// Feetech STS protocol instructions
constexpr uint8_t ST3215_INST_PING = 0x01;
constexpr uint8_t ST3215_INST_READ = 0x02;
constexpr uint8_t ST3215_INST_WRITE = 0x03;
constexpr uint8_t ST3215_INST_SYNC_READ = 0x82;
constexpr uint8_t ST3215_INST_SYNC_WRITE = 0x83;
constexpr uint8_t ST3215_BROADCAST_ID = 0xFE;

// ST3215 control table (multi-byte values are little-endian)
constexpr uint8_t ST3215_REG_ID = 5;                    // EEPROM, unlock with ST3215_REG_LOCK first
constexpr uint8_t ST3215_REG_MIN_ANGLE_LIMIT = 9;      // EEPROM, steps (0 and 4095: no limit)
constexpr uint8_t ST3215_REG_MAX_ANGLE_LIMIT = 11;     // EEPROM, steps
constexpr uint8_t ST3215_REG_POSITION_OFFSET = 31;     // EEPROM, steps, sign bit 11 (+-2047)
constexpr uint8_t ST3215_REG_TORQUE_ENABLE = 40;
constexpr uint8_t ST3215_TORQUE_SET_CENTRE = 128;      // written to TORQUE_ENABLE: the current position becomes 2048
constexpr uint8_t ST3215_REG_ACCELERATION = 41;
constexpr uint8_t ST3215_REG_LOCK = 55;                 // 0 = EEPROM writable, 1 = locked
constexpr uint8_t ST3215_REG_PRESENT_POSITION = 56;     // then speed, load, voltage, temperature

constexpr int ST3215_STEPS_PER_REV = 4096;
constexpr int ST3215_CENTER_POSITION = 2048;

/**
 * @brief Feedback read from one servo
*/
struct St3215State {
    int position = 0;         // steps, 0-4095
    int speed = 0;            // steps/s, signed
    int load = 0;             // 0.1% of max torque, signed
    uint8_t voltage = 0;      // 0.1 V
    uint8_t temperature = 0;  // degrees C
    uint8_t error = 0;        // status packet error bits (0 = no error)
};

/**
 * @brief Serial bus driver for ST3215 servos (Feetech STS protocol) on an ESP32 UART
 * @note Uses fixed buffers, so nothing
 *       is allocated while the gait runs. The bus is half-duplex: set echo if the adapter loops
 *       transmitted bytes back to RX.
*/
class St3215Bus {
public:
    static constexpr size_t MAX_SERVOS = 32;

    void begin(HardwareSerial& port, uint32_t baud, int rxPin, int txPin, bool echo);

    bool ping(uint8_t id);
    bool setTorqueEnabled(const uint8_t* ids, size_t count, bool enabled);
    /**
     * @brief Send goal positions (steps, 0-4095) to several servos in one sync write
    */
    bool writeGoalPositions(const uint8_t* ids, const int* positions, size_t count, uint8_t acceleration,
                            uint16_t speed);
    /**
     * @brief Read position, speed, load, voltage and temperature from several servos with one sync read
     * @param[out] states The state of each servo
     * @param[out] received Whether each servo replied
     * @return How many replied
    */
    size_t readStates(const uint8_t* ids, size_t count, St3215State* states, bool* received);
    bool readState(uint8_t id, St3215State& state);

    bool readRegisters(uint8_t id, uint8_t address, uint8_t length, uint8_t* data, uint8_t* error = nullptr);
    /**
     * @brief Write registers on one servo (acknowledged) or every servo (ST3215_BROADCAST_ID, not)
    */
    bool writeRegisters(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length);
    /**
     * @brief Write EEPROM registers, unlocking them around the write so they're kept through a power cycle
    */
    bool writeEeprom(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length);
    bool readWord(uint8_t id, uint8_t address, int& value, int signBit = -1);

    static int decodeSigned(uint16_t value, int signBit);
    static uint16_t encodeSigned(int value, int signBit);

private:
    HardwareSerial* serial = nullptr;
    bool echo = false;
    uint8_t packet[260];

    size_t buildPacket(uint8_t id, uint8_t instruction, const uint8_t* params, size_t paramCount);
    bool transmit(size_t length);
    /**
     * @brief Read the next status packet, from whichever servo sends it
     * @return false on a timeout or a bad checksum
    */
    bool readStatus(uint8_t& id, uint8_t* params, size_t maxParams, size_t& paramCount, uint8_t& error);
    bool readStatusFrom(uint8_t id, uint8_t* params, size_t length, uint8_t* error);
    bool readBytes(uint8_t* buffer, size_t length);
};

#endif

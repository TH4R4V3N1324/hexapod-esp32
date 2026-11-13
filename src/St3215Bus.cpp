#include "St3215Bus.h"

#include <cstring>

namespace {

// How long to wait for the next byte of a reply. A servo replies within ~0.1 ms at 1 Mbaud, so this
// only costs time when one is missing.
constexpr uint32_t BYTE_TIMEOUT_US = 3000;
constexpr uint8_t STATE_LENGTH = 8;  // position, speed, load (2 bytes each), voltage, temperature

/**
 * @brief The STS checksum: the inverted low byte of the sum of every byte after the 0xFF 0xFF header
*/
uint8_t checksum(const uint8_t* packet, size_t length) {
    uint32_t sum = 0;
    for (size_t i = 2; i < length; ++i) {
        sum += packet[i];
    }
    return static_cast<uint8_t>(~sum);
}

St3215State decodeState(const uint8_t* data, uint8_t error) {
    St3215State state;
    state.position = St3215Bus::decodeSigned(static_cast<uint16_t>(data[0] | (data[1] << 8)), 15);
    state.speed = St3215Bus::decodeSigned(static_cast<uint16_t>(data[2] | (data[3] << 8)), 15);
    state.load = St3215Bus::decodeSigned(static_cast<uint16_t>(data[4] | (data[5] << 8)), 10);
    state.voltage = data[6];
    state.temperature = data[7];
    state.error = error;
    return state;
}

}  // namespace

void St3215Bus::begin(HardwareSerial& port, uint32_t baud, int rxPin, int txPin, bool loopsBack) {
    serial = &port;
    echo = loopsBack;
    serial->setRxBufferSize(512);  // a full sync read of 18 servos is 252 bytes
    serial->begin(baud, SERIAL_8N1, rxPin, txPin);
}

bool St3215Bus::ping(uint8_t id) {
    if (!transmit(buildPacket(id, ST3215_INST_PING, nullptr, 0))) {
        return false;
    }
    return readStatusFrom(id, nullptr, 0, nullptr);
}

bool St3215Bus::setTorqueEnabled(const uint8_t* ids, size_t count, bool enabled) {
    if (count == 0 || count > MAX_SERVOS) {
        return false;
    }
    uint8_t params[2 + MAX_SERVOS * 2];
    size_t n = 0;
    params[n++] = ST3215_REG_TORQUE_ENABLE;
    params[n++] = 1;
    for (size_t i = 0; i < count; ++i) {
        params[n++] = ids[i];
        params[n++] = enabled ? 1 : 0;
    }
    // Sync writes are broadcast, so the servos don't reply
    return transmit(buildPacket(ST3215_BROADCAST_ID, ST3215_INST_SYNC_WRITE, params, n));
}

bool St3215Bus::writeGoalPositions(const uint8_t* ids, const int* positions, size_t count, uint8_t acceleration,
                                   uint16_t speed) {
    if (count == 0 || count > MAX_SERVOS) {
        return false;
    }
    constexpr uint8_t DATA_LENGTH = 7;
    uint8_t params[2 + MAX_SERVOS * (1 + DATA_LENGTH)];
    size_t n = 0;
    params[n++] = ST3215_REG_ACCELERATION;
    params[n++] = DATA_LENGTH;
    for (size_t i = 0; i < count; ++i) {
        const uint16_t position = static_cast<uint16_t>(positions[i]);
        // [acceleration][goal position][goal time = 0][goal speed]
        params[n++] = ids[i];
        params[n++] = acceleration;
        params[n++] = static_cast<uint8_t>(position & 0xFF);
        params[n++] = static_cast<uint8_t>(position >> 8);
        params[n++] = 0;
        params[n++] = 0;
        params[n++] = static_cast<uint8_t>(speed & 0xFF);
        params[n++] = static_cast<uint8_t>(speed >> 8);
    }
    return transmit(buildPacket(ST3215_BROADCAST_ID, ST3215_INST_SYNC_WRITE, params, n));
}

size_t St3215Bus::readStates(const uint8_t* ids, size_t count, St3215State* states, bool* received) {
    for (size_t i = 0; i < count; ++i) {
        received[i] = false;
    }
    if (count == 0 || count > MAX_SERVOS) {
        return 0;
    }
    uint8_t params[2 + MAX_SERVOS];
    params[0] = ST3215_REG_PRESENT_POSITION;
    params[1] = STATE_LENGTH;
    memcpy(params + 2, ids, count);
    if (!transmit(buildPacket(ST3215_BROADCAST_ID, ST3215_INST_SYNC_READ, params, 2 + count))) {
        return 0;
    }

    // Each servo replies in turn. Replies are matched by ID, so a missing one doesn't lose the rest.
    size_t replies = 0;
    for (size_t attempt = 0; attempt < count; ++attempt) {
        uint8_t id = 0, error = 0;
        uint8_t data[STATE_LENGTH];
        size_t length = 0;
        if (!readStatus(id, data, sizeof(data), length, error)) {
            break;  // timed out: nothing more is coming
        }
        if (length != STATE_LENGTH) {
            continue;
        }
        for (size_t i = 0; i < count; ++i) {
            if (ids[i] == id && !received[i]) {
                states[i] = decodeState(data, error);
                received[i] = true;
                ++replies;
                break;
            }
        }
    }
    return replies;
}

bool St3215Bus::readState(uint8_t id, St3215State& state) {
    uint8_t data[STATE_LENGTH];
    uint8_t error = 0;
    if (!readRegisters(id, ST3215_REG_PRESENT_POSITION, STATE_LENGTH, data, &error)) {
        return false;
    }
    state = decodeState(data, error);
    return true;
}

bool St3215Bus::readRegisters(uint8_t id, uint8_t address, uint8_t length, uint8_t* data, uint8_t* error) {
    const uint8_t params[2] = {address, length};
    if (!transmit(buildPacket(id, ST3215_INST_READ, params, 2))) {
        return false;
    }
    return readStatusFrom(id, data, length, error);
}

bool St3215Bus::writeRegisters(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length) {
    uint8_t params[1 + 32];
    if (length > 32) {
        return false;
    }
    params[0] = address;
    memcpy(params + 1, data, length);
    if (!transmit(buildPacket(id, ST3215_INST_WRITE, params, 1 + length))) {
        return false;
    }
    if (id == ST3215_BROADCAST_ID) {
        return true;
    }
    return readStatusFrom(id, nullptr, 0, nullptr);
}

bool St3215Bus::writeEeprom(uint8_t id, uint8_t address, const uint8_t* data, uint8_t length) {
    const uint8_t unlock = 0, lock = 1;
    if (!writeRegisters(id, ST3215_REG_LOCK, &unlock, 1)) {
        return false;
    }
    const bool written = writeRegisters(id, address, data, length);
    writeRegisters(id, ST3215_REG_LOCK, &lock, 1);
    return written;
}

bool St3215Bus::readWord(uint8_t id, uint8_t address, int& value, int signBit) {
    uint8_t data[2];
    if (!readRegisters(id, address, 2, data)) {
        return false;
    }
    const uint16_t raw = static_cast<uint16_t>(data[0] | (data[1] << 8));
    value = signBit < 0 ? raw : decodeSigned(raw, signBit);
    return true;
}

int St3215Bus::decodeSigned(uint16_t value, int signBit) {
    const uint16_t mask = static_cast<uint16_t>(1u << signBit);
    return (value & mask) ? -static_cast<int>(value & ~mask) : static_cast<int>(value);
}

uint16_t St3215Bus::encodeSigned(int value, int signBit) {
    return static_cast<uint16_t>(value < 0 ? (-value) | (1 << signBit) : value);
}

size_t St3215Bus::buildPacket(uint8_t id, uint8_t instruction, const uint8_t* params, size_t paramCount) {
    if (paramCount + 6 > sizeof(packet)) {
        return 0;
    }
    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = id;
    packet[3] = static_cast<uint8_t>(paramCount + 2);
    packet[4] = instruction;
    if (paramCount > 0) {
        memcpy(packet + 5, params, paramCount);
    }
    packet[5 + paramCount] = checksum(packet, 5 + paramCount);
    return 6 + paramCount;
}

/**
 * @note Stale input is discarded first, so a late reply to an earlier packet can't be taken for this one's
*/
bool St3215Bus::transmit(size_t length) {
    if (serial == nullptr || length == 0) {
        return false;
    }
    while (serial->available() > 0) {
        serial->read();
    }
    if (serial->write(packet, length) != length) {
        return false;
    }
    serial->flush();  // wait until it's on the wire
    if (echo) {
        uint8_t echoed[sizeof(packet)];
        return readBytes(echoed, length);
    }
    return true;
}

bool St3215Bus::readStatus(uint8_t& id, uint8_t* params, size_t maxParams, size_t& paramCount, uint8_t& error) {
    // Find the 0xFF 0xFF header
    uint8_t byte = 0;
    int headerCount = 0;
    while (headerCount < 2) {
        if (!readBytes(&byte, 1)) {
            return false;
        }
        headerCount = (byte == 0xFF) ? headerCount + 1 : 0;
    }
    // Skip any extra 0xFF bytes: the first other byte is the ID
    uint8_t replyId = 0xFF;
    while (replyId == 0xFF) {
        if (!readBytes(&replyId, 1)) {
            return false;
        }
    }

    uint8_t length = 0;
    if (!readBytes(&length, 1) || length < 2) {
        return false;
    }
    // [error][params...][checksum]
    uint8_t body[256];
    if (!readBytes(body, length)) {
        return false;
    }
    uint8_t sum = static_cast<uint8_t>(replyId + length);
    for (size_t i = 0; i + 1 < length; ++i) {
        sum = static_cast<uint8_t>(sum + body[i]);
    }
    if (static_cast<uint8_t>(~sum) != body[length - 1]) {
        return false;
    }

    id = replyId;
    error = body[0];
    paramCount = length - 2;
    if (params != nullptr) {
        memcpy(params, body + 1, paramCount < maxParams ? paramCount : maxParams);
    }
    return true;
}

bool St3215Bus::readStatusFrom(uint8_t id, uint8_t* params, size_t length, uint8_t* error) {
    uint8_t replyId = 0, replyError = 0;
    size_t count = 0;
    if (!readStatus(replyId, params, length, count, replyError) || replyId != id || count != length) {
        return false;
    }
    if (error != nullptr) {
        *error = replyError;
    }
    return true;
}

bool St3215Bus::readBytes(uint8_t* buffer, size_t length) {
    size_t received = 0;
    uint32_t lastByte = micros();
    while (received < length) {
        if (serial->available() > 0) {
            buffer[received++] = static_cast<uint8_t>(serial->read());
            lastByte = micros();
        } else if (micros() - lastByte > BYTE_TIMEOUT_US) {
            return false;
        }
    }
    return true;
}

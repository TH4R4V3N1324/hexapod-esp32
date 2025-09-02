
#include <cstdio>
#include "ConfigManager.h"

ConfigManager configManager;

// Definition of static member
int16_t ConfigManager::jointOffsets[NUM_LEGS][NUM_JOINTS] = {};

// Initialize EEPROM
void ConfigManager::initEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
}

// Save offsets
void ConfigManager::saveLegOffsets() {
    int addr = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            EEPROM.put(addr, jointOffsets[i][j]);
            addr += sizeof(int16_t);
        }
    }
    EEPROM.commit();
}

// Load offsets
void ConfigManager::loadLegOffsets() {
    int addr = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            EEPROM.get(addr, jointOffsets[i][j]);
            addr += sizeof(int16_t);
        }
    }
}

// Change offsets for leg
void ConfigManager::SetLegConfig(int legNum, int joint, int offset) {
    // Clamp offset to safe range
    if (offset > 60) offset = 60;
    if (offset < -60) offset = -60;
    jointOffsets[legNum][joint] = offset;
    saveLegOffsets();
    // Optional: print for debug
    printf("[ConfigManager] SetLegConfig: leg %d, joint %d, offset %d\n", legNum, joint, offset);
}

// Returns the offsets for a given leg
const int16_t* ConfigManager::getLegOffsets(int legNum) const {
    return jointOffsets[legNum];
}
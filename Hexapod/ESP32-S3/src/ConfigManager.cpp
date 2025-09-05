
#include <cstdio>
#include "ConfigManager.h"

ConfigManager configManager;

// Definition of static member
int16_t ConfigManager::jointOffsets[NUM_LEGS][NUM_JOINTS] = {};

/*
@brief Initialize the EEPROM for storing leg offsets
*/
void ConfigManager::initEEPROM() {
    EEPROM.begin(EEPROM_SIZE);
}

/*
@brief Save the current leg offsets to EEPROM
*/
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

/*
@brief Load the leg offsets from EEPROM
*/
void ConfigManager::loadLegOffsets() {
    int addr = 0;
    for (int i = 0; i < NUM_LEGS; i++) {
        for (int j = 0; j < NUM_JOINTS; j++) {
            EEPROM.get(addr, jointOffsets[i][j]);
            addr += sizeof(int16_t);
        }
    }
}

/*
@brief Set the offset for a specific leg and joint, then save to EEPROM
@param legNum The leg number (0-5)
@param joint The joint index (0-2)
@param offset The offset value to set (-60 to +60)
*/
void ConfigManager::SetLegConfig(int legNum, int joint, int offset) {
    // Clamp offset to safe range
    if (offset > 60) offset = 60;
    if (offset < -60) offset = -60;
    jointOffsets[legNum][joint] = offset;
    saveLegOffsets();
    // Optional: print for debug
    printf("[ConfigManager] SetLegConfig: leg %d, joint %d, offset %d\n", legNum, joint, offset);
}

/*
@brief Get the leg offsets for a specific leg
@param legNum The leg number (0-5)
@return A pointer to the array of joint offsets for the specified leg
*/
const int16_t* ConfigManager::getLegOffsets(int legNum) const {
    return jointOffsets[legNum];
}
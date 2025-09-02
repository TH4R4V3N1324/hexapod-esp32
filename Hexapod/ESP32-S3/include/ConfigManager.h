#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include <EEPROM.h>
#include <cstring>

#define NUM_LEGS 6
#define NUM_JOINTS 3
#define EEPROM_SIZE (NUM_LEGS * NUM_JOINTS * sizeof(int16_t))

class ConfigManager {
    private:
        static int16_t jointOffsets[NUM_LEGS][NUM_JOINTS];

    public:
        void initEEPROM();
        void saveLegOffsets();
        void loadLegOffsets();
        void SetLegConfig(int legNum, int joint, int offset);
        const int16_t* getLegOffsets(int legNum) const;
};

extern ConfigManager configManager;

#endif
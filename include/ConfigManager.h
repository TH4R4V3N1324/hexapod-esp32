#ifndef _CONFIG_MANAGER_H_
#define _CONFIG_MANAGER_H_

#include <cstdint>

/**
 * @brief Calibration offsets set from the controller (CMD_SET_CONFIG), kept in the ESP32's NVS flash
 * @note Degrees, per leg (gait_core index) and joint (coxa, femur, tibia), added on top of each
 *       servo's own centre. The centres themselves are stored in the servos (the console's
 *       calibrate command), so these are only for fine-tuning and are 0 until set.
*/
class ConfigManager {
public:
    void loadLegOffsets();
    void SetLegConfig(int legNum, int joint, int offset);
    const int16_t* getLegOffsets(int legNum) const { return jointOffsets[legNum]; }
    void clear();

private:
    int16_t jointOffsets[6][3] = {};
    void saveLegOffsets();
};

extern ConfigManager configManager;

#endif

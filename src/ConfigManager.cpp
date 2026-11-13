// gait_core (Eigen) before any Arduino header: Arduino defines macros such as B1 that break Eigen
#include "RobotConfig.h"

#include "ConfigManager.h"

#include <Arduino.h>
#include <Preferences.h>

#include <algorithm>
#include <cstring>

ConfigManager configManager;

namespace {

constexpr const char* NAMESPACE = "hexapod";
constexpr const char* OFFSETS_KEY = "offsets";

}  // namespace

void ConfigManager::loadLegOffsets() {
    Preferences prefs;
    memset(jointOffsets, 0, sizeof(jointOffsets));
    if (!prefs.begin(NAMESPACE, true)) {
        return;  // nothing saved yet
    }
    if (prefs.getBytesLength(OFFSETS_KEY) == sizeof(jointOffsets)) {
        prefs.getBytes(OFFSETS_KEY, jointOffsets, sizeof(jointOffsets));
    }
    prefs.end();
}

void ConfigManager::saveLegOffsets() {
    Preferences prefs;
    if (!prefs.begin(NAMESPACE, false)) {
        Serial.println("[config] Couldn't open NVS: offsets not saved");
        return;
    }
    prefs.putBytes(OFFSETS_KEY, jointOffsets, sizeof(jointOffsets));
    prefs.end();
}

void ConfigManager::SetLegConfig(int legNum, int joint, int offset) {
    offset = std::clamp(offset, -robot::MAX_CONFIG_OFFSET_DEG, robot::MAX_CONFIG_OFFSET_DEG);
    jointOffsets[legNum][joint] = static_cast<int16_t>(offset);
    saveLegOffsets();
    Serial.printf("[config] Leg %d joint %d offset %d deg\n", legNum, joint, offset);
}

void ConfigManager::clear() {
    memset(jointOffsets, 0, sizeof(jointOffsets));
    saveLegOffsets();
}

#ifndef _DATAPACKET_H_
#define _DATAPACKET_H_

#include <cstdint>

// Gait and Mode wire values match gait_core's; MODE_CONFIG is handled by the firmware itself.
// New values must be appended so existing ones don't change.
enum Gait : uint8_t {
    GAIT_TRIPOD,
    GAIT_RIPPLE,
    GAIT_WAVE,
    NUM_GAITS
};

enum Mode : uint8_t {
    MODE_NORMAL,
    MODE_STRAFE,
    MODE_TILT,
    MODE_CONFIG,
    NUM_MODES
};

enum Command : uint8_t {
    CMD_NONE = 0,
    CMD_SET_GAIT,
    CMD_SET_MODE,
    CMD_SET_CONFIG,
    CMD_HOME_STANCE,     // stand up; the feet return to their stance by themselves when idle
    CMD_REQUEST_CONFIG,
    CMD_SIT              // commandArgs[0]: 1 sit down, 0 stand up
};

#pragma pack(push, 1)
// Controller -> robot
struct ControlPacket {
    int16_t joystick1X;
    int16_t joystick1Y;
    int16_t joystick2X;
    int16_t joystick2Y;
    int16_t currentHeight;
    Command command;
    int16_t commandArgs[3];
};

// Robot -> controller
struct HexPacket {
    int16_t legConfigs[3];
    int16_t currentHeight;
    int8_t currentPhase;
    Gait currentGait;
    Mode currentMode;
};
#pragma pack(pop)

extern ControlPacket controlPacket;
extern HexPacket hexPacket;

/**
 * @brief Start ESP-NOW, listening for robot::CONTROLLER_MAC
 * @return false if ESP-NOW couldn't start
*/
bool InitEspNow();

/**
 * @brief Copy the latest packet from the controller into controlPacket
 * @return true if a new packet had arrived since the last call
*/
bool ReadInputData();

/**
 * @brief Milliseconds since the last packet from the controller (UINT32_MAX if none yet)
*/
uint32_t MsSinceLastPacket();

/**
 * @brief Send hexPacket to the controller
*/
void SendHexData();

/**
 * @brief true once each time controlPacket's command or its arguments change
*/
bool CommandChanged();

/**
 * @brief Print the address of a sender that isn't the controller, if one has been heard from
*/
void ReportUnknownSender();

#endif

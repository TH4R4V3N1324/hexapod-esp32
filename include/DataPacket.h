#ifndef _DATAPACKET_H_
#define _DATAPACKET_H_

#include <cstdint>

// ControlPacket, HexPacket, Command, Gait, Mode, GaitFeature and HEX_FLAG_*: the packets every
// controller and robot share, from hexapod-gait-core (include/hexapod_protocol/packets.h)
#include <hexapod_protocol/packets.h>

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

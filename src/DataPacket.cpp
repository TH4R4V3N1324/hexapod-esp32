// gait_core (Eigen) before any Arduino header: Arduino defines macros such as B1 that break Eigen
#include "RobotConfig.h"

#include "DataPacket.h"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <cstring>

ControlPacket controlPacket = {};
HexPacket hexPacket = {};

namespace {

Command lastCommand = CMD_NONE;
int16_t lastArgs[3] = {0, 0, 0};

// Written by the Wi-Fi task, read by the gait loop
portMUX_TYPE packetLock = portMUX_INITIALIZER_UNLOCKED;
ControlPacket received = {};
bool hasNewPacket = false;
bool hasAnyPacket = false;
uint32_t lastPacketMs = 0;
uint8_t unknownMac[6] = {};
bool hasUnknownSender = false;

void onReceive(const uint8_t* mac, const uint8_t* data, int length) {
    if (memcmp(mac, robot::CONTROLLER_MAC, 6) != 0) {
        portENTER_CRITICAL(&packetLock);
        memcpy(unknownMac, mac, 6);
        hasUnknownSender = true;
        portEXIT_CRITICAL(&packetLock);
        return;
    }
    if (length != sizeof(ControlPacket)) {
        return;
    }
    portENTER_CRITICAL(&packetLock);
    memcpy(&received, data, sizeof(ControlPacket));
    hasNewPacket = true;
    hasAnyPacket = true;
    lastPacketMs = millis();
    portEXIT_CRITICAL(&packetLock);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onReceiveInfo(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
    onReceive(info->src_addr, data, length);
}
#endif

}  // namespace

bool InitEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() != ESP_OK) {
        Serial.println("[esp-now] Failed to start");
        return false;
    }
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_now_register_recv_cb(onReceiveInfo);
#else
    esp_now_register_recv_cb(onReceive);
#endif

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, robot::CONTROLLER_MAC, 6);
    peer.channel = 0;  // the current channel
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println("[esp-now] Failed to add the controller as a peer");
        return false;
    }
    Serial.printf("[esp-now] Listening on %s for controller %02X:%02X:%02X:%02X:%02X:%02X\n",
                  WiFi.macAddress().c_str(), robot::CONTROLLER_MAC[0], robot::CONTROLLER_MAC[1],
                  robot::CONTROLLER_MAC[2], robot::CONTROLLER_MAC[3], robot::CONTROLLER_MAC[4],
                  robot::CONTROLLER_MAC[5]);
    return true;
}

bool ReadInputData() {
    bool isNew;
    portENTER_CRITICAL(&packetLock);
    isNew = hasNewPacket;
    if (isNew) {
        controlPacket = received;
        hasNewPacket = false;
    }
    portEXIT_CRITICAL(&packetLock);
    return isNew;
}

uint32_t MsSinceLastPacket() {
    portENTER_CRITICAL(&packetLock);
    const bool any = hasAnyPacket;
    const uint32_t last = lastPacketMs;
    portEXIT_CRITICAL(&packetLock);
    return any ? millis() - last : UINT32_MAX;
}

void SendHexData() {
    esp_now_send(robot::CONTROLLER_MAC, reinterpret_cast<const uint8_t*>(&hexPacket), sizeof(hexPacket));
}

bool CommandChanged() {
    const bool changed = controlPacket.command != lastCommand || controlPacket.commandArgs[0] != lastArgs[0] ||
                         controlPacket.commandArgs[1] != lastArgs[1] || controlPacket.commandArgs[2] != lastArgs[2];
    if (changed) {
        lastCommand = controlPacket.command;
        memcpy(lastArgs, controlPacket.commandArgs, sizeof(lastArgs));
    }
    return changed;
}

void ReportUnknownSender() {
    static uint32_t lastReportMs = 0;
    uint8_t mac[6];
    portENTER_CRITICAL(&packetLock);
    const bool unknown = hasUnknownSender;
    memcpy(mac, unknownMac, 6);
    hasUnknownSender = false;
    portEXIT_CRITICAL(&packetLock);
    if (unknown && millis() - lastReportMs > 5000) {
        lastReportMs = millis();
        Serial.printf("[esp-now] Ignored a packet from %02X:%02X:%02X:%02X:%02X:%02X: not robot::CONTROLLER_MAC\n",
                      mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}

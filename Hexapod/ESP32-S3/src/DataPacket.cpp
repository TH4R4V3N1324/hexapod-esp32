#include "DataPacket.h"

Command lastCommand = CMD_NONE;
int16_t lastArgs[3] = {0, 0, 0};

// Definitions for data packets
ControlPacket controlPacket;
HexPacket hexPacket;

uint8_t controllerMAC[6] = {0x80, 0x65, 0x99, 0xE9, 0x6F, 0x56};

// Function to handle espNOW receive event (Arduino ESP32 signature)
static void receiveEventEspNOW(const uint8_t *mac, const uint8_t *data, int len) {
    if (len != sizeof(ControlPacket)) {
        Serial.println("Received ESP-NOW data size incorrect");
        return;
    }
    ControlPacket incomingControlPacket = {};
    memcpy(&incomingControlPacket, data, sizeof(ControlPacket));
    if (controlPacketChanged(incomingControlPacket, controlPacket)) {
        controlPacket = incomingControlPacket;
    }
}

// Function to initialize ESP-NOW communication
void initEspNow() {
    // Initialize WiFi in station mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Initialize esp_now
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register the receive callback function
    esp_now_register_recv_cb(receiveEventEspNOW);

    // Add the controller MAC address as a peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, controllerMAC, sizeof(controllerMAC));
    peerInfo.channel = 0; // Use the current channel
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer");
        return;
    }
}

// Compare if two ControlPackets are different
bool controlPacketChanged(const ControlPacket& a, const ControlPacket& b) {
    return memcmp(&a, &b, sizeof(ControlPacket)) != 0;
}

// Sends hexapod data to ESP32 stored in hexPacket
void SendHexData() {
    esp_now_send(controllerMAC, (uint8_t*)&hexPacket, sizeof(HexPacket));
}

// Returns true if the a ControlPacket command changes
bool CommandChanged() {
    bool changed = (controlPacket.command != lastCommand) ||
                   (controlPacket.commandArgs[0] != lastArgs[0]) ||
                   (controlPacket.commandArgs[1] != lastArgs[1]) ||
                   (controlPacket.commandArgs[2] != lastArgs[2]);

    if (changed) {
        lastCommand = controlPacket.command;
        lastArgs[0] = controlPacket.commandArgs[0];
        lastArgs[1] = controlPacket.commandArgs[1];
        lastArgs[2] = controlPacket.commandArgs[2];
    }
    return changed;
}
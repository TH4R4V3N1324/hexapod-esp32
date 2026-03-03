#include <Arduino.h>
#include "Hexapod.h"

Animation animation;

// Handles command processing
void CommandFSM() {
  switch (controlPacket.command) {
    case CMD_SET_GAIT:
      animation.SetGait(static_cast<Gait>(controlPacket.commandArgs[0]));
      break;
    case CMD_SET_MODE:
      animation.SetMode(static_cast<Mode>(controlPacket.commandArgs[0]));
      break;
    case CMD_SET_CONFIG:
      configManager.SetLegConfig(controlPacket.commandArgs[0], controlPacket.commandArgs[1], controlPacket.commandArgs[2]);
      break;
    case CMD_HOME_STANCE:
      animation.returnToStart();
      break;
    case CMD_REQUEST_CONFIG:
        memcpy(hexPacket.legConfigs, configManager.getLegOffsets(controlPacket.commandArgs[0]), sizeof(int16_t) * 3);
      break;
    default:
      break;
  }
}

// Handles state transitions
void StateFSM() {
  switch (hexPacket.currentMode) {
    case MODE_NORMAL:
      animation.Normal();
      break;
    case MODE_STRAFE:
      animation.Strafe();
      break;
    case MODE_TILT:
      //animation.Tilt();
      break;
    case MODE_CONFIG:
      animation.ConfigState();
      break;
    default:
      break;
    }    
}

// Setup function
void setup() {
    i2cManager.init(20, 21);
    configManager.initEEPROM();
    configManager.loadLegOffsets();
    hexPacket.currentHeight = 120;

    delay(5000);
    animation.Startup();
}

// Main loop
void loop() {
  if(CommandChanged()) {CommandFSM();}
  StateFSM();
  delay(1);
}


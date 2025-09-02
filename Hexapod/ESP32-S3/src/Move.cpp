#include "Move.h"

using namespace std;

Move::Move(){
    SetupSwitches();
    servoController.init();
}

float Move::GetCurrentDraw() {
    // Placeholder implementation, replace with actual current draw logic
    return 0.0f;
}

//sets up the switches with pull down resistors if not setup already
void Move::SetupSwitches(){
    static bool initialized = false;

    if (!initialized) {
        // Placeholder for switch setup logic
        initialized = true;
    } 
}

//returns state of switch found in legSwitch
bool Move::GetSwitchStatus(int legNum){
    // Placeholder for switch status logic
    return false; // Default to false if not implemented
}

//returns leg position found in legPosition if it exists
Vector3 Move::GetLegPosition(int legNum) const{
    if (Calculate::legPosition.find(legNum) != Calculate::legPosition.end()) {
        return Calculate::legPosition.at(legNum).position;
    } else {
        return {};
    }
}

//moves leg tip to position through a vector
void Move::Position(const Vector3& position, int legNum){
    LegServo Servos = legs.at(legNum);
    JointAngles angles = Cal.angle(position, legNum);

    // Apply offsets to angles
    angles = Cal.ApplyOffsets(angles, legNum);

    // Assign angles to servos directly (assuming 3 servos per leg)
    servoController.setAngle(Servos.coxa, angles.coxaAngle);
    servoController.setAngle(Servos.femur, angles.femurAngle);
    servoController.setAngle(Servos.tibia, angles.tibiaAngle);

    Calculate::legPosition[legNum].position = position;
    Calculate::legPosition[legNum].angles = angles;
};

//turns the servos off in a given leg
void Move::Deactivate(int legNum){
    LegServo Servos = legs.at(legNum);

    servoController.disable(Servos.coxa);
    servoController.disable(Servos.femur);
    servoController.disable(Servos.tibia);
};

// Sets leg angles to specific values
void Move::Angles(JointAngles& angles, int legNum) {
    // Apply offsets to angles
    angles = Cal.ApplyOffsets(angles, legNum);

    // Assign angles to servos directly (assuming 3 servos per leg)
    LegServo Servos = legs.at(legNum);
    servoController.setAngle(Servos.coxa, angles.coxaAngle);
    servoController.setAngle(Servos.femur, angles.femurAngle);
    servoController.setAngle(Servos.tibia, angles.tibiaAngle);

    Calculate::legPosition[legNum].angles = angles;
    Calculate::legPosition[legNum].position = Cal.position(angles, legNum);
}
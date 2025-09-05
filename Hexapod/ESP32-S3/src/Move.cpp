#include "Move.h"

using namespace std;

Move::Move(){
    servoController.init();
}

/*
@brief Get the current position of the specified leg.
@param legNum The leg number (1-6).
@return The current position of the leg as a Vector3 object.
*/
Vector3 Move::GetLegPosition(int legNum) const{
    if (Calculate::legPosition.find(legNum) != Calculate::legPosition.end()) {
        return Calculate::legPosition.at(legNum).position;
    } else {
        return {};
    }
}

/*
@brief Move the specified leg to a given position.
@param position The target position as a Vector3 object.
@param legNum The leg number (1-6).
@return void
*/
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

/*
brief Deactivate the specified leg by disabling its servos.
@param legNum The leg number (1-6).
@return void
*/
void Move::Deactivate(int legNum){
    LegServo Servos = legs.at(legNum);

    servoController.disable(Servos.coxa);
    servoController.disable(Servos.femur);
    servoController.disable(Servos.tibia);
};

/*
@brief Set the angles of the specified leg's servos.
@param angles The target joint angles as a JointAngles object.
@param legNum The leg number (1-6).
@return void
*/
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
#ifndef _MOVE_H_
#define _MOVE_H_
#include "Servo.h"
#include "Calculate.h"
#include "DataPacket.h"
#include <map>
#include <cmath>
#include <iostream>

struct LegServo {
    int coxa;
    int femur;
    int tibia;
};

class Move{
private:
    Servo servoController;

    //assigns the relavant servos to their corrosponding leg
    std::map<int, LegServo> legs = {
        {1, {0, 1, 2}},
        {2, {3, 4, 5}},
        {3, {6, 7, 8}},
        {4, {9, 10, 11}},
        {5, {12, 13, 14}},
        {6, {15, 16, 17}}
    };

public:
    Move();
    Calculate Cal;
    Vector3 GetLegPosition(int legNum) const;
    void Position(const Vector3& position, int legNum);
    void Deactivate(int legNum);
    void Angles(JointAngles& angles, int legNum);
};

#endif 
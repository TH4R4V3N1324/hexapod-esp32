#ifndef SENSOR_H
#define SENSOR_H

#include "LegContactSensor.h"
#include "CurrentSensor.h"
#include "IMUSensor.h"

class Sensor {
public:
    LegContactSensor legContactSensor;
    CurrentSensor currentSensor;
    IMUSensor imuSensor;
    void init() {
        legContactSensor.init();
        currentSensor.init();
        imuSensor.init();
    }
};

#endif
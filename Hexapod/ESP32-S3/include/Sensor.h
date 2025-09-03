#ifndef SENSOR_H
#define SENSOR_H

#include "LegContactSensor.h"

class Sensor {
public:
    LegContactSensor legContactSensor;
    void init() {
        legContactSensor.init();
    }
};

#endif
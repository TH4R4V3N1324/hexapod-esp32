#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "SensorBase.h"

class CurrentSensor : public SensorBase {
public:
    void init() override;
    float readCurrent();
};

#endif
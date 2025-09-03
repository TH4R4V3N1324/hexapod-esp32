#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "SensorBase.h"
#include "Adafruit_INA260.h"

class CurrentSensor : public SensorBase {
private:
    Adafruit_INA260 ina260;
public:
    void init() override;
    float readCurrent();
    float readVoltage();
    float readPower();
};

#endif
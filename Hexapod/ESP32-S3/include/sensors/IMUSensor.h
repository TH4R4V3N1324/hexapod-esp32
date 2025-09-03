#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

#include "SensorBase.h"

class IMUSensor : public SensorBase {
public:
    void init() override;
};

#endif
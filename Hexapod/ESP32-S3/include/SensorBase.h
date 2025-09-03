#ifndef SENSOR_BASE_H
#define SENSOR_BASE_H

class SensorBase {
public:
    virtual void init() = 0;
    virtual ~SensorBase() {}
};

#endif
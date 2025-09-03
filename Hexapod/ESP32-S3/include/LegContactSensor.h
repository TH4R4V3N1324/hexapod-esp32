#ifndef LEG_CONTACT_SENSOR_H
#define LEG_CONTACT_SENSOR_H

#include "SensorBase.h"
#include "driver/gpio.h"

#define SENSOR1_PIN 34
#define SENSOR2_PIN 35
#define SENSOR3_PIN 36
#define SENSOR4_PIN 37
#define SENSOR5_PIN 38
#define SENSOR6_PIN 39

class LegContactSensor : public SensorBase {
private:
    static const gpio_num_t LEG_SENSORS[];
public:
    LegContactSensor();
    void init() override;
    bool readState(int legNum);
};

#endif
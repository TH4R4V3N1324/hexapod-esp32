#include "CurrentSensor.h"

void CurrentSensor::init() {
    ina260.begin();
}

float CurrentSensor::readCurrent() {
    return ina260.readCurrent();
}

float CurrentSensor::readVoltage() {
    return ina260.readBusVoltage();
}

float CurrentSensor::readPower() {
    return ina260.readPower();
}

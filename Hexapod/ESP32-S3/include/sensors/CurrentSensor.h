#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "SensorBase.h"
#include "I2CManager.h"

#define INA260_ADDRESS           0x40
#define INA260_REG_CONFIG        0x00
#define INA260_REG_CURRENT       0x01
#define INA260_REG_BUS_VOLTAGE   0x02
#define INA260_REG_POWER         0x03
#define INA260_REG_MASK_ENABLE   0x06
#define INA260_REG_ALERT_LIMIT   0x07

enum TriggerMode {
    ALERT_OVERCURRENT = 0x8000,
    ALERT_UNDERCURRENT = 0x4000,
    ALERT_OVERVOLTAGE = 0x2000,
    ALERT_UNDERVOLTAGE = 0x1000,
    ALERT_OVERPOWER = 0x0800
};

class CurrentSensor : public SensorBase {
private:
    uint16_t readRegister(uint8_t reg);
public:
    void init() override;
    float readCurrent();
    float readVoltage();
    float readPower();
    void setTriggerMode(TriggerMode mode);
    void setTriggerThreshold(float threshold);
};

#endif
#include "LegContactSensor.h"

LegContactSensor::LegContactSensor() {
    // Constructor implementation
}

void LegContactSensor::init() {
    // Initialization code
    for (int i = 0; i < sizeof(SENSORS) / sizeof(SENSORS[0]); i++) {
        gpio_set_direction(SENSORS[i], GPIO_MODE_INPUT);
    }

    for (int i = 0; i < sizeof(SENSORS) / sizeof(SENSORS[0]); i++) {
        gpio_set_pull_mode(SENSORS[i], GPIO_PULLUP_ONLY);
    }
}

bool LegContactSensor::readState(int legNum) {
    if (legNum < 0 || legNum >= sizeof(SENSORS) / sizeof(SENSORS[0])) {
        return false;
    }

    bool state = gpio_get_level(SENSORS[legNum]);
    return state;

}

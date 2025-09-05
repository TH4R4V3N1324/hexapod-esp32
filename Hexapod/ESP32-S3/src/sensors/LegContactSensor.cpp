#include "LegContactSensor.h"

const gpio_num_t LegContactSensor::LEG_SENSORS[] = {
    gpio_num_t(SENSOR1_PIN),
    gpio_num_t(SENSOR2_PIN),
    gpio_num_t(SENSOR3_PIN),
    gpio_num_t(SENSOR4_PIN),
    gpio_num_t(SENSOR5_PIN),
    gpio_num_t(SENSOR6_PIN)
};

LegContactSensor::LegContactSensor() {
    // Constructor implementation
}

/*
@brief Initialize the leg contact sensors
*/
void LegContactSensor::init() {
    // Initialization code
    for (int i = 0; i < sizeof(LEG_SENSORS) / sizeof(LEG_SENSORS[0]); i++) {
        gpio_set_direction(LEG_SENSORS[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(LEG_SENSORS[i], GPIO_PULLUP_ONLY);
    }
}

/*
@brief Read the state of a specific leg contact sensor
@param legNum The leg number to read (0-5)
@return The state of the leg contact sensor (true = contact, false = no contact)
*/
bool LegContactSensor::readState(int legNum) {
    if (legNum < 0 || legNum >= sizeof(LEG_SENSORS) / sizeof(LEG_SENSORS[0])) {
        return false;
    }

    bool state = gpio_get_level(LEG_SENSORS[legNum]);
    return state;

}

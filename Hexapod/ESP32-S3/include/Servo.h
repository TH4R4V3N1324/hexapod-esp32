#ifndef _SERVO_H_
#define _SERVO_H_

#include "driver/mcpwm.h"
#include "driver/ledc.h"

#define NUM_MCPWM_SERVOS 12
#define NUM_LEDC_SERVOS 6

#define SERVO1_GPIO 1
#define SERVO2_GPIO 2
#define SERVO3_GPIO 3
#define SERVO4_GPIO 4
#define SERVO5_GPIO 5
#define SERVO6_GPIO 6
#define SERVO7_GPIO 7
#define SERVO8_GPIO 8
#define SERVO9_GPIO 9
#define SERVO10_GPIO 10
#define SERVO11_GPIO 11
#define SERVO12_GPIO 12
#define SERVO13_GPIO 13
#define SERVO14_GPIO 14
#define SERVO15_GPIO 15
#define SERVO16_GPIO 16
#define SERVO17_GPIO 17
#define SERVO18_GPIO 18

struct mcpwm_servo {
    uint8_t gpio_num;
    mcpwm_unit_t mcpwm_unit;
    mcpwm_timer_t mcpwm_timer;
    mcpwm_io_signals_t io_signal;
    mcpwm_generator_t gen;
};

struct ledc_servo {
    uint8_t gpio_num;
    ledc_channel_t channel;
};

class Servo {
private:
    mcpwm_servo MCPWM_SERVOS[NUM_MCPWM_SERVOS];
    ledc_servo LEDC_SERVOS[NUM_LEDC_SERVOS];
    
public:
    void init();
    void disable(int servoNum);
    void setServoAngle(int servoNum, double angle);
};

#endif

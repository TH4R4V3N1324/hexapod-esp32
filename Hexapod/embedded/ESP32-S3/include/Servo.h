#ifndef _SERVO_H_
#define _SERVO_H_

#include "driver/mcpwm.h"
#include "driver/ledc.h"

enum ServoType { SERVO_MCPWM, SERVO_LEDC };

struct ServoConfig {
    uint8_t gpio_num;
    ServoType type;
    union {
        struct { mcpwm_unit_t unit; mcpwm_timer_t timer; mcpwm_io_signals_t io_signal; mcpwm_generator_t gen; } mcpwm;
        struct { ledc_channel_t channel; } ledc;
    };
};

class Servo {
private:
    static const int NUM_SERVOS = 18;
    ServoConfig servos[NUM_SERVOS];
    void initMCPWM(const ServoConfig& cfg);
    void initLEDC(const ServoConfig& cfg);

public:
    Servo();
    void init();
    void disable(int servoNum);
    void setAngle(int servoNum, double angle);
};

#endif

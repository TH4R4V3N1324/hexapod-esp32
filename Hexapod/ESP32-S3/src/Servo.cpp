#include "Servo.h"

mcpwm_servo MCPWM_SERVOS[NUM_MCPWM_SERVOS] = {
    {SERVO1_GPIO, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM_OPR_A},
    {SERVO2_GPIO, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, MCPWM_OPR_B},
    {SERVO3_GPIO, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM_OPR_A},
    {SERVO4_GPIO, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1B, MCPWM_OPR_B},
    {SERVO5_GPIO, MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2A, MCPWM_OPR_A},
    {SERVO6_GPIO, MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2B, MCPWM_OPR_B},
    {SERVO7_GPIO, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM_OPR_A},
    {SERVO8_GPIO, MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0B, MCPWM_OPR_B},
    {SERVO9_GPIO, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM_OPR_A},
    {SERVO10_GPIO, MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1B, MCPWM_OPR_B},
    {SERVO11_GPIO, MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM_OPR_A},
    {SERVO12_GPIO, MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2B, MCPWM_OPR_B}
};

ledc_servo LEDC_SERVOS[NUM_LEDC_SERVOS] = {
    {SERVO13_GPIO, LEDC_CHANNEL_0},
    {SERVO14_GPIO, LEDC_CHANNEL_1},
    {SERVO15_GPIO, LEDC_CHANNEL_2},
    {SERVO16_GPIO, LEDC_CHANNEL_3},
    {SERVO17_GPIO, LEDC_CHANNEL_4},
    {SERVO18_GPIO, LEDC_CHANNEL_5}
};

void Servo::init_servos() {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // Set frequency to 50Hz
    pwm_config.cmpr_a = 0;     // Set duty cycle to 0%
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    for (int i = 0; i < NUM_MCPWM_SERVOS; i++) {
        mcpwm_gpio_init(MCPWM_SERVOS[i].mcpwm_unit, MCPWM_SERVOS[i].io_signal, MCPWM_SERVOS[i].gpio_num);
        mcpwm_init(MCPWM_SERVOS[i].mcpwm_unit, MCPWM_SERVOS[i].mcpwm_timer, &pwm_config);
    }

    ledc_timer_config_t ledc_timer;
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
    ledc_timer.freq_hz = 50;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < NUM_LEDC_SERVOS; i++) {
        ledc_channel_config_t ledc_channel;
        ledc_channel.gpio_num = LEDC_SERVOS[i].gpio_num;
        ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_channel.channel = LEDC_SERVOS[i].channel;
        ledc_channel.timer_sel = LEDC_TIMER_0;
        ledc_channel.duty = 0;
        ledc_channel.hpoint = 0;
        ledc_channel_config(&ledc_channel);
    }
}
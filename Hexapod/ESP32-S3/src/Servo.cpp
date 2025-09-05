#include "Servo.h"
#include "Arduino.h"

// Example safe GPIOs for 12 MCPWM and 6 LEDC servos
static ServoConfig defaultServos[18] = {
    // MCPWM servos
    {4,  SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM_OPR_A}}},
    {5,  SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, MCPWM_OPR_B}}},
    {12, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM_OPR_A}}},
    {13, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1B, MCPWM_OPR_B}}},
    {14, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2A, MCPWM_OPR_A}}},
    {15, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM2B, MCPWM_OPR_B}}},
    {16, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0A, MCPWM_OPR_A}}},
    {17, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM0B, MCPWM_OPR_B}}},
    {18, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM_OPR_A}}},
    {19, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1B, MCPWM_OPR_B}}},
    {21, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM_OPR_A}}},
    {22, SERVO_MCPWM, {.mcpwm = {MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2B, MCPWM_OPR_B}}},
    // LEDC servos
    {23, SERVO_LEDC, {.ledc = {LEDC_CHANNEL_0}}},
    {25, SERVO_LEDC, {.ledc = {LEDC_CHANNEL_1}}},
    {26, SERVO_LEDC, {.ledc = {LEDC_CHANNEL_2}}},
    {27, SERVO_LEDC, {.ledc = {LEDC_CHANNEL_3}}},
    {32, SERVO_LEDC, {.ledc = {LEDC_CHANNEL_4}}},
    {33, SERVO_LEDC, {.ledc = {LEDC_CHANNEL_5}}},
};

// Constructor
Servo::Servo() {
    for (int i = 0; i < NUM_SERVOS; ++i) servos[i] = defaultServos[i];
}

/*
@brief Initialize MCPWM for a specific servo
@param cfg The ServoConfig for the servo to initialize
*/
void Servo::initMCPWM(const ServoConfig& cfg) {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_gpio_init(cfg.mcpwm.unit, cfg.mcpwm.io_signal, cfg.gpio_num);
    mcpwm_init(cfg.mcpwm.unit, cfg.mcpwm.timer, &pwm_config);
}

/*
@brief Initialize LEDC for a specific servo
@param cfg The ServoConfig for the servo to initialize
*/
void Servo::initLEDC(const ServoConfig& cfg) {
    static bool timerConfigured = false;
    if (!timerConfigured) {
        ledc_timer_config_t ledc_timer;
        ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
        ledc_timer.timer_num = LEDC_TIMER_0;
        ledc_timer.duty_resolution = LEDC_TIMER_13_BIT;
        ledc_timer.freq_hz = 50;
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer_config(&ledc_timer);
        timerConfigured = true;
    }
    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = cfg.gpio_num;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = cfg.ledc.channel;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel.duty = 0;
    ledc_channel.hpoint = 0;
    ledc_channel_config(&ledc_channel);
}

// Initialize all servos
void Servo::init() {
    for (int i = 0; i < NUM_SERVOS; ++i) {
        if (servos[i].type == SERVO_MCPWM) initMCPWM(servos[i]);
        else if (servos[i].type == SERVO_LEDC) initLEDC(servos[i]);
    }
}

/*
@brief Disable a specific servo
@param servoNum The index of the servo to disable (0-17)
*/
void Servo::disable(int servoNum) {
    if (servoNum < 0 || servoNum >= NUM_SERVOS) return;
    if (servos[servoNum].type == SERVO_MCPWM) {
        mcpwm_stop(servos[servoNum].mcpwm.unit, servos[servoNum].mcpwm.timer);
    } else if (servos[servoNum].type == SERVO_LEDC) {
        ledc_stop(LEDC_LOW_SPEED_MODE, servos[servoNum].ledc.channel, 0);
    }
}

/*
@brief Set the angle of a specific servo
@param servoNum The index of the servo to set (0-17)
@param angle The target angle in degrees
*/
void Servo::setAngle(int servoNum, double angle) {
    if (servoNum < 0 || servoNum >= NUM_SERVOS) return;
    if (servos[servoNum].type == SERVO_MCPWM) {
        int32_t pulse_us = 1000 + (int)((angle / 180.0) * 1000);
        mcpwm_set_duty_in_us(servos[servoNum].mcpwm.unit, servos[servoNum].mcpwm.timer, servos[servoNum].mcpwm.gen, pulse_us);
    } else if (servos[servoNum].type == SERVO_LEDC) {
        int pulse_us = 1000 + (int)((angle / 180.0) * 1000);  // 1000–2000us
        int duty = (pulse_us * ((1 << 13) - 1)) / 20000;
        ledc_set_duty(LEDC_LOW_SPEED_MODE, servos[servoNum].ledc.channel, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, servos[servoNum].ledc.channel);
    }
}

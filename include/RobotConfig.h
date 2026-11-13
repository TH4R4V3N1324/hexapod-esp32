#ifndef _ROBOT_CONFIG_H_
#define _ROBOT_CONFIG_H_

#include <array>
#include <cstdint>

#include "gait_core/gait_engine.hpp"
#include "gait_core/robot_geometry.hpp"
#include "hexapod_behaviour/battery_monitor.hpp"

/**
 * Everything specific to running the hexapod on an ESP32 with ST3215 bus servos: the pins, its
 * shape (gait_core's preset), the gait settings and how gait_core's joint angles map onto the servos.
 *
 * gait_core works in SI units with the REP 103 body frame (+x forward, +y left, +z up), and legs
 * indexed 0-5 clockwise from the front right (legs 1-6 in hexapod-docs). Servo IDs are
 * leg (counted from 1) x 10 + joint (coxa 1, femur 2, tibia 3).
 *
 *   gait_core::Leg       leg   servo IDs
 *   LEG_FRONT_RIGHT  0   1     11, 12, 13   (coxa, femur, tibia)
 *   LEG_MIDDLE_RIGHT 1   2     21, 22, 23
 *   LEG_REAR_RIGHT   2   3     31, 32, 33
 *   LEG_REAR_LEFT    3   4     41, 42, 43
 *   LEG_MIDDLE_LEFT  4   5     51, 52, 53
 *   LEG_FRONT_LEFT   5   6     61, 62, 63
*/
namespace robot {

using gait_core::Scalar;

// ---------------------------------------------------------------------------------------------
// Pins
// ---------------------------------------------------------------------------------------------

// Servo bus: UART2 to a Waveshare Bus Servo Adapter (A). ESP32 TX goes to the
// adapter's RX and ESP32 RX to its TX.
constexpr int SERVO_RX_PIN = 16;
constexpr int SERVO_TX_PIN = 17;
constexpr uint32_t SERVO_BAUD = 1000000;   // every ST3215 on the robot must be set to this
constexpr bool SERVO_ECHO = false;         // true if the adapter loops transmitted bytes back to RX

// I2C bus: the IMU and both leg hubs' INA226s (the ESP32's default pins)
constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 22;
constexpr uint32_t I2C_FREQUENCY = 400000;

// SERVO_EN on the leg hubs' J7 (pin 4), both hubs on the one pin so the sides switch as one. The
// hubs power the servos only while it's high and the kill loop is closed. GPIO 25 isn't a
// strapping pin, so it floats at boot and the hubs' pull-downs keep the servos off until the gait
// asks for them. -1 if there's no enable line (servos always powered).
constexpr int SERVO_ENABLE_PIN = 25;

// ---------------------------------------------------------------------------------------------
// Controller (ESP-NOW)
// ---------------------------------------------------------------------------------------------

// MAC address of the handheld controller. Packets from any other address are ignored, and the
// firmware prints the sender's address so you can copy it here.
constexpr uint8_t CONTROLLER_MAC[6] = {0x80, 0x65, 0x99, 0xE9, 0x6F, 0x56};

constexpr int JOYSTICK_FULL_SCALE = 128;   // joystick value at full deflection
constexpr int JOYSTICK_DEADZONE = 10;      // values this close to centre count as centred
constexpr Scalar VELOCITY_SMOOTHING = Scalar(0.5);  // per tick
constexpr uint32_t COMMS_TIMEOUT_MS = 200; // no packet for this long and the sticks count as centred

// ---------------------------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------------------------

/**
 * @brief gait_core's explorer() geometry preset
 * @note ST3215s turn +-180 degrees, so the preset's joint limits need no narrowing. The servos'
 *       own angle limits, set with the console's calibrate command, guard the end stops.
*/
inline gait_core::RobotGeometry geometry() {
    return gait_core::RobotGeometry::explorer();
}

// ---------------------------------------------------------------------------------------------
// Gait settings
// ---------------------------------------------------------------------------------------------

/**
 * @brief GaitEngine settings; anything not set here keeps gait_core's default
*/
inline gait_core::GaitParams gaitParams() {
    gait_core::GaitParams p;

    // With a servo power switch the servos stay off until the first stand command (CMD_HOME_STANCE,
    // or CMD_SIT 0); without one they're powered from the start, and startup seats the robot
    p.power_on_startup = SERVO_ENABLE_PIN < 0;

    // The ST3215s report their load, so the foot forces are there: load balancing and terrain
    // adaptation stay on. Leveling uses the IMU.
    return p;
}

// ---------------------------------------------------------------------------------------------
// Servos
// ---------------------------------------------------------------------------------------------

/**
 * @brief Servo ID of a joint: leg (gait_core index, from 0) and joint (0 coxa, 1 femur, 2 tibia)
*/
constexpr uint8_t servoId(int legNum, int joint) {
    return static_cast<uint8_t>((legNum + 1) * 10 + joint + 1);
}

/**
 * @brief servo (deg from its centre) = JOINT_DIRECTION * joint (deg), then the controller's offset
 * @note Each servo's centre (2048 steps) is the joint at 0: the leg laid out flat. Set it with the
 *       console's calibrate command, which stores it in the servo. If a joint turns the wrong way,
 *       flip its direction.
*/
constexpr std::array<float, 3> JOINT_DIRECTION = {1, 1, -1};  // coxa, femur, tibia

constexpr float MAX_TORQUE_NM = 2.94f;   // ST3215 stall torque at 12 V, the load reading at 100%
constexpr uint8_t SERVO_ACCELERATION = 0;  // 0 = no limit
constexpr uint16_t SERVO_SPEED = 0;        // steps/s, 0 = no limit

// Calibration offsets set from the controller (CMD_SET_CONFIG), on top of each servo's own centre
constexpr int MAX_CONFIG_OFFSET_DEG = 60;

// Speed (deg/s) servos move at into and out of the calibration pose (MODE_CONFIG: every joint at 0)
constexpr float CONFIG_SLEW_DEG_PER_S = 90.0f;

// ---------------------------------------------------------------------------------------------
// IMU
// ---------------------------------------------------------------------------------------------

// ISM330DHCX, mounted with its axes along the body's (x forward, y left, z up)
constexpr uint8_t IMU_ADDRESS = 0x6A;        // 0x6B if the address jumper is bridged
constexpr int IMU_ACCEL_RANGE_G = 8;         // 2, 4, 8 or 16
constexpr int IMU_GYRO_RANGE_DPS = 1000;     // 125, 250, 500, 1000, 2000 or 4000
constexpr float IMU_FILTER_GAIN = 0.1f;      // Madgwick filter gain

// ---------------------------------------------------------------------------------------------
// Power (leg hubs)
// ---------------------------------------------------------------------------------------------

// Each leg hub's INA226 measures everything its board draws: its three legs and its 5 V output.
// Left hub JP1 open = 0x40, right hub JP1 bridged = 0x41.
constexpr std::array<uint8_t, 2> LEG_HUB_ADDRESSES = {0x40, 0x41};
constexpr std::array<const char*, 2> LEG_HUB_NAMES = {"left", "right"};
constexpr float LEG_HUB_SHUNT_OHMS = 0.002f;    // R1, 2 mOhm
constexpr uint32_t POWER_READ_PERIOD_US = 100000;    // INA226s read at 10 Hz
constexpr uint32_t BATTERY_UPDATE_PERIOD_US = 500000; // battery decisions at 2 Hz

/**
 * @brief Battery behaviour (3S LiPo): a warning when low; at critical, sit down and turn the servo
 *        power off. Starting points: tune them against real readings under load.
*/
inline hexapod_behaviour::BatteryParams batteryParams() {
    hexapod_behaviour::BatteryParams p;
    p.warning_voltage = 10.5f;   // 3.5 V/cell
    p.critical_voltage = 9.9f;   // 3.3 V/cell
    p.hysteresis = 0.2f;         // V above a threshold before it clears
    p.filter_time = 5.0f;        // s, so the dips while walking don't trip it
    p.stale_timeout = 2.0f;      // s without a reading before a hub is ignored
    p.auto_sit = true;
    p.power_off = true;          // once seated at critical, turn the servo power off
    return p;
}

// After switching servo power on from the console, how long to let the servos boot
constexpr uint32_t SERVO_POWER_SETTLE_MS = 500;

// ---------------------------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------------------------

// Gait loop rate. Reading and writing the 18 servos takes ~4 ms of each tick.
constexpr uint32_t LOOP_PERIOD_US = 10000;   // 100 Hz
constexpr Scalar MAX_TICK_DT = Scalar(0.1);  // s, a stalled tick can't make the legs jump further than this

}  // namespace robot

#endif

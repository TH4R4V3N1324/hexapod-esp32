#ifndef _SERVOS_H_
#define _SERVOS_H_

#include <array>
#include <cstdint>

#include "gait_core/gait_engine.hpp"

#include "St3215Bus.h"

/**
 * @brief Servo angles of one leg in degrees from each servo's centre: coxa, femur, tibia
*/
using ServoDegrees = std::array<float, 3>;

/**
 * @brief The 18 leg servos: ST3215s on one serial bus, IDs leg x 10 + joint (robot::servoId), and
 *        their power switch (the leg hubs' SERVO_EN)
 * @note Legs are indexed like gait_core (0 = front right, clockwise). Servo n in the arrays is
 *       leg n / 3, joint n % 3.
 *
 *       Each servo is either answering or not. Unpowered servos (SERVO_EN low, kill loop open)
 *       don't answer, and a leg with one that isn't is reported to the engine as missing. Only one
 *       silent servo is tried each read, since each costs a reply timeout. A servo that starts
 *       answering again is held where it is with torque on, ignoring the target it had before,
 *       until a new one arrives, so it never jumps to a stale target.
*/
class Servos {
public:
    static constexpr int NUM_SERVOS = gait_core::NUM_LEGS * 3;

    Servos();

    /**
     * @brief Open the servo bus, with servo power off
    */
    void begin();

    /**
     * @brief Switch servo power (SERVO_EN) on or off
     * @note The servos need a moment to boot after it goes on; read() picks them up as they answer.
     *       Does nothing without an enable line (robot::SERVO_ENABLE_PIN -1): they're always on.
    */
    void setPower(bool on);
    /**
     * @brief Switch servo power on and wait for the servos to boot (for the console)
    */
    void powerOnAndWait();
    bool powered() const { return isPowered; }

    /**
     * @brief Read every answering servo's position and load, and try one silent one, as a joint state
    */
    void read(gait_core::JointFeedback& feedback, uint64_t nowUs);

    /**
     * @brief Move every answering servo, in one packet, adding the controller's calibration offsets
     * @param[in] degrees Servo angles before calibration offsets
     * @note Servos that aren't answering, or are holding until a new target arrives, are skipped
    */
    void write(const std::array<ServoDegrees, gait_core::NUM_LEGS>& degrees);

    /**
     * @brief Turn the torque off on every servo on the bus. The legs will drop.
    */
    void limp();

    int answeringCount() const;

    /**
     * @brief gait_core joint angles (rad) to servo angles (deg from centre), before calibration offsets
    */
    static ServoDegrees toServoDegrees(const gait_core::JointAngles& angles);
    /**
     * @brief Servo angles (deg from centre, before calibration offsets) back to gait_core joint angles (rad)
    */
    static gait_core::JointAngles toJointAngles(const ServoDegrees& degrees);

    static float stepsToDegrees(int steps);
    static int degreesToSteps(float degrees);

    St3215Bus& bus() { return servoBus; }
    const std::array<uint8_t, NUM_SERVOS>& ids() const { return servoIds; }

private:
    struct Joint {
        bool answering = false;
        int missed = 0;          // replies missed in a row
        bool holding = false;    // answering again: held where it is until a new target arrives
        int heldTarget = -1;     // the target (steps) it had when it started answering again
    };

    St3215Bus servoBus;
    std::array<uint8_t, NUM_SERVOS> servoIds{};
    std::array<St3215State, NUM_SERVOS> states{};
    std::array<Joint, NUM_SERVOS> joints{};
    std::array<int, NUM_SERVOS> targets{};  // last target written (steps), -1 for none yet
    int nextProbe = 0;                      // which silent servo to try next
    bool isPowered = false;
    uint64_t lastNoReplyWarningUs = 0;
    uint64_t lastErrorWarningUs = 0;

    float withOffset(int servo, float degrees) const;
    float withoutOffset(int servo, float degrees) const;
};

#endif

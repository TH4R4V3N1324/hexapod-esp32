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
 * @brief The 18 leg servos: ST3215s on one serial bus, IDs leg x 10 + joint (robot::servoId)
 * @note Legs are indexed like gait_core (0 = front right, clockwise). Servo n in the arrays is leg n / 3, joint n % 3.
*/
class Servos {
public:
    static constexpr int NUM_SERVOS = gait_core::NUM_LEGS * 3;

    Servos();

    /**
     * @brief Open the servo bus
    */
    void begin();

    /**
     * @brief Ping every leg servo
     * @param[out] missing The IDs that didn't reply
     * @return How many are missing
    */
    int findMissing(std::array<uint8_t, NUM_SERVOS>& missing);

    /**
     * @brief Hold every servo where it is and turn its torque on, so the legs don't jump
     * @param[out] held The servo angles they're held at (deg, before calibration offsets)
     * @return false if a servo didn't reply (nothing is turned on then)
    */
    bool holdCurrentPose(std::array<ServoDegrees, gait_core::NUM_LEGS>& held);

    /**
     * @brief Read every servo's position and load as a joint state for the engine
     * @note A servo that doesn't reply keeps its last reading; a warning is
     *       printed if it stays silent.
    */
    void read(gait_core::JointFeedback& feedback, uint64_t nowUs);

    /**
     * @brief Move every servo, in one packet, adding the controller's calibration offsets
     * @param[in] degrees Servo angles before calibration offsets
    */
    void write(const std::array<ServoDegrees, gait_core::NUM_LEGS>& degrees);

    /**
     * @brief Turn the torque off on every servo on the bus. The legs will drop.
    */
    void limp();

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
    const St3215State& state(int servo) const { return states[servo]; }

private:
    St3215Bus servoBus;
    std::array<uint8_t, NUM_SERVOS> servoIds{};
    std::array<St3215State, NUM_SERVOS> states{};
    std::array<uint64_t, NUM_SERVOS> lastReplyUs{};
    uint64_t lastWarningUs = 0;

    float withOffset(int servo, float degrees) const;
    float withoutOffset(int servo, float degrees) const;
};

#endif

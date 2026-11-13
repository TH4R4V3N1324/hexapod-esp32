#include "Servos.h"

#include <algorithm>
#include <cmath>

#include "ConfigManager.h"
#include "RobotConfig.h"

namespace {

constexpr float kRadToDeg = 180.0f / gait_core::kPi;
constexpr float kDegToRad = gait_core::kPi / 180.0f;

// A servo silent for this long is reported (s)
constexpr uint64_t SILENT_WARNING_US = 500000;
constexpr uint64_t WARNING_PERIOD_US = 2000000;

}  // namespace

Servos::Servos() {
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
        servoIds[servo] = robot::servoId(servo / 3, servo % 3);
    }
}

void Servos::begin() {
    servoBus.begin(Serial2, robot::SERVO_BAUD, robot::SERVO_RX_PIN, robot::SERVO_TX_PIN, robot::SERVO_ECHO);
}

int Servos::findMissing(std::array<uint8_t, NUM_SERVOS>& missing) {
    int count = 0;
    for (uint8_t id : servoIds) {
        if (!servoBus.ping(id)) {
            missing[count++] = id;
        }
    }
    return count;
}

bool Servos::holdCurrentPose(std::array<ServoDegrees, gait_core::NUM_LEGS>& held) {
    bool received[NUM_SERVOS];
    if (servoBus.readStates(servoIds.data(), NUM_SERVOS, states.data(), received) != NUM_SERVOS) {
        return false;
    }
    std::array<int, NUM_SERVOS> positions;
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
        positions[servo] = states[servo].position;
        held[servo / 3][servo % 3] = withoutOffset(servo, stepsToDegrees(states[servo].position));
        lastReplyUs[servo] = esp_timer_get_time();
    }
    // Goals first, so the servos hold where they are when the torque comes on
    return servoBus.writeGoalPositions(servoIds.data(), positions.data(), NUM_SERVOS, robot::SERVO_ACCELERATION,
                                       robot::SERVO_SPEED) &&
           servoBus.setTorqueEnabled(servoIds.data(), NUM_SERVOS, true);
}

void Servos::read(gait_core::JointFeedback& feedback, uint64_t nowUs) {
    std::array<St3215State, NUM_SERVOS> fresh;
    bool received[NUM_SERVOS];
    servoBus.readStates(servoIds.data(), NUM_SERVOS, fresh.data(), received);

    int silent = 0;
    uint8_t silentId = 0;
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
        if (received[servo]) {
            states[servo] = fresh[servo];
            lastReplyUs[servo] = nowUs;
        } else if (nowUs - lastReplyUs[servo] > SILENT_WARNING_US) {
            ++silent;
            silentId = servoIds[servo];
        }
    }
    if (silent > 0 && nowUs - lastWarningUs > WARNING_PERIOD_US) {
        lastWarningUs = nowUs;
        Serial.printf("[servos] %d servo%s not replying (e.g. ID %d)\n", silent, silent == 1 ? "" : "s", silentId);
    }

    feedback.stamp = static_cast<gait_core::Timestamp>(nowUs) * 1000;
    feedback.hasEffort = true;
    for (int legNum = 0; legNum < gait_core::NUM_LEGS; ++legNum) {
        ServoDegrees degrees;
        for (int joint = 0; joint < 3; ++joint) {
            const int servo = legNum * 3 + joint;
            degrees[joint] = withoutOffset(servo, stepsToDegrees(states[servo].position));
            // The load reading as a joint torque
            feedback.effort[legNum][joint] =
                robot::JOINT_DIRECTION[joint] * states[servo].load / 1000.0f * robot::MAX_TORQUE_NM;
        }
        feedback.angles[legNum] = toJointAngles(degrees);
        feedback.present[legNum] = true;
    }
}

void Servos::write(const std::array<ServoDegrees, gait_core::NUM_LEGS>& degrees) {
    std::array<int, NUM_SERVOS> positions;
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
        positions[servo] = degreesToSteps(withOffset(servo, degrees[servo / 3][servo % 3]));
    }
    servoBus.writeGoalPositions(servoIds.data(), positions.data(), NUM_SERVOS, robot::SERVO_ACCELERATION,
                                robot::SERVO_SPEED);
}

void Servos::limp() {
    const uint8_t off = 0;
    servoBus.writeRegisters(ST3215_BROADCAST_ID, ST3215_REG_TORQUE_ENABLE, &off, 1);
}

/**
 * @note gait_core's coxa angle is anticlockwise from straight out along the leg,
 *       femur is positive lowering the femur, and tibia is 0 with the tibia in line with the femur.
 *       Every joint at 0 is each servo at its centre. See robot::JOINT_DIRECTION.
*/
ServoDegrees Servos::toServoDegrees(const gait_core::JointAngles& angles) {
    const float joints[3] = {angles.coxa, angles.femur, angles.tibia};
    ServoDegrees degrees;
    for (int joint = 0; joint < 3; ++joint) {
        degrees[joint] = robot::JOINT_DIRECTION[joint] * joints[joint] * kRadToDeg;
    }
    return degrees;
}

gait_core::JointAngles Servos::toJointAngles(const ServoDegrees& degrees) {
    float joints[3];
    for (int joint = 0; joint < 3; ++joint) {
        joints[joint] = degrees[joint] / robot::JOINT_DIRECTION[joint] * kDegToRad;
    }
    return {joints[0], joints[1], joints[2]};
}

float Servos::stepsToDegrees(int steps) {
    return (steps - ST3215_CENTER_POSITION) * 360.0f / ST3215_STEPS_PER_REV;
}

int Servos::degreesToSteps(float degrees) {
    const int steps = ST3215_CENTER_POSITION + static_cast<int>(std::lround(degrees * ST3215_STEPS_PER_REV / 360.0f));
    return std::clamp(steps, 0, ST3215_STEPS_PER_REV - 1);
}

float Servos::withOffset(int servo, float degrees) const {
    return degrees + configManager.getLegOffsets(servo / 3)[servo % 3];
}

float Servos::withoutOffset(int servo, float degrees) const {
    return degrees - configManager.getLegOffsets(servo / 3)[servo % 3];
}

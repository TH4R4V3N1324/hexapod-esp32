#include "Servos.h"

#include <algorithm>
#include <cmath>

#include "ConfigManager.h"
#include "RobotConfig.h"

namespace {

constexpr float kRadToDeg = 180.0f / gait_core::kPi;
constexpr float kDegToRad = gait_core::kPi / 180.0f;

// Replies missed in a row before a servo counts as not answering
constexpr int MISSED_REPLIES = 3;
constexpr uint64_t WARNING_PERIOD_US = 2000000;

}  // namespace

Servos::Servos() {
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
        servoIds[servo] = robot::servoId(servo / 3, servo % 3);
    }
    targets.fill(-1);
}

void Servos::begin() {
    if (robot::SERVO_ENABLE_PIN >= 0) {
        digitalWrite(robot::SERVO_ENABLE_PIN, LOW);
        pinMode(robot::SERVO_ENABLE_PIN, OUTPUT);
    } else {
        isPowered = true;  // no enable line: always on
    }
    servoBus.begin(Serial2, robot::SERVO_BAUD, robot::SERVO_RX_PIN, robot::SERVO_TX_PIN, robot::SERVO_ECHO);
}

void Servos::setPower(bool on) {
    if (robot::SERVO_ENABLE_PIN < 0 || on == isPowered) {
        return;
    }
    digitalWrite(robot::SERVO_ENABLE_PIN, on ? HIGH : LOW);
    isPowered = on;
    Serial.printf("[servos] Servo power %s\n", on ? "on" : "off");
}

void Servos::powerOnAndWait() {
    if (isPowered) {
        return;
    }
    setPower(true);
    delay(robot::SERVO_POWER_SETTLE_MS);
}

void Servos::read(gait_core::JointFeedback& feedback, uint64_t nowUs) {
    // The answering servos, plus the next silent one
    std::array<int, NUM_SERVOS> polled;
    int count = 0;
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
        if (joints[servo].answering) {
            polled[count++] = servo;
        }
    }
    for (int n = 0; n < NUM_SERVOS; ++n) {
        const int servo = (nextProbe + n) % NUM_SERVOS;
        if (!joints[servo].answering) {
            polled[count++] = servo;
            nextProbe = servo + 1;
            break;
        }
    }

    std::array<uint8_t, NUM_SERVOS> pollIds;
    for (int k = 0; k < count; ++k) {
        pollIds[k] = servoIds[polled[k]];
    }
    std::array<St3215State, NUM_SERVOS> fresh;
    bool received[NUM_SERVOS];
    servoBus.readStates(pollIds.data(), count, fresh.data(), received);

    std::array<uint8_t, NUM_SERVOS> revivedIds, lostIds;
    std::array<int, NUM_SERVOS> revivedPositions;
    int revived = 0, lost = 0, noReply = 0, errored = 0;
    uint8_t noReplyId = 0, erroredId = 0, errorBits = 0;
    for (int k = 0; k < count; ++k) {
        const int servo = polled[k];
        Joint& joint = joints[servo];
        if (!received[k]) {
            // A missed reply keeps the previous state; MISSED_REPLIES in a row and it's not answering
            if (joint.answering && ++joint.missed >= MISSED_REPLIES) {
                joint.answering = false;
                joint.holding = false;
                lostIds[lost++] = servoIds[servo];
            } else if (joint.answering) {
                ++noReply;
                noReplyId = servoIds[servo];
            }
            continue;
        }
        joint.missed = 0;
        if (!joint.answering) {
            // Powered up: hold it where it is, and ignore the target it had until a new one arrives
            joint.answering = true;
            joint.holding = true;
            joint.heldTarget = targets[servo];
            revivedIds[revived] = servoIds[servo];
            revivedPositions[revived++] = fresh[k].position;
        }
        if (fresh[k].error != 0) {
            ++errored;
            erroredId = servoIds[servo];
            errorBits = fresh[k].error;
        }
        states[servo] = fresh[k];
    }

    if (lost > 0) {
        Serial.printf("[servos] %d servo%s stopped answering (servo power off?):", lost, lost == 1 ? "" : "s");
        for (int i = 0; i < lost; ++i) {
            Serial.printf(" %d", lostIds[i]);
        }
        Serial.println();
    }
    if (revived > 0) {
        if (servoBus.writeGoalPositions(revivedIds.data(), revivedPositions.data(), revived, robot::SERVO_ACCELERATION,
                                        robot::SERVO_SPEED) &&
            servoBus.setTorqueEnabled(revivedIds.data(), revived, true)) {
            Serial.printf("[servos] %d servo%s answering, held in place with torque on (%d of %d)\n", revived,
                          revived == 1 ? "" : "s", answeringCount(), NUM_SERVOS);
        } else {
            Serial.printf("[servos] Failed to hold %d servo(s) that started answering\n", revived);
        }
    }
    if (noReply > 0 && nowUs - lastNoReplyWarningUs > WARNING_PERIOD_US) {
        lastNoReplyWarningUs = nowUs;
        Serial.printf("[servos] No reply from %d servo%s (e.g. ID %d)\n", noReply, noReply == 1 ? "" : "s", noReplyId);
    }
    if (errored > 0 && nowUs - lastErrorWarningUs > WARNING_PERIOD_US) {
        lastErrorWarningUs = nowUs;
        Serial.printf("[servos] %d servo%s reporting an error (e.g. ID %d: 0x%02X)\n", errored,
                      errored == 1 ? "" : "s", erroredId, errorBits);
    }

    // A leg is present only while all three of its servos are answering
    feedback.stamp = static_cast<gait_core::Timestamp>(nowUs) * 1000;
    feedback.hasEffort = true;
    for (int legNum = 0; legNum < gait_core::NUM_LEGS; ++legNum) {
        feedback.present[legNum] = joints[legNum * 3].answering && joints[legNum * 3 + 1].answering &&
                                   joints[legNum * 3 + 2].answering;
        if (!feedback.present[legNum]) {
            continue;
        }
        ServoDegrees degrees;
        for (int joint = 0; joint < 3; ++joint) {
            const int servo = legNum * 3 + joint;
            degrees[joint] = withoutOffset(servo, stepsToDegrees(states[servo].position));
            // The load reading as a joint torque
            feedback.effort[legNum][joint] =
                robot::JOINT_DIRECTION[joint] * states[servo].load / 1000.0f * robot::MAX_TORQUE_NM;
        }
        feedback.angles[legNum] = toJointAngles(degrees);
    }
}

void Servos::write(const std::array<ServoDegrees, gait_core::NUM_LEGS>& degrees) {
    std::array<uint8_t, NUM_SERVOS> writeIds;
    std::array<int, NUM_SERVOS> positions;
    int count = 0;
    for (int servo = 0; servo < NUM_SERVOS; ++servo) {
        const int target = degreesToSteps(withOffset(servo, degrees[servo / 3][servo % 3]));
        targets[servo] = target;
        Joint& joint = joints[servo];
        if (joint.holding) {
            if (target == joint.heldTarget) {
                continue;
            }
            joint.holding = false;
        }
        if (!joint.answering) {
            continue;
        }
        writeIds[count] = servoIds[servo];
        positions[count++] = target;
    }
    if (count > 0) {
        servoBus.writeGoalPositions(writeIds.data(), positions.data(), count, robot::SERVO_ACCELERATION,
                                    robot::SERVO_SPEED);
    }
}

void Servos::limp() {
    const uint8_t off = 0;
    servoBus.writeRegisters(ST3215_BROADCAST_ID, ST3215_REG_TORQUE_ENABLE, &off, 1);
}

int Servos::answeringCount() const {
    return static_cast<int>(std::count_if(joints.begin(), joints.end(), [](const Joint& j) { return j.answering; }));
}

/**
 * @note gait_core's coxa angle is anticlockwise from straight out along the leg, femur is positive
 *       lowering the femur, and tibia is 0 with the tibia in line with the femur. Every joint at 0 is
 *       each servo at its centre. See robot::JOINT_DIRECTION.
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

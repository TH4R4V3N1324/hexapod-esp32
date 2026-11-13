#include "GaitController.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "ConfigManager.h"
#include "RobotConfig.h"

using gait_core::Scalar;
using gait_core::NUM_LEGS;

// The wire values of Gait and Mode are passed straight to the engine
static_assert(static_cast<int>(::GAIT_TRIPOD) == gait_core::GAIT_TRIPOD &&
              static_cast<int>(::GAIT_RIPPLE) == gait_core::GAIT_RIPPLE &&
              static_cast<int>(::GAIT_WAVE) == gait_core::GAIT_WAVE &&
              static_cast<int>(::NUM_GAITS) == gait_core::NUM_GAITS, "Gait values must match gait_core");
static_assert(static_cast<int>(::MODE_NORMAL) == gait_core::MODE_NORMAL &&
              static_cast<int>(::MODE_STRAFE) == gait_core::MODE_STRAFE &&
              static_cast<int>(::MODE_TILT) == gait_core::MODE_TILT &&
              static_cast<int>(::MODE_CONFIG) == gait_core::NUM_MODES, "Mode values must match gait_core");

namespace {

/**
 * @brief A joystick value as -1 to 1, with the dead zone removed
*/
Scalar stick(int16_t value) {
    if (std::abs(value) <= robot::JOYSTICK_DEADZONE) {
        return 0;
    }
    return std::clamp(static_cast<Scalar>(value) / robot::JOYSTICK_FULL_SCALE, Scalar(-1), Scalar(1));
}

const char* levelName(gait_core::LogLevel level) {
    switch (level) {
        case gait_core::LogLevel::Debug: return "DEBUG";
        case gait_core::LogLevel::Info: return "INFO";
        case gait_core::LogLevel::Warn: return "WARN";
    }
    return "";
}

}  // namespace

GaitController::GaitController(Servos& servos)
    : servos(servos), engine(robot::gaitParams(), robot::geometry()) {
    engine.log = [](gait_core::LogLevel level, const char* message) {
        Serial.printf("[gait %s] %s\n", levelName(level), message);
    };
    engine.seatedChanged = [](bool seated) {
        Serial.printf("[gait] %s\n", seated ? "seated" : "standing");
    };
}

bool GaitController::begin(uint64_t nowUs) {
    if (!servos.holdCurrentPose(servoOutput)) {
        return false;
    }
    // From the measured pose, the engine's startup moves the legs to the seated pose, then stands up
    servos.read(feedback, nowUs);
    engine.onJointStates(feedback);
    return true;
}

void GaitController::handleCommand(const ControlPacket& packet) {
    const int16_t* args = packet.commandArgs;
    switch (packet.command) {
        case CMD_SET_GAIT:
            if (args[0] >= 0 && args[0] < NUM_GAITS) {
                engine.setGait(static_cast<uint8_t>(args[0]));
            } else {
                Serial.printf("Ignored invalid gait %d\n", args[0]);
            }
            break;

        case CMD_SET_MODE:
            if (args[0] == MODE_CONFIG) {
                if (!configMode) {
                    // Out of tilt mode first, so the engine eases the body back to neutral meanwhile
                    engine.setMode(gait_core::MODE_NORMAL);
                    configMode = true;
                    blending = true;
                }
            } else if (args[0] >= 0 && args[0] < MODE_CONFIG) {
                if (engine.setMode(static_cast<uint8_t>(args[0])).success && configMode) {
                    configMode = false;
                    blending = true;
                }
            } else {
                Serial.printf("Ignored invalid mode %d\n", args[0]);
            }
            break;

        case CMD_SET_CONFIG:
            if (args[0] >= 0 && args[0] < NUM_LEGS && args[1] >= 0 && args[1] < 3) {
                configManager.SetLegConfig(args[0], args[1], args[2]);
            } else {
                Serial.printf("Ignored calibration for leg %d joint %d\n", args[0], args[1]);
            }
            break;

        case CMD_HOME_STANCE:
            engine.sit(false);
            break;

        case CMD_REQUEST_CONFIG:
            if (args[0] >= 0 && args[0] < NUM_LEGS) {
                requestedConfigLeg = args[0];
            }
            break;

        case CMD_SIT:
            engine.sit(args[0] != 0);
            break;

        case CMD_NONE:
        default:
            break;
    }
}

/**
 * @brief Turn the sticks into a velocity command (or a body pose in tilt mode)
 * @note Sticks: X positive is right, Y positive is forward. Left stick drives, right stick X turns,
 *       In normal mode the engine turns with the left stick's
 *       X instead of strafing. In tilt mode the left stick shifts the body and the right stick
 *       rolls and pitches it.
*/
void GaitController::applySticks(const ControlPacket& packet, bool commsOk) {
    Scalar lx = 0, ly = 0, rx = 0, ry = 0;
    if (commsOk && !configMode) {
        lx = stick(packet.joystick1X);
        ly = stick(packet.joystick1Y);
        rx = stick(packet.joystick2X);
        ry = stick(packet.joystick2Y);
    }

    gait_core::VelocityCommand command{};
    if (engine.gaitConfig().currentMode == gait_core::MODE_TILT) {
        const gait_core::GaitParams& p = engine.params();
        engine.setBodyPose(rx * p.tilt_max_angle,     // stick right lowers the right side
                           ry * p.tilt_max_angle,     // stick forward lowers the nose
                           0,
                           gait_core::Vec3(ly, -lx, 0) * p.tilt_max_translation);
    } else {
        // Full stick is the gait's top speed
        const Scalar maxLinear = engine.maxFootSpeed();
        const Scalar maxAngular = maxLinear / engine.trajectory().maxBodyRadius();
        command.vx = ly * maxLinear;
        command.vy = -lx * maxLinear;
        command.wz = -rx * maxAngular;
    }

    // Smoothed; a stop goes straight through
    if (command.vx == 0 && command.vy == 0 && command.wz == 0) {
        filteredVelocity = command;
    } else {
        const Scalar a = robot::VELOCITY_SMOOTHING;
        filteredVelocity.vx += a * (command.vx - filteredVelocity.vx);
        filteredVelocity.vy += a * (command.vy - filteredVelocity.vy);
        filteredVelocity.wz += a * (command.wz - filteredVelocity.wz);
    }
    engine.setVelocity(filteredVelocity);
}

/**
 * @brief Body height from the controller (mm), passed on whenever it changes
 * @note 0 means not set. Asked again once standing if it changed while seated, since the engine
 *       refuses height changes while sitting. Out of range values are refused by the engine.
*/
void GaitController::applyHeight(int16_t heightMm) {
    if (heightMm == lastHeightRequest || heightMm <= 0 || !engine.initialized() || engine.seated()) {
        return;
    }
    lastHeightRequest = heightMm;
    engine.setHeight(-static_cast<Scalar>(heightMm) / 1000);
}

void GaitController::update(Scalar dt, uint64_t nowUs, const ControlPacket& packet, bool commsOk) {
    servos.read(feedback, nowUs);
    engine.onJointStates(feedback);

    applySticks(packet, commsOk);
    if (commsOk) {
        applyHeight(packet.currentHeight);
    }
    const gait_core::GaitOutput output = engine.update(dt, static_cast<gait_core::Timestamp>(nowUs) * 1000);
    writeServos(output, dt);
}

void GaitController::onImu(float roll, float pitch, float yaw, uint64_t nowUs) {
    gait_core::ImuSample sample;
    sample.roll = roll;
    sample.pitch = pitch;
    sample.yaw = yaw;
    sample.stamp = static_cast<gait_core::Timestamp>(nowUs) * 1000;
    engine.setImu(sample);
}

/**
 * @brief Write the engine's joint targets, or the calibration pose in MODE_CONFIG
 * @note Moving into or out of the calibration pose, the servos slew there at CONFIG_SLEW_DEG_PER_S
 *       rather than jumping. Before the engine's first output the servos hold where they are.
*/
void GaitController::writeServos(const gait_core::GaitOutput& output, Scalar dt) {
    std::array<ServoDegrees, NUM_LEGS> target = servoOutput;
    if (configMode) {
        for (auto& leg : target) {
            leg = {0, 0, 0};
        }
    } else if (output.commanded) {
        for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
            target[legNum] = Servos::toServoDegrees(output.joints[legNum]);
        }
    }

    if (blending) {
        const float maxStep = robot::CONFIG_SLEW_DEG_PER_S * dt;
        bool arrived = true;
        for (int legNum = 0; legNum < NUM_LEGS; ++legNum) {
            for (int joint = 0; joint < 3; ++joint) {
                float& value = servoOutput[legNum][joint];
                const float error = target[legNum][joint] - value;
                value += std::clamp(error, -maxStep, maxStep);
                arrived = arrived && std::abs(error) <= maxStep;
            }
        }
        blending = !arrived;
    } else {
        servoOutput = target;
    }

    servos.write(servoOutput);
}

void GaitController::fillTelemetry(HexPacket& packet) const {
    const gait_core::GaitConfig& config = engine.gaitConfig();
    memcpy(packet.legConfigs, configManager.getLegOffsets(requestedConfigLeg), sizeof(packet.legConfigs));
    packet.currentHeight = static_cast<int16_t>(std::lround(-config.currentHeight * 1000));
    packet.currentPhase = 0;  // GaitEngine doesn't expose its phase
    packet.currentGait = static_cast<Gait>(config.gaitChangeRequested ? config.pendingGait : config.currentGait);
    packet.currentMode = configMode ? MODE_CONFIG : static_cast<Mode>(config.currentMode);
}

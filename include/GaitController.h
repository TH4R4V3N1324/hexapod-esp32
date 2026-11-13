#ifndef _GAIT_CONTROLLER_H_
#define _GAIT_CONTROLLER_H_

#include <array>
#include <cstdint>

#include "gait_core/gait_engine.hpp"

#include "DataPacket.h"
#include "Servos.h"

/**
 * @brief ESP32 wrapper around gait_core::GaitEngine
 * @note The gait itself (startup, walking, sitting, tilt mode, gait changes, idle return, leveling,
 *       load balancing, terrain adaptation) is in hexapod-gait-core. This class turns the
 *       controller's packets into engine inputs, feeds it the servos' measured positions and loads,
 *       ticks it, and writes its joint targets to the servos. It also owns MODE_CONFIG, the
 *       calibration pose.
 *
 *       The ST3215s report where they are and how hard they're pushing, so the engine runs
 *       closed loop: it starts from the measured pose and gets the foot forces it needs to feel
 *       the ground.
*/
class GaitController {
public:
    explicit GaitController(Servos& servos);

    /**
     * @brief Act on a command from the controller (call when it changes)
    */
    void handleCommand(const ControlPacket& packet);

    /**
     * @brief Read the servos, advance the gait one tick and move the servos
     * @param[in] dt Time since the last tick (s)
     * @param[in] nowUs The current time (us since boot)
     * @param[in] packet The latest controller input
     * @param[in] commsOk false once the controller has gone quiet: the sticks count as centred
    */
    void update(gait_core::Scalar dt, uint64_t nowUs, const ControlPacket& packet, bool commsOk);

    /**
     * @brief Pass the body's orientation from the IMU to the engine (rad)
     * @note The engine ignores it once it's 0.2 s old, so leveling stops if the IMU does
    */
    void onImu(float roll, float pitch, float yaw, uint64_t nowUs);

    /**
     * @brief Fill in the telemetry sent back to the controller
    */
    void fillTelemetry(HexPacket& packet) const;

    /**
     * @brief Sit down, as CMD_SIT 1 does (for a low battery)
    */
    gait_core::CommandResult sitDown() { return engine.sit(true); }
    /**
     * @brief Sit down, then turn the servo power off once seated (CMD_POWER_DOWN). A stand command
     *        powers them up again.
    */
    gait_core::CommandResult powerDown();

    const gait_core::GaitEngine& gait() const { return engine; }

private:
    Servos& servos;
    gait_core::GaitEngine engine;
    gait_core::JointFeedback feedback;

    gait_core::VelocityCommand filteredVelocity{};

    // Servo angles last written to each leg (deg, before calibration offsets)
    std::array<ServoDegrees, gait_core::NUM_LEGS> servoOutput{};

    bool configMode = false;    // MODE_CONFIG: every joint held at 0 for calibration
    bool blending = false;      // slewing between the calibration pose and the engine's output
    int16_t lastHeightRequest = 0;
    int requestedConfigLeg = 0; // leg whose calibration offsets are reported back (CMD_REQUEST_CONFIG)

    void applySticks(const ControlPacket& packet, bool commsOk);
    void applyHeight(int16_t heightMm);
    void writeServos(const gait_core::GaitOutput& output, gait_core::Scalar dt);
};

#endif

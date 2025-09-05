#include "Animation.h"

//Returns leg configuration based on given gate
std::vector<std::vector<int>> Animation::GetLegConfig(Gait gait){
    switch (gait){
        case GAIT_TRIPOD:
            return {{1, 3, 5}, {2, 4, 6}};
            break;
        case GAIT_RIPPLE:
            return {{3, 6}, {2, 4}, {1, 5}};
            break;
        case GAIT_WAVE:
            return {{3}, {2}, {1}, {4}, {5}, {6}};
            break;
        default:
            return {};
            break;
    }
}

/*
@brief Changes to the next gait when called
*/
void Animation::CycleGait(){
    pendingGait = static_cast<Gait>((hexPacket.currentGait + 1) % NUM_GAITS);
    gaitChangeRequested = true;
}

/*
@brief Sets the current gait to the specified gait
*/
void Animation::SetGait(Gait gait) {
    pendingGait = gait;
    gaitChangeRequested = true;
}

/*
@brief Changes to the next mode when called
*/
void Animation::CycleMode() {
    hexPacket.currentMode = static_cast<Mode>((hexPacket.currentMode + 1) % NUM_MODES);
}

/*
@brief Sets the current mode to the specified mode
*/
void Animation::SetMode(Mode mode) {
    hexPacket.currentMode = mode;
}

/*
@brief Set the height of the hexapod and update start positions accordingly
@param newHeight The new height to set
*/
void Animation::SetHeight(double newHeight) {
    hexPacket.currentHeight = newHeight;
    startPos = Vector3{0, 130, -static_cast<double>(hexPacket.currentHeight)};
    startPosition = {
        {1, startPos.rotate(-15)},
        {2, startPos},
        {3, startPos.rotate(15)},
        {4, startPos.rotate(15)},
        {5, startPos},
        {6, startPos.rotate(-15)}
    };
}

/*
@brief Move all legs to the home position and deactivate servos
*/
void Animation::Shutdown() {
    for(size_t i = 1; i <= 6; ++i){
        move.Position(homePos, i);
        delay(1000);
        move.Deactivate(i);
    }
}

/*
@brief Startup sequence to move legs to start position
*/
void Animation::Startup() {
    static bool initialized = false;
    int resolution = 50;

    if(!initialized){
        Shutdown();
        initialized = true;
    }

    // Reset phase to 0 when starting up
    hexPacket.currentPhase = 0;

    // Update start position based on current height
    startPos = Vector3{0, 130, -static_cast<double>(hexPacket.currentHeight)};
    startPosition = {
        {1, startPos.rotate(-15)},
        {2, startPos},
        {3, startPos.rotate(15)},
        {4, startPos.rotate(15)},
        {5, startPos},
        {6, startPos.rotate(-15)}
    };

    // Generate trajectories for each leg to move to home position
    std::array<std::array<Vector3, MAX_RESOLUTION>, MAX_LEGS + 1> trajectory;
    std::array<int, MAX_LEGS + 1> sizes{};
    for (int leg = 1; leg <= MAX_LEGS; ++leg) {
        cal.GenerateStraightTrajectory(trajectory[leg].data(), sizes[leg], homePos, startPosition.at(leg), resolution);
    }
    for (int step = 0; step < resolution; ++step) {
        for (int leg = 1; leg <= MAX_LEGS; ++leg) {
            if (step < sizes[leg]) {
                move.Position(trajectory[leg][step], leg);
            }
        }
    }
}

/*
@brief Check if there has been a significant change in joystick position
@param stickX The current X position of the joystick
@param stickY The current Y position of the joystick
@return true if the change is significant, false otherwise
*/
bool SignificantStickChange(const int& stickX, const int& stickY) {
    static int lastX = 0;
    static int lastY = 0;

    // Check if the change in stick position is significant
    bool significantChange = (std::abs(stickX - lastX) > 10 || std::abs(stickY - lastY) > 10);

    // Update the last known positions
    lastX = stickX;
    lastY = stickY;

    return significantChange;
}

/*
@brief Blend target positions from forward and rotation inputs
@param currentPos The current position of the leg
@param forwardPos The target position based on forward input
@param rotationPos The target position based on rotation input
@return The blended target position
*/
Vector3 Animation::BlendTargetPosition(const Vector3& currentPos, const Vector3& forwardPos, const Vector3& rotationPos) {
    // Compute deltas from current position
    Vector3 forwardDelta = forwardPos - currentPos;
    Vector3 rotationDelta = rotationPos - currentPos;

    // Add the deltas
    Vector3 blended = currentPos + forwardDelta + rotationDelta;

    // Optionally, clamp the stride to a maximum distance from currentPos if needed
    double maxStride = 100.0;
    if ((blended - currentPos).length() > maxStride) {
         blended = currentPos + (blended - currentPos).normalized() * maxStride;
    }

    return blended;
}

/*
@brief Handle idle/return-to-start logic
@return true if the stick is idle, false otherwise
*/
bool Animation::HandleIdleReturn() {
    static int idleCount = 0;
    static const int idleThreshold = 100;

    // Check if stick is idle
    bool stickIdle = (std::abs(controlPacket.joystick1X) <= 10 && std::abs(controlPacket.joystick1Y) <= 10);
    if (stickIdle) idleCount++;
    else idleCount = 0;

    // Handle idle/return-to-start logic
    if (idleCount > idleThreshold || gaitState.idleReturning) {
        if (!gaitState.idleReturning) {
            gaitState.idleReturning = true;
            gaitState.step = 0;
        }
        returnToStart();
        idleCount = 0;
    }
    return stickIdle; // Return whether stick is idle
}

/*
@brief Ensure the gait configuration is set up correctly
*/
void Animation::EnsureGaitConfig() {
    if (gaitState.config.empty()) {
        gaitState.config = GetLegConfig(hexPacket.currentGait);
    }
}

/*
@brief Calculate stride multiplier based on gait configuration
@return The stride multiplier
*/
double Animation::CalculateStrideMultiplier() {
    return (gaitState.config.size() > 1) ? 1.0 / (gaitState.config.size() - 1) : 1.0;
}

/*
@brief Generate swing and stance trajectories for the legs
@param liftHeight The height to lift the legs during swing
@param resolution The number of steps in the trajectory
@param swingTargetFunc A function to compute the swing target position for a leg
@param stanceTargetFunc A function to compute the stance target position for a leg
*/
void Animation::GenerateTrajectories(
    int liftHeight,
    int resolution,
    std::function<Vector3(int, const Vector3&)> swingTargetFunc,
    std::function<Vector3(int, const Vector3&)> stanceTargetFunc
) {
    // Assign legs to their respective swing and stance groups
    auto swingGroup = gaitState.config[hexPacket.currentPhase];
    std::vector<int> stanceGroup;
    for (int idx = 0; idx < gaitState.config.size(); ++idx) {
        if (idx == hexPacket.currentPhase) continue;
        for (int legNum : gaitState.config[idx])
            stanceGroup.push_back(legNum);
    }
    for (int i = 1; i <= MAX_LEGS; ++i) {
        gaitState.swingSizes[i] = 0;
        gaitState.stanceSizes[i] = 0;
    }

    std::map<int, Vector3> swingTargetsBodyFrame;
    std::map<int, Vector3> stanceTargetsBodyFrame;

    // Calculate swing and stance targets in body frame
    for (int legNum : swingGroup) {
        Vector3 currentPos = move.GetLegPosition(legNum);
        Vector3 targetLegFrame = swingTargetFunc(legNum, currentPos);
        Vector3 targetBodyFrame = cal.convertToBodyFrame(targetLegFrame, legNum);
        swingTargetsBodyFrame[legNum] = targetBodyFrame;
    }
    for (int legNum : stanceGroup) {
        Vector3 currentPos = move.GetLegPosition(legNum);
        Vector3 targetLegFrame = stanceTargetFunc(legNum, currentPos);
        Vector3 targetBodyFrame = cal.convertToBodyFrame(targetLegFrame, legNum);
        stanceTargetsBodyFrame[legNum] = targetBodyFrame;
    }

    // Collision check and adjustment
    double threshold = 50.0; // mm
    for (int legNum : swingGroup) {
        Vector3 swingTargetBody = swingTargetsBodyFrame[legNum];
        for (const auto& [stanceNum, stanceTargetBody] : stanceTargetsBodyFrame) {
            if ((swingTargetBody - stanceTargetBody).length() < threshold) {
                // Clamp swingTargetBody outward
                Vector3 dir = (swingTargetBody - stanceTargetBody).normalized();
                swingTargetBody = stanceTargetBody + dir * threshold;
            }
        }
        // Convert back to leg frame
        Vector3 targetLegFrame = cal.convertToLegFrame(swingTargetBody, legNum);
        int size = 0;
        cal.GenerateBezierTrajectory(
            gaitState.swingTrajectory[legNum].data(),
            size,
            move.GetLegPosition(legNum),
            targetLegFrame,
            liftHeight,
            resolution
        );
        gaitState.swingSizes[legNum] = size;
    }

    // Stance
    for (int legNum : stanceGroup) {
        Vector3 targetLegFrame = cal.convertToLegFrame(stanceTargetsBodyFrame[legNum], legNum);
        int size = 0;
        cal.GenerateStraightTrajectory(
            gaitState.stanceTrajectory[legNum].data(),
            size,
            move.GetLegPosition(legNum),
            targetLegFrame,
            resolution
        );
        gaitState.stanceSizes[legNum] = size;
    }
}

/*
@brief Perform a leg step based on the current trajectories
@param stickIdle Whether the joystick is idle
@param resolution The resolution of the trajectories
@param handlePhaseTransition Whether to handle phase transitions
@note If stickIdle is true, the step will not advance
@note If handlePhaseTransition is false, phase transitions will be skipped and must be handled externally
*/
void Animation::PerformLegStep(bool stickIdle, int resolution, bool handlePhaseTransition) {
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        int swingSize = gaitState.swingSizes[legNum];
        int stanceSize = gaitState.stanceSizes[legNum];
        if (gaitState.step < swingSize)
            move.Position(gaitState.swingTrajectory[legNum][gaitState.step], legNum);
        if (gaitState.step < stanceSize)
            move.Position(gaitState.stanceTrajectory[legNum][gaitState.step], legNum);
    }
    // Advance step if not idle
    if (!stickIdle) gaitState.step++;
    if (!handlePhaseTransition) return;

    // Phase transition
    if (gaitState.step > resolution) {
        hexPacket.currentPhase = (hexPacket.currentPhase + 1) % gaitState.config.size();
        gaitState.step = 0;
    }    
}

/*
@brief Returns the hexapod to its start position
*/
void Animation::returnToStart() {
    static int counter = 0;
    static bool trajectoryGenerated = false;
    int liftHeight = 50;
    int resolution = 50;

    // Safety check: phase must be valid
    if (hexPacket.currentPhase >= gaitState.config.size()) {
        std::cerr << "[returnToStart] ERROR: Invalid hexPacket.currentPhase: "
                  << hexPacket.currentPhase << ", config size: "
                  << gaitState.config.size() << std::endl;
        gaitState.idleReturning = false;
        counter = 0;
        gaitState.step = 0;
        trajectoryGenerated = false;
        return;
    }

    if (gaitState.step == 0 && !trajectoryGenerated) {
        GenerateTrajectories(
            liftHeight,
            resolution,
            // Swing: move to start position
            [this](int legNum, const Vector3& currentPos) {
                auto it = startPosition.find(legNum);
                return (it != startPosition.end()) ? it->second : currentPos;
            },
            // Stance: hold current position
            [](int, const Vector3& currentPos) {
                return currentPos;
            }
        );
    }

    // Move all legs for this step
    PerformLegStep(false, resolution, false);

    // Phase transition
    if (gaitState.step > resolution) {
        counter++;
        gaitState.step = 0;
        hexPacket.currentPhase = (hexPacket.currentPhase + 1) % gaitState.config.size();

        // After all phases, finish return-to-start and handle gait change if requested
        if (counter > gaitState.config.size()) {
            counter = 0;
            gaitState.idleReturning = false;
            trajectoryGenerated = false;

            if (gaitChangeRequested) {
                hexPacket.currentGait = pendingGait;
                gaitState.config = GetLegConfig(hexPacket.currentGait);
                hexPacket.currentPhase = 0;
                gaitState.step = 0;
                gaitChangeRequested = false;
            }
        }
    }
}

/*
@brief Handles the configuration state for the hexapod
*/
void Animation::ConfigState() {
    JointAngles angles = {0, 0, 0}; // Default angles for configuration state
    int resolution = 50;
    std::array<std::array<Vector3, MAX_RESOLUTION>, MAX_LEGS + 1> trajectory;
    std::array<int, MAX_LEGS + 1> sizes{};

    // Generate trajectories for each leg
    for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
        Vector3 currentPos = move.GetLegPosition(legNum);
        Vector3 targetPos = cal.position(angles, legNum);
        cal.GenerateStraightTrajectory(trajectory[legNum].data(), sizes[legNum], currentPos, targetPos, resolution);
    }

    // Interpolate movement
    for (int step = 0; step < resolution; ++step) {
        for (int legNum = 1; legNum <= MAX_LEGS; ++legNum) {
            if (step < sizes[legNum]) {
                move.Position(trajectory[legNum][step], legNum);
            }
        }
    }
}

/*
@brief Handles the strafing motion of the hexapod
@note This function assumes that the joystick inputs are mapped such that:
      - Left joystick Y controls lateral movement (left/right)
      - Left joystick X controls forward/backward movement
      - Right joystick X controls rotation (turning)
*/
void Animation::Strafe() {
    int liftHeight = 50;
    int resolution = 50;

    // Check if stick is idle
    bool stickIdle = HandleIdleReturn();
    if (gaitState.idleReturning) return;

    // Ensure gait config is set
    EnsureGaitConfig();

    // Calculate stride multiplier safely
    double strideMultiplier = CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (gaitState.step == 0) {
        GenerateTrajectories(
        liftHeight,
        resolution,
        // Swing target
        [this](int legNum, const Vector3& currentPos) {
            Vector3 directionPos = cal.direction(controlPacket.joystick1X, controlPacket.joystick1Y, currentPos, legNum);
            Vector3 rotationPos = cal.direction(controlPacket.joystick2X, 0, currentPos, legNum, false, 1.0, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, directionPos, rotationPos);
            return targetPos;
        },
        // Stance target
        [this, strideMultiplier](int legNum, const Vector3& currentPos) {
            Vector3 directionPos = cal.direction(controlPacket.joystick1X, controlPacket.joystick1Y, currentPos, legNum, true, strideMultiplier);
            Vector3 rotationPos = cal.direction(controlPacket.joystick2X, 0, currentPos, legNum, false, strideMultiplier, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, directionPos, rotationPos);
            return targetPos;
        }
        );
    }
    // Move all legs for this step
    PerformLegStep(stickIdle, resolution);
}

/*
@brief Handles the normal walking motion of the hexapod
@note This function assumes that the joystick inputs are mapped such that:
      - Left joystick Y controls forward/backward movement
      - Left joystick X controls turning (left/right)
*/
void Animation::Normal() {
    int liftHeight = 50;
    int resolution = 50;

    // Check if stick is idle
    bool stickIdle = HandleIdleReturn();
    if (gaitState.idleReturning) return;

    // Ensure gait config is set
    EnsureGaitConfig();

    // Calculate stride multiplier safely
    double strideMultiplier = CalculateStrideMultiplier();

    // Generate trajectories at the start of each phase
    if (gaitState.step == 0) {
        GenerateTrajectories(
        liftHeight,
        resolution,
        // Swing target
        [this](int legNum, const Vector3& currentPos) {
            Vector3 forwardPos = cal.direction(0, controlPacket.joystick1Y, currentPos, legNum);
            Vector3 rotationPos = cal.direction(controlPacket.joystick1X, 0, currentPos, legNum, false, 1.0, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, forwardPos, rotationPos);
            return targetPos;
        },
        // Stance target
        [this, strideMultiplier](int legNum, const Vector3& currentPos) {
            Vector3 forwardPos = cal.direction(0, controlPacket.joystick1Y, currentPos, legNum, true , strideMultiplier);
            Vector3 rotationPos = cal.direction(controlPacket.joystick1X, 0, currentPos, legNum, true, strideMultiplier, false);
            Vector3 targetPos = BlendTargetPosition(currentPos, forwardPos, rotationPos);
            return targetPos;
        }
        );
    }
    // Move all legs for this step
    PerformLegStep(stickIdle, resolution);
} 
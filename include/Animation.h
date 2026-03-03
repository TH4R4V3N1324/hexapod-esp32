#ifndef _ANIMATION_H_
#define _ANIMATION_H_
#include "Move.h"
#include "Calculate.h"
#include "DataPacket.h"
#include <stdio.h>
#include <array>
#include <functional>

static constexpr int MAX_LEGS = 6;
static constexpr int MAX_RESOLUTION = 50 + 1; // +1 for inclusive endpoint

struct GaitState {
    std::vector<std::vector<int>> config;
    std::array<std::array<Vector3, MAX_RESOLUTION>, MAX_LEGS + 1> swingTrajectory;  // 1-based indexing
    std::array<std::array<Vector3, MAX_RESOLUTION>, MAX_LEGS + 1> stanceTrajectory;
    std::array<int, MAX_LEGS + 1> swingSizes{};   // Store actual size for each leg
    std::array<int, MAX_LEGS + 1> stanceSizes{};
    int step = 0;
    bool idleReturning = false;
};

class Animation{
private:
    Vector3 homePos {0, 150, 0};
    Vector3 startPos {0, 130, -static_cast<double>(hexPacket.currentHeight)};
    std::map<int, Vector3> startPosition{
        {1, startPos.rotate(-15)},
        {2, startPos},
        {3, startPos.rotate(15)},
        {4, startPos.rotate(15)},
        {5, startPos},
        {6, startPos.rotate(-15)}
    };
    Move move;
    Calculate cal;
    GaitState gaitState;
    bool firstStepAfterRest = true;
    bool gaitChangeRequested = false;
    Vector3 BlendTargetPosition(const Vector3& currentPos, const Vector3& forwardPos, const Vector3& rotationPos);
    bool HandleIdleReturn();
    void EnsureGaitConfig();
    double CalculateStrideMultiplier();
    void GenerateTrajectories(
        int liftHeight,
        int resolution,
        std::function<Vector3(int, const Vector3&)> swingTargetFunc,
        std::function<Vector3(int, const Vector3&)> stanceTargetFunc
    );
    void PerformLegStep(bool stickIdle, int resolution, bool handlePhaseTransition = true);

public:
    Gait pendingGait;
    std::vector<std::vector<int>> GetLegConfig(Gait gait);
    void CycleGait();
    void SetGait(Gait gait);
    void CycleMode();
    void SetMode(Mode mode);
    void SetHeight(double newHeight);
    void Startup();
    void Shutdown();
    void Strafe();
    void returnToStart();
    void Normal();
    void ConfigState();
};

#endif 
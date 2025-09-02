#ifndef _CALCULATE_H_
#define _CALCULATE_H_
#include <vector>
#include <cmath>
#include <map>
#include <unordered_map>
#include <iostream>
#include "DataPacket.h"
#include "ConfigManager.h"

struct Vector3 {
    double x;
    double y;
    double z;

    Vector3(double x_ = 0, double y_ = 0, double z_ = 0)
        : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    Vector3 operator-(const Vector3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    Vector3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    Vector3 operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }
    double length() const {
        return sqrt(x * x + y * y + z * z);
    }
    Vector3 normalized() const {
        double len = length();
        if (len < 1e-8) return {0, 0, 0};
        return *this / len;
    }
    Vector3 rotate(double angleDeg) const {
        double angleRad = angleDeg * M_PI / 180;
        double cosA = cos(angleRad);
        double sinA = sin(angleRad);
        return {
            x * cosA - y * sinA,
            x * sinA + y * cosA,
            z
        };
    }
    bool operator==(const Vector3& other) const {
        const double epsilon = 1e-6;
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }
};

struct LegConfig {
    double rotationAngle;
    bool isMirrored;
    Vector3 translationOffset;
};

struct JointAngles {
    double coxaAngle;
    double femurAngle;
    double tibiaAngle;
};

struct LegPosition {
    Vector3 position;
    JointAngles angles;
};

class Calculate {
private:
    static constexpr double coxaLength = 50.50;
    static constexpr double femurLength= 90;
    static constexpr double tibiaLength= 150.35;
    const double pi = acos(-1.0);

public:
    static std::unordered_map<int, LegPosition> legPosition;
    void GenerateArcTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int liftHeight, int resolution, bool invert = false);
    void GenerateStraightTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int resolution);
    void GenerateBezierTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int liftHeight, int resolution, bool invert = false);
    JointAngles angle(Vector3 position, int legNum);
    Vector3 position(JointAngles angles, int legNum);
    Vector3 direction(const int16_t& joystickValueX, const int16_t& joystickValueY, const Vector3& start, int legNum, bool invert = false, double strideMultiplier = 1.0, bool useBodyFrame = true);
    Vector3 convertToBodyFrame(const Vector3& position, int legNum);
    Vector3 convertToLegFrame(const Vector3& position, int legNum);
    JointAngles ApplyOffsets(JointAngles& angles, int legNum);
    std::map<int, LegConfig> legConfigs;

    Calculate() {
        legConfigs[1] = {30 * pi / 180, false, Vector3(67.33, 84.6, 0)};
        legConfigs[2] = {0, false, Vector3(90, 0, 0)};
        legConfigs[3] = {-30 * pi / 180, false, Vector3(67.33, -84.6, 0)};
        legConfigs[4] = {-30 * pi / 180, true, Vector3(-67.33, -84.6, 0)};
        legConfigs[5] = {0, true, Vector3(-90, 0, 0)};
        legConfigs[6] = {30 * pi / 180, true, Vector3(-67.33, 84.6, 0)};
    }
};

#endif 
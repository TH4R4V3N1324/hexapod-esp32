#include "Calculate.h"
using namespace std;

unordered_map<int, LegPosition> Calculate::legPosition;

/*
@brief Inverse kinematics, calculates the joint angles based on the desired position
@param position The desired position in 3D space
@param legNum The leg number (1-6)
@return The calculated joint angles
*/
JointAngles Calculate::angle(Vector3 position, int legNum){
    LegConfig config = legConfigs[legNum];

    if(config.isMirrored){
        position.x = -position.x; // Mirror the x-coordinate for mirrored legs
    }

    double a1 = coxaLength;
    double a2 = femurLength;
    double a3 = tibiaLength;

    double coxaAngle = (atan2(position.x, position.y) * (180/pi));
    double r1 = std::hypot(position.x, position.y) - a1;
    double r2 = position.z;
    double q2 = atan(r2/r1) * (180/pi);
    double r3 = std::hypot(r1, r2);
    double q1 = acos((pow(a3,2) - pow(a2,2) - pow(r3,2)) / (-2*a2*r3)) * (180/pi);
    double femurAngle = (q2+q1);
    double q3 = acos((pow(r3,2) - pow(a2,2) - pow(a3,2)) / (-2*a2*a3)) * (180/pi);
    double tibiaAngle = 90 - q3;

    return {coxaAngle, femurAngle, tibiaAngle};
};

/*
@brief Forward kinematics, calculates the position based on the joint angles
@param angles The joint angles
@param legNum The leg number (1-6)
@return The calculated position in 3D space
*/
Vector3 Calculate::position(JointAngles angles, int legNum) {
    // Convert angles from degrees to radians
    angles.coxaAngle *= M_PI / 180.0;
    angles.femurAngle *= M_PI / 180.0;
    angles.tibiaAngle *= M_PI / 180.0;

    // Calculate the position based on the angles
    double r1 = femurLength * sin(angles.femurAngle);
    double r2 = femurLength * cos(angles.femurAngle);
    double r3 = tibiaLength * sin(angles.femurAngle + angles.tibiaAngle);
    double r4 = tibiaLength * cos(angles.femurAngle + angles.tibiaAngle);
    double z = r1 + r3 + coxaLength;
    double r5 = r2 + r4;
    double x = r5 * sin(angles.coxaAngle);
    double y = r5 * cos(angles.coxaAngle);

    Vector3 position = {x, y, z};

    return position;
}

/*
@brief Converts a position from leg frame to body frame based on the leg configuration
@param position The position in leg frame
@param legNum The leg number (1-6)
@return The position in body frame
*/
Vector3 Calculate::convertToBodyFrame(const Vector3& position, int legNum) {
    return position + legConfigs[legNum].translationOffset;
}

/*
@brief Converts a position from body frame to leg frame based on the leg configuration
@param position The position in body frame
@param legNum The leg number (1-6)
@return The position in leg frame
*/
Vector3 Calculate::convertToLegFrame(const Vector3& position, int legNum) {
    return position - legConfigs[legNum].translationOffset;
}

/*
@brief Calculates the direction vector based on joystick input
@param joystickValueX The joystick X value (-128 to 127)
@param joystickValueY The joystick Y value (-128 to 127)
@param start The starting position
@param legNum The leg number (1-6)
@param invert Whether to invert the direction
@param strideMultiplier The stride multiplier
@param useBodyFrame Whether to use body frame coordinates
@return The calculated direction vector
@note The function handles mirroring for legs and can operate in both body and leg frames
*/
Vector3 Calculate::direction(const int16_t& joystickValueX, const int16_t& joystickValueY, const Vector3& start, int legNum, bool invert, double strideMultiplier, bool useBodyFrame) {
    const double maxStride = 60.0;

    double stickX = static_cast<double>(joystickValueX);
    double stickY = static_cast<double>(joystickValueY);

    if (useBodyFrame) std::swap(stickX, stickY); // Swap X and Y to match the leg's coordinate system

    if (invert) {
        stickX = -stickX;
        stickY = -stickY;
    }

    if (legConfigs[legNum].isMirrored) {
        if (useBodyFrame) stickY = -stickY; else stickX = -stickX;
    }

    if (!useBodyFrame) stickX = -stickX;

    if (std::abs(stickX) <= 10 && std::abs(stickY) <= 10) {return start;}
   
    double magnitude = std::hypot(stickX, stickY) / 128.0;
    if (magnitude > 1.0) magnitude = 1.0;

    double stride = maxStride * magnitude * strideMultiplier;

    double angle = atan2(stickY, stickX);

    double rotationAngle = legConfigs[legNum].rotationAngle;
    double deltaX = stride * cos(angle);
    double deltaY = stride * sin(angle);

    if (!useBodyFrame) {return {start.x + deltaX, start.y + deltaY, start.z};}

    double dx_rot = deltaX * cos(rotationAngle) - deltaY * sin(rotationAngle);
    double dy_rot = deltaX * sin(rotationAngle) + deltaY * cos(rotationAngle);

    return {start.x + dx_rot, start.y + dy_rot, start.z};
}

/*
@brief Generates an arc trajectory between start and end positions
@param trajectory Pointer to an array to store the trajectory points
@param outSize Reference to an integer to store the number of points generated
@param start The starting position
@param end The ending position
@param liftHeight The height to lift the trajectory
@param resolution The number of points to generate
@param invert Whether to invert the trajectory
@return void
*/
void Calculate::GenerateArcTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int liftHeight, int resolution, bool invert) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        std::cerr << "Invalid resolution: " << resolution << std::endl;
        std::terminate();
    }

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x - end.x) < 1e-6 &&
        std::abs(start.y - end.y) < 1e-6 &&
        std::abs(start.z - end.z) < 1e-6) {
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    //Linear interpolation for x and y
    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        trajectory[i] = {
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
            start.z + (end.z - start.z) * t + (invert ? -1 : 1) * liftHeight * std::sin(M_PI * t)
        };
    }
    outSize = resolution + 1;
}

/*
@brief Generates a straight trajectory between start and end positions
@param trajectory Pointer to an array to store the trajectory points
@param outSize Reference to an integer to store the number of points generated
@param start The starting position
@param end The ending position
@param resolution The number of points to generate
@return void
*/
void Calculate::GenerateStraightTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int resolution) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        std::cerr << "Invalid resolution: " << resolution << std::endl;
        std::terminate();
    }

    // If start and end are (almost) the same, return a flat trajectory
    if (std::abs(start.x - end.x) < 1e-6 &&
        std::abs(start.y - end.y) < 1e-6 &&
        std::abs(start.z - end.z) < 1e-6) {
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    for (int i = 0; i <= resolution; i++) {
        double t = static_cast<double>(i) / resolution;
        trajectory[i] = {
            start.x + (end.x - start.x) * t,
            start.y + (end.y - start.y) * t,
            start.z + (end.z - start.z) * t
        };
    }
    outSize = resolution + 1;
}

/*
@brief Generates a Bezier curve trajectory between start and end positions
@param trajectory Pointer to an array to store the trajectory points
@param outSize Reference to an integer to store the number of points generated
@param start The starting position
@param end The ending position
@param liftHeight The height to lift the trajectory
@param resolution The number of points to generate
@param invert Whether to invert the trajectory
@return void
*/
void Calculate::GenerateBezierTrajectory(Vector3* trajectory, int& outSize, const Vector3& start, const Vector3& end, int liftHeight, int resolution, bool invert) {
    outSize = 0;
    if (resolution <= 0 || resolution > 10000) {
        std::cerr << "Invalid resolution: " << resolution << std::endl;
        std::terminate();
    }

    Vector3 dir = end - start;
    if (dir.length() == 0.0) {
        // Stationary case: generate flat path
        for (int i = 0; i <= resolution; ++i) {
            trajectory[i] = start;
        }
        outSize = resolution + 1;
        return;
    }

    Vector3 dirNorm = dir.normalized();
    double offsetScale = dir.length() * 0.25; // tweak as needed

    // Place control points before start and after end along the movement direction
    Vector3 P0 = start;
    Vector3 P3 = end;
    Vector3 P1 = start - dirNorm * offsetScale;
    Vector3 P2 = end + dirNorm * offsetScale;

    P1.z = start.z + liftHeight;
    P2.z = end.z + liftHeight;

    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        double u = 1.0 - t;

        Vector3 point =
            P0 * (u * u * u) +
            P1 * (3 * u * u * t) +
            P2 * (3 * u * t * t) +
            P3 * (t * t * t);

        trajectory[i] = point;
    }
    outSize = resolution + 1;
}

/*
@brief Applies joint angle offsets based on leg configuration
@param angles The original joint angles
@param legNum The leg number (1-6)
@return The joint angles with offsets applied
*/
JointAngles Calculate::ApplyOffsets(JointAngles& angles, int legNum) {
    // Apply leg offsets
    const int16_t* offset = configManager.getLegOffsets(legNum - 1);
    angles.coxaAngle  += offset[0];
    angles.femurAngle += offset[1];
    angles.tibiaAngle += offset[2];
    return angles;
}
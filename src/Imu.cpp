// gait_core (Eigen) before any Arduino header: Arduino defines macros such as B1 that break Eigen
#include "RobotConfig.h"

#include "Imu.h"

#include <Arduino.h>

#include <cmath>

namespace {

float invSqrt(float x) {
    return 1.0f / std::sqrt(x);
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// MadgwickFilter
// ---------------------------------------------------------------------------------------------

void MadgwickFilter::update(const float gyro[3], const float accel[3], float dt) {
    float ax = accel[0], ay = accel[1], az = accel[2];
    const bool hasAccel = !(ax == 0.0f && ay == 0.0f && az == 0.0f);

    // First reading: roll and pitch straight from gravity, yaw 0
    if (!initialised) {
        if (!hasAccel) {
            return;
        }
        const float roll = std::atan2(ay, az);
        const float pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        const float cr = std::cos(roll / 2), sr = std::sin(roll / 2);
        const float cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
        q0 = cr * cp;
        q1 = sr * cp;
        q2 = cr * sp;
        q3 = -sr * sp;
        initialised = true;
        return;
    }

    const float gx = gyro[0], gy = gyro[1], gz = gyro[2];

    // Rate of change of the quaternion from the gyroscope
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Gradient descent step pulling the estimated gravity towards the measured one
    if (hasAccel) {
        const float recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        const float _2q0 = 2.0f * q0, _2q1 = 2.0f * q1, _2q2 = 2.0f * q2, _2q3 = 2.0f * q3;
        const float _4q0 = 4.0f * q0, _4q1 = 4.0f * q1, _4q2 = 4.0f * q2;
        const float _8q1 = 8.0f * q1, _8q2 = 8.0f * q2;
        const float q0q0 = q0 * q0, q1q1 = q1 * q1, q2q2 = q2 * q2, q3q3 = q3 * q3;

        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        const float sNorm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (sNorm > 0.0f) {
            const float recipSNorm = invSqrt(sNorm);
            s0 *= recipSNorm;
            s1 *= recipSNorm;
            s2 *= recipSNorm;
            s3 *= recipSNorm;
            qDot0 -= gain * s0;
            qDot1 -= gain * s1;
            qDot2 -= gain * s2;
            qDot3 -= gain * s3;
        }
    }

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    const float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void MadgwickFilter::rollPitchYaw(float& roll, float& pitch, float& yaw) const {
    roll = std::atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    float sinPitch = 2.0f * (q0 * q2 - q3 * q1);
    sinPitch = sinPitch > 1.0f ? 1.0f : (sinPitch < -1.0f ? -1.0f : sinPitch);
    pitch = std::asin(sinPitch);
    yaw = std::atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
}

// ---------------------------------------------------------------------------------------------
// Imu
// ---------------------------------------------------------------------------------------------

Imu::Imu(TwoWire& bus, uint8_t address) : bus(bus), address(address), filter(robot::IMU_FILTER_GAIN) {}

bool Imu::update(uint64_t nowUs) {
    if (!ready) {
        if (resetting) {
            // Second step, on a later tick: the reset takes ~10 ms, so wait it out without blocking
            if (nowUs < resetDoneUs) {
                return false;
            }
            resetting = false;
            ready = sensor.configure(robot::IMU_ACCEL_RANGE_G, robot::IMU_GYRO_RANGE_DPS,
                                     1e6f / robot::LOOP_PERIOD_US);
            if (!ready) {
                Serial.printf("[imu] %s\n", sensor.error());
                return false;
            }
            reportedMissing = false;
            Serial.printf("[imu] %s ready (accel +-%d g, gyro +-%d dps, %.1f Hz output rate)\n", sensor.chipName(),
                          robot::IMU_ACCEL_RANGE_G, robot::IMU_GYRO_RANGE_DPS,
                          static_cast<double>(sensor.outputRate()));
            filter.reset();
            hasLastRead = false;
            return false;  // first reading next tick
        }

        // First step, once a second until the sensor answers: a missing one fails at once
        if (nowUs < nextInitAttemptUs) {
            return false;
        }
        nextInitAttemptUs = nowUs + 1000000;
        if (!sensor.startReset(bus, address)) {
            if (!reportedMissing) {
                Serial.printf("[imu] %s; walking without leveling, still looking\n", sensor.error());
                reportedMissing = true;
            }
            return false;
        }
        resetting = true;
        resetDoneUs = nowUs + 10000;
        return false;
    }

    ImuReading reading;
    if (!sensor.read(reading)) {
        Serial.printf("[imu] %s read failed, reinitialising\n", sensor.chipName());
        ready = false;
        return false;
    }

    const float dt = hasLastRead ? static_cast<float>(nowUs - lastReadUs) / 1e6f : 0.0f;
    lastReadUs = nowUs;
    hasLastRead = true;
    filter.update(reading.gyro, reading.accel, dt);
    return true;
}

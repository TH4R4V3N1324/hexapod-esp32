// gait_core (Eigen) before any Arduino header: Arduino defines macros such as B1 that break Eigen
#include "GaitController.h"
#include "RobotConfig.h"
#include "Servos.h"

#include <Arduino.h>
#include <Wire.h>

#include <algorithm>

#include "ConfigManager.h"
#include "Console.h"
#include "DataPacket.h"
#include "Imu.h"

namespace {

enum class State {
    BootWait,       // BOOT_DELAY_MS after boot, so you can stand clear
    FindingServos,  // waiting for all 18 servos to answer
    Running,        // the gait owns the servos
    Setup,          // the console has the servos; 'run' restarts into the gait
};

Servos servos;
GaitController controller(servos);
Imu imu(Wire, robot::IMU_ADDRESS);
Console console(servos);

State state = State::BootWait;
uint32_t bootMs = 0;
uint32_t nextServoCheckMs = 0;

uint64_t lastTickUs = 0;
uint64_t nextTickUs = 0;
uint32_t overruns = 0;
uint32_t longestTickUs = 0;

const char* stateName() {
    switch (state) {
        case State::BootWait: return "waiting after boot";
        case State::FindingServos: return "waiting for all 18 servos";
        case State::Running: return "running the gait";
        case State::Setup: return "setup (gait stopped; run starts it)";
    }
    return "";
}

void printStatus() {
    Serial.printf("State: %s\n", stateName());
    const uint32_t sinceController = MsSinceLastPacket();
    if (sinceController == UINT32_MAX) {
        Serial.println("Controller: nothing received yet");
    } else {
        Serial.printf("Controller: last packet %lu ms ago\n", static_cast<unsigned long>(sinceController));
    }
    Serial.printf("IMU: %s\n", imu.isReady() ? "ready" : "not found (walking without leveling)");
    if (state == State::Running) {
        const gait_core::GaitEngine& gait = controller.gait();
        const gait_core::GaitConfig& config = gait.gaitConfig();
        Serial.printf("Gait: %s, %s mode, %s, height %.0f mm\n", gait_core::GaitConfig::gaits[config.currentGait].name,
                      gait_core::GaitConfig::modes[config.currentMode].name, gait.seated() ? "seated" : "standing",
                      -config.currentHeight * 1000);
        Serial.printf("Loop: longest tick %lu us of %lu, %lu overruns\n", static_cast<unsigned long>(longestTickUs),
                      static_cast<unsigned long>(robot::LOOP_PERIOD_US), static_cast<unsigned long>(overruns));
        longestTickUs = 0;
    }
}

/**
 * @brief Start the gait once every servo answers
*/
void findServos() {
    if (static_cast<int32_t>(millis() - nextServoCheckMs) < 0) {
        return;
    }
    nextServoCheckMs = millis() + robot::SERVO_RETRY_MS;

    std::array<uint8_t, Servos::NUM_SERVOS> missing;
    const int count = servos.findMissing(missing);
    if (count > 0) {
        Serial.printf("[servos] %d of 18 missing:", count);
        for (int i = 0; i < count; ++i) {
            Serial.printf(" %d", missing[i]);
        }
        Serial.println(count == Servos::NUM_SERVOS ? ". Is the servo power on? (scan all finds other IDs)"
                                                   : ". (scan all finds other IDs)");
        return;
    }

    const uint64_t now = esp_timer_get_time();
    if (!controller.begin(now)) {
        Serial.println("[servos] A servo stopped replying while starting; trying again");
        return;
    }
    Serial.println("[servos] All 18 found, torque on. Starting the gait.");
    state = State::Running;
    lastTickUs = now;
    nextTickUs = now + robot::LOOP_PERIOD_US;
}

/**
 * @brief One 100 Hz tick of the gait: controller input, IMU, servos, telemetry
*/
void tick(uint64_t now) {
    const bool received = ReadInputData();
    if (received && CommandChanged()) {
        controller.handleCommand(controlPacket);
    }
    const bool commsOk = MsSinceLastPacket() < robot::COMMS_TIMEOUT_MS;

    const gait_core::Scalar dt = std::clamp(static_cast<gait_core::Scalar>(now - lastTickUs) / 1e6f,
                                            gait_core::Scalar(0), robot::MAX_TICK_DT);
    lastTickUs = now;

    if (imu.update(now)) {
        float roll, pitch, yaw;
        imu.rollPitchYaw(roll, pitch, yaw);
        controller.onImu(roll, pitch, yaw, now);
    }
    controller.update(dt, now, controlPacket, commsOk);

    // Answer each packet with the robot's status
    if (received) {
        controller.fillTelemetry(hexPacket);
        SendHexData();
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    configManager.loadLegOffsets();
    servos.begin();
    Wire.begin(robot::I2C_SDA_PIN, robot::I2C_SCL_PIN, robot::I2C_FREQUENCY);
    Wire.setTimeOut(3);  // ms, so a missing IMU can't stall the gait
    InitEspNow();

    console.printStatus = printStatus;
    bootMs = millis();
    Serial.printf("\nhexapod-esp32: the gait starts in %lu s. Type help for servo setup.\n",
                  static_cast<unsigned long>(robot::BOOT_DELAY_MS / 1000));
}

void loop() {
    switch (console.poll(state == State::Running)) {
        case ConsoleRequest::StopGait:
            servos.limp();
            state = State::Setup;
            Serial.println("Gait stopped, torque off. Type run to start it again.");
            break;
        case ConsoleRequest::EnterSetup:
            if (state != State::Setup) {
                state = State::Setup;
                Serial.println("Setup: the gait won't start until you type run.");
            }
            break;
        case ConsoleRequest::None:
            break;
    }
    ReportUnknownSender();

    switch (state) {
        case State::BootWait:
            if (millis() - bootMs >= robot::BOOT_DELAY_MS) {
                state = State::FindingServos;
            }
            break;

        case State::FindingServos:
            findServos();
            break;

        case State::Running: {
            uint64_t now = esp_timer_get_time();
            if (now < nextTickUs) {
                if (nextTickUs - now > 1500) {
                    delay(1);
                }
                return;
            }
            tick(now);
            const uint32_t took = static_cast<uint32_t>(esp_timer_get_time() - now);
            longestTickUs = std::max(longestTickUs, took);

            nextTickUs += robot::LOOP_PERIOD_US;
            // An overrun tick skips ahead rather than running the missed ones back to back
            now = esp_timer_get_time();
            if (nextTickUs < now) {
                ++overruns;
                nextTickUs = now + robot::LOOP_PERIOD_US;
            }
            return;
        }

        case State::Setup:
            break;
    }
    delay(1);
}

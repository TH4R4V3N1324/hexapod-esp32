// gait_core (Eigen) before any Arduino header: Arduino defines macros such as B1 that break Eigen
#include "GaitController.h"
#include "RobotConfig.h"
#include "Servos.h"
#include "hexapod_behaviour/battery_monitor.hpp"

#include <Arduino.h>
#include <Wire.h>

#include <algorithm>

#include "ConfigManager.h"
#include "Console.h"
#include "DataPacket.h"
#include "Imu.h"
#include "PowerMonitor.h"

using hexapod_behaviour::BatteryLevel;

namespace {

enum class State {
    Running,  // the gait owns the servos (servo power off until the first stand command)
    Setup,    // the console has the servos; 'run' restarts into the gait
};

Servos servos;
GaitController controller(servos);
Imu imu(Wire, robot::IMU_ADDRESS);
PowerMonitor power(Wire);
hexapod_behaviour::BatteryMonitor battery(PowerMonitor::NUM_HUBS, robot::batteryParams());
Console console(servos);

State state = State::Running;
uint64_t lastTickUs = 0;
uint64_t nextTickUs = 0;
uint64_t nextBatteryUs = 0;
uint32_t overruns = 0;
uint32_t longestTickUs = 0;

const char* levelName(gait_core::LogLevel level) {
    switch (level) {
        case gait_core::LogLevel::Debug: return "DEBUG";
        case gait_core::LogLevel::Info: return "INFO";
        case gait_core::LogLevel::Warn: return "WARN";
        case gait_core::LogLevel::Error: return "ERROR";
    }
    return "";
}

const char* batteryLevelName(BatteryLevel level) {
    switch (level) {
        case BatteryLevel::Ok: return "OK";
        case BatteryLevel::Warning: return "low";
        case BatteryLevel::Critical: return "critical";
    }
    return "";
}

gait_core::Timestamp stamp(uint64_t nowUs) {
    return static_cast<gait_core::Timestamp>(nowUs) * 1000;
}

void printStatus() {
    const gait_core::GaitEngine& gait = controller.gait();
    if (state == State::Running) {
        Serial.printf("State: running the gait, servo power %s, %d of %d servos answering\n",
                      gait.servoPower() ? "on" : "off (stand from the controller to power up)",
                      servos.answeringCount(), Servos::NUM_SERVOS);
        const gait_core::GaitConfig& config = gait.gaitConfig();
        Serial.printf("Gait: %s, %s mode, %s, height %.0f mm\n", gait_core::GaitConfig::gaits[config.currentGait].name,
                      gait_core::GaitConfig::modes[config.currentMode].name, gait.seated() ? "seated" : "standing",
                      -config.currentHeight * 1000);
        Serial.printf("Loop: longest tick %lu us of %lu, %lu overruns\n", static_cast<unsigned long>(longestTickUs),
                      static_cast<unsigned long>(robot::LOOP_PERIOD_US), static_cast<unsigned long>(overruns));
        longestTickUs = 0;
    } else {
        Serial.println("State: setup (gait stopped; run starts it)");
    }

    const uint32_t sinceController = MsSinceLastPacket();
    if (sinceController == UINT32_MAX) {
        Serial.println("Controller: nothing received yet");
    } else {
        Serial.printf("Controller: last packet %lu ms ago\n", static_cast<unsigned long>(sinceController));
    }
    Serial.printf("IMU: %s\n", imu.isReady() ? "ready" : "not found (walking without leveling)");

    for (int hub = 0; hub < PowerMonitor::NUM_HUBS; ++hub) {
        if (power.present(hub)) {
            Serial.printf("Power, %s hub: %.2f V, %.2f A\n", robot::LEG_HUB_NAMES[hub], power.voltage(hub),
                          power.current(hub));
        } else {
            Serial.printf("Power, %s hub: INA226 not found at 0x%02X\n", robot::LEG_HUB_NAMES[hub],
                          robot::LEG_HUB_ADDRESSES[hub]);
        }
    }
    bool hasData = false;
    float lowest = 0;
    for (int hub = 0; hub < battery.monitors(); ++hub) {
        const hexapod_behaviour::PowerReading& reading = battery.reading(hub);
        if (!battery.stale(hub, stamp(esp_timer_get_time())) && (!hasData || reading.voltage < lowest)) {
            lowest = reading.voltage;
            hasData = true;
        }
    }
    if (hasData) {
        Serial.printf("Battery: %s, %.2f V (filtered)\n", batteryLevelName(battery.level()), lowest);
    } else {
        Serial.println("Battery: no readings");
    }
}

/**
 * @brief Pass the leg hubs' readings to the battery monitor, and act on what it decides
 * @note Critical sits the robot down and, with power_off, turns the servos off once seated
*/
void updatePower(uint64_t now) {
    if (power.update(now)) {
        for (int hub = 0; hub < PowerMonitor::NUM_HUBS; ++hub) {
            if (power.fresh(hub)) {
                // The battery monitor takes current as negative while discharging
                battery.onReading(hub, power.voltage(hub), -power.current(hub), stamp(now));
            }
        }
    }
    if (state != State::Running || now < nextBatteryUs) {
        return;
    }
    nextBatteryUs = now + robot::BATTERY_UPDATE_PERIOD_US;
    const hexapod_behaviour::BatteryStatus status = battery.update(stamp(now));
    if (status.sit) {
        const gait_core::CommandResult result = status.powerOff ? controller.powerDown() : controller.sitDown();
        battery.sitRequested();
        if (!result.success) {
            Serial.printf("[battery] Couldn't sit down: %s\n", result.message);
        }
    }
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
    servos.begin();  // servo power off
    Wire.begin(robot::I2C_SDA_PIN, robot::I2C_SCL_PIN, robot::I2C_FREQUENCY);
    Wire.setTimeOut(3);  // ms, so a missing sensor can't stall the gait
    InitEspNow();

    battery.log = [](gait_core::LogLevel level, const char* message) {
        Serial.printf("[battery %s] %s\n", levelName(level), message);
    };
    console.printStatus = printStatus;

    lastTickUs = esp_timer_get_time();
    nextTickUs = lastTickUs + robot::LOOP_PERIOD_US;
    Serial.printf("\nhexapod-esp32: %s Type help for servo setup.\n",
                  robot::SERVO_ENABLE_PIN >= 0 ? "servo power is off until a stand command from the controller."
                                               : "the servos are powered; startup seats the robot.");
}

void loop() {
    // The console may take the servos whenever the gait isn't powering them
    const bool gaitOwnsServos = state == State::Running && controller.gait().servoPower();
    switch (console.poll(gaitOwnsServos)) {
        case ConsoleRequest::StopGait:
            servos.limp();
            state = State::Setup;
            Serial.println("Gait stopped, torque off. Type run to start it again.");
            break;
        case ConsoleRequest::EnterSetup:
            if (state != State::Setup) {
                state = State::Setup;
                Serial.println("Setup: the gait is stopped until you type run.");
            }
            break;
        case ConsoleRequest::None:
            break;
    }
    ReportUnknownSender();

    uint64_t now = esp_timer_get_time();
    updatePower(now);
    if (state != State::Running) {
        delay(1);
        return;
    }

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
}

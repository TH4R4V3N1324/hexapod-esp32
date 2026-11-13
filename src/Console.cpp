#include "Console.h"

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "ConfigManager.h"
#include "RobotConfig.h"

namespace {

constexpr uint16_t MOVE_SPEED = 341;        // steps/s, 30 deg/s
constexpr int ARRIVED_TOLERANCE = 10;       // steps
constexpr int CALIBRATE_MIN_TRAVEL = 50;    // steps
constexpr int DEFAULT_MARGIN = 20;          // steps inside each end of the travel

const char* HELP = R"(Servo setup. IDs are leg x 10 + joint: 11-13 ... 61-63.
  status                    What the firmware is doing
  scan [all]                Ping the 18 leg servos (or every ID 0-253) and show their state
  pos                       Where each leg servo is (degrees from its centre)
  set-id <old> <new>        Change a servo's ID. New servos are all ID 1: connect them one at a time
  wiggle <id> [degrees]     Move +degrees, back, -degrees, back (default 10): which joint, which way
  move <id> <degrees>       Move to an angle from the servo's centre and hold it
  limp [id]                 Torque off (default every servo). The legs will drop!
  calibrate <id> [margin]   Centre and limits by hand: move the joint through its full travel, Enter,
                            then hold it in its centre pose (leg laid out flat), Enter
  centre <id>               Just the centre (the limits move with it: redo them)
  offset <id> [steps]       Show the servo's stored centre offset, or set it (+-2047) to fine-tune
  limits <id>               Show the angle limits
  offsets clear             Clear the controller's calibration offsets (CMD_SET_CONFIG)
  stop                      Stop the gait and turn the torque off. The legs will drop!
  run                       Restart the firmware, which starts the gait (servos off until a stand)
Servo commands work while the gait isn't powering the servos: they stop the gait and power the
servos for setup, until run. Type q at an Enter prompt to cancel.
)";

/**
 * @brief "leg2 femur" for a leg servo ID, "-" otherwise
*/
const char* jointName(int id) {
    static char name[16];
    const int leg = id / 10, joint = id % 10;
    if (leg < 1 || leg > 6 || joint < 1 || joint > 3) {
        return "-";
    }
    static const char* joints[3] = {"coxa", "femur", "tibia"};
    snprintf(name, sizeof(name), "leg%d %s", leg, joints[joint - 1]);
    return name;
}

float stepsToDegreesSpan(int steps) {
    return steps * 360.0f / ST3215_STEPS_PER_REV;
}

/**
 * @brief The signed change between two position readings, across the 4095 -> 0 wrap
*/
int unwrapStep(int previous, int now) {
    return (now - previous + ST3215_CENTER_POSITION + ST3215_STEPS_PER_REV) % ST3215_STEPS_PER_REV -
           ST3215_CENTER_POSITION;
}

bool parseInt(const char* text, int& value) {
    if (text == nullptr) {
        return false;
    }
    char* end = nullptr;
    const long parsed = strtol(text, &end, 0);
    if (end == text || *end != '\0') {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool parseId(const char* text, uint8_t& id) {
    int value;
    if (!parseInt(text, value) || value < 0 || value >= ST3215_BROADCAST_ID) {
        Serial.println("Give a servo ID, 0-253");
        return false;
    }
    id = static_cast<uint8_t>(value);
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------------------------

bool Console::readLine() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        if (c == '\r' || c == '\n') {
            if (lineLength > 0) {
                line[lineLength] = '\0';
                lineLength = 0;
                return true;
            }
        } else if (lineLength + 1 < sizeof(line)) {
            line[lineLength++] = c;
        }
    }
    return false;
}

/**
 * @brief Call tick every periodMs until a line arrives
 * @return false if the line was q (cancel)
*/
bool Console::waitForEnter(const std::function<void()>& tick, uint32_t periodMs) {
    while (Serial.available() > 0) {
        Serial.read();
    }
    lineLength = 0;
    bool gotNewline = false;
    while (!gotNewline) {
        tick();
        const uint32_t until = millis() + periodMs;
        while (static_cast<int32_t>(millis() - until) < 0) {
            if (Serial.available() > 0) {
                const char c = static_cast<char>(Serial.read());
                if (c == '\r' || c == '\n') {
                    gotNewline = true;
                    break;
                }
                if (lineLength + 1 < sizeof(line)) {
                    line[lineLength++] = c;
                }
            }
            delay(1);
        }
    }
    line[lineLength] = '\0';
    lineLength = 0;
    Serial.println();
    return strcmp(line, "q") != 0;
}

bool Console::askYes(const char* question) {
    Serial.printf("%s [y/N] ", question);
    waitForEnter([] {}, 50);
    return line[0] == 'y' || line[0] == 'Y';
}

// ---------------------------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------------------------

ConsoleRequest Console::poll(bool gaitRunning) {
    if (!readLine()) {
        return ConsoleRequest::None;
    }
    char text[sizeof(line)];
    strcpy(text, line);
    Serial.printf("> %s\n", text);
    return run(text, gaitRunning);
}

ConsoleRequest Console::run(char* text, bool gaitRunning) {
    char* args[4] = {};
    int argc = 0;
    char* save = nullptr;
    const char* command = strtok_r(text, " \t", &save);
    while (argc < 4 && (args[argc] = strtok_r(nullptr, " \t", &save)) != nullptr) {
        ++argc;
    }
    if (command == nullptr) {
        return ConsoleRequest::None;
    }

    if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
        help();
        return ConsoleRequest::None;
    }
    if (strcmp(command, "status") == 0) {
        if (printStatus) {
            printStatus();
        }
        return ConsoleRequest::None;
    }
    if (strcmp(command, "run") == 0) {
        Serial.println("Restarting...");
        Serial.flush();
        ESP.restart();
    }
    if (strcmp(command, "stop") == 0) {
        return ConsoleRequest::StopGait;
    }

    static const char* servoCommands[] = {"scan", "pos", "set-id", "wiggle", "move", "limp", "calibrate",
                                          "centre", "center", "offset", "limits", "offsets"};
    const bool known = std::any_of(std::begin(servoCommands), std::end(servoCommands),
                                   [command](const char* name) { return strcmp(command, name) == 0; });
    if (!known) {
        Serial.printf("Unknown command '%s'. Type help.\n", command);
        return ConsoleRequest::None;
    }
    if (gaitRunning) {
        Serial.println("The gait is powering the servos. Type stop first (the legs will go limp).");
        return ConsoleRequest::None;
    }
    servos.powerOnAndWait();  // the gait stops for setup, and the servos need power to answer

    uint8_t id = 0;
    int value = 0;
    if (strcmp(command, "scan") == 0) {
        scan(argc > 0 && strcmp(args[0], "all") == 0);
    } else if (strcmp(command, "pos") == 0) {
        positions();
    } else if (strcmp(command, "set-id") == 0) {
        int newId;
        if (parseInt(args[0], value) && parseInt(args[1], newId)) {
            setId(value, newId);
        } else {
            Serial.println("Usage: set-id <old> <new>");
        }
    } else if (strcmp(command, "wiggle") == 0) {
        if (parseId(args[0], id)) {
            wiggle(id, argc > 1 ? static_cast<float>(atof(args[1])) : 10.0f);
        }
    } else if (strcmp(command, "move") == 0) {
        if (parseId(args[0], id) && argc > 1) {
            move(id, static_cast<float>(atof(args[1])));
        } else {
            Serial.println("Usage: move <id> <degrees>");
        }
    } else if (strcmp(command, "limp") == 0) {
        if (argc == 0) {
            limp(-1);
        } else if (parseId(args[0], id)) {
            limp(id);
        }
    } else if (strcmp(command, "calibrate") == 0) {
        if (parseId(args[0], id)) {
            int margin = DEFAULT_MARGIN;
            if (argc > 1 && !parseInt(args[1], margin)) {
                Serial.println("Usage: calibrate <id> [margin]");
            } else {
                calibrate(id, margin);
            }
        }
    } else if (strcmp(command, "centre") == 0 || strcmp(command, "center") == 0) {
        if (parseId(args[0], id)) {
            centre(id);
        }
    } else if (strcmp(command, "offset") == 0) {
        if (parseId(args[0], id)) {
            const bool set = argc > 1 && parseInt(args[1], value);
            offset(id, set, value);
        }
    } else if (strcmp(command, "limits") == 0) {
        if (parseId(args[0], id)) {
            printCalibration(id);
        }
    } else if (strcmp(command, "offsets") == 0) {
        if (argc > 0 && strcmp(args[0], "clear") == 0) {
            configManager.clear();
            Serial.println("Controller calibration offsets cleared");
        } else {
            for (int leg = 0; leg < gait_core::NUM_LEGS; ++leg) {
                const int16_t* o = configManager.getLegOffsets(leg);
                Serial.printf("  leg%d  coxa %+d  femur %+d  tibia %+d deg\n", leg + 1, o[0], o[1], o[2]);
            }
        }
    }
    return ConsoleRequest::EnterSetup;
}

void Console::help() {
    Serial.print(HELP);
}

void Console::scan(bool all) {
    Serial.printf("Pinging %s...\n", all ? "every ID 0-253" : "the 18 leg servos");
    Serial.println("   ID  joint         position  voltage  temp  error");
    int found = 0, missing = 0;
    for (int id = 0; id < (all ? ST3215_BROADCAST_ID : 64); ++id) {
        const bool legServo = std::strcmp(jointName(id), "-") != 0;
        if (!all && !legServo) {
            continue;
        }
        St3215State state;
        if (!bus().ping(static_cast<uint8_t>(id))) {
            if (legServo) {
                ++missing;
                if (!all) {
                    Serial.printf("  %3d  %-12s  missing\n", id, jointName(id));
                }
            }
            continue;
        }
        ++found;
        if (bus().readState(static_cast<uint8_t>(id), state)) {
            Serial.printf("  %3d  %-12s  %+7.1f   %5.1f V  %3d C  0x%02X%s\n", id, jointName(id),
                          Servos::stepsToDegrees(state.position), state.voltage / 10.0f, state.temperature,
                          state.error, legServo ? "" : "   <- not a leg ID: set-id it");
        } else {
            Serial.printf("  %3d  %-12s  replied to ping but not to a read\n", id, jointName(id));
        }
    }
    Serial.printf("%d found, %d leg servo%s missing\n", found, missing, missing == 1 ? "" : "s");
    if (found == 0) {
        Serial.println("Nothing replied: is the servo power on, and the adapter's TX/RX the right way round?");
    }
}

void Console::positions() {
    for (int legNum = 0; legNum < gait_core::NUM_LEGS; ++legNum) {
        Serial.printf("  leg%d", legNum + 1);
        for (int joint = 0; joint < 3; ++joint) {
            St3215State state;
            const uint8_t id = robot::servoId(legNum, joint);
            if (bus().readState(id, state)) {
                Serial.printf("   %2d %+7.1f", id, Servos::stepsToDegrees(state.position));
            } else {
                Serial.printf("   %2d      --", id);
            }
        }
        Serial.println();
    }
}

void Console::setId(int oldId, int newId) {
    if (newId < 0 || newId >= ST3215_BROADCAST_ID || newId == oldId || oldId < 0 || oldId >= ST3215_BROADCAST_ID) {
        Serial.println("IDs must be 0-253 and different");
        return;
    }
    const uint8_t oldU8 = static_cast<uint8_t>(oldId), newU8 = static_cast<uint8_t>(newId);
    if (!bus().ping(oldU8)) {
        Serial.printf("No servo replied on ID %d. Run scan all.\n", oldId);
        return;
    }
    if (bus().ping(newU8)) {
        Serial.printf("A servo already uses ID %d\n", newId);
        return;
    }
    if (std::strcmp(jointName(newId), "-") == 0) {
        Serial.printf("Note: %d isn't a leg servo ID (leg x 10 + joint, 11-13 ... 61-63)\n", newId);
    }
    const uint8_t unlock = 0, lock = 1;
    if (!bus().writeRegisters(oldU8, ST3215_REG_LOCK, &unlock, 1)) {
        Serial.printf("Servo %d didn't acknowledge the EEPROM unlock\n", oldId);
        return;
    }
    // The acknowledgement may come back from either ID, so check with a ping instead
    bus().writeRegisters(oldU8, ST3215_REG_ID, &newU8, 1);
    delay(50);
    bus().writeRegisters(newU8, ST3215_REG_LOCK, &lock, 1);
    if (!bus().ping(newU8)) {
        Serial.printf("The servo didn't reply on its new ID %d. Run scan all.\n", newId);
        return;
    }
    Serial.printf("Servo %d is now ID %d (%s)\n", oldId, newId, jointName(newId));
}

bool Console::holdCurrentPosition(uint8_t id, int& position) {
    St3215State state;
    if (!bus().readState(id, state)) {
        Serial.printf("Servo %d didn't reply. Run scan.\n", id);
        return false;
    }
    const int goal = state.position;
    bus().writeGoalPositions(&id, &goal, 1, 0, MOVE_SPEED);
    bus().setTorqueEnabled(&id, 1, true);
    position = goal;
    Serial.printf("Servo %d (%s) at %+.1f deg, torque on\n", id, jointName(id), Servos::stepsToDegrees(goal));
    return true;
}

bool Console::moveTo(uint8_t id, int goal) {
    St3215State state;
    if (!bus().readState(id, state)) {
        Serial.println("  no reply");
        return false;
    }
    Serial.printf("  -> %+.1f deg  ", Servos::stepsToDegrees(goal));
    bus().writeGoalPositions(&id, &goal, 1, 0, MOVE_SPEED);
    const uint32_t start = millis();
    const uint32_t timeout = std::abs(goal - state.position) * 1000u / MOVE_SPEED + 2000;
    int peakLoad = 0;
    while (millis() - start < timeout) {
        delay(50);
        if (!bus().readState(id, state)) {
            Serial.println("no reply");
            return false;
        }
        peakLoad = std::max(peakLoad, std::abs(state.load));
        if (std::abs(state.position - goal) <= ARRIVED_TOLERANCE && state.speed == 0) {
            break;
        }
    }
    const int error = state.position - goal;
    Serial.printf("reached %+.1f deg (%s by %+.1f deg) in %.2f s, peak load %.1f%%\n",
                  Servos::stepsToDegrees(state.position), std::abs(error) <= ARRIVED_TOLERANCE ? "ok" : "MISSED",
                  stepsToDegreesSpan(error), (millis() - start) / 1000.0f, peakLoad / 10.0f);
    return true;
}

void Console::wiggle(uint8_t id, float degrees) {
    int home;
    if (!holdCurrentPosition(id, home)) {
        return;
    }
    const int offset = static_cast<int>(std::lround(degrees * ST3215_STEPS_PER_REV / 360.0f));
    const int goals[4] = {home + offset, home, home - offset, home};
    for (int goal : goals) {
        if (!moveTo(id, std::clamp(goal, 0, ST3215_STEPS_PER_REV - 1))) {
            return;
        }
    }
    Serial.printf("Torque is still on. limp %d releases it.\n", id);
}

void Console::move(uint8_t id, float degrees) {
    int home;
    if (holdCurrentPosition(id, home) && moveTo(id, Servos::degreesToSteps(degrees))) {
        Serial.printf("Torque is still on. limp %d releases it.\n", id);
    }
}

void Console::limp(int id) {
    if (id < 0) {
        servos.limp();
        Serial.println("Torque off on every servo");
        return;
    }
    const uint8_t u8 = static_cast<uint8_t>(id);
    bus().setTorqueEnabled(&u8, 1, false);
    Serial.printf("Torque off on servo %d\n", id);
}

bool Console::positionByHand(uint8_t id, const char* what, int& position) {
    St3215State state;
    if (!bus().setTorqueEnabled(&id, 1, false) || !bus().readState(id, state)) {
        Serial.printf("Servo %d didn't reply. Run scan.\n", id);
        return false;
    }
    Serial.printf("Torque off on servo %d (%s). Move it by hand to %s, then press Enter (q cancels).\n", id,
                  jointName(id), what);
    bool have = false;
    const bool entered = waitForEnter([&] {
        St3215State now;
        if (bus().readState(id, now)) {
            position = now.position;
            have = true;
            Serial.printf("\r  at %+7.1f deg (%4d steps)   ", Servos::stepsToDegrees(now.position), now.position);
        } else {
            Serial.print("\r  no reply                    ");
        }
    }, 100);
    if (entered && have) {
        return true;
    }
    Serial.println("Cancelled");
    return false;
}

/**
 * @brief Make where the joint is now read as the centre (2048 steps), kept in EEPROM, and leave torque off
*/
bool Console::centreHere(uint8_t id, int& position) {
    const uint8_t unlock = 0, lock = 1, setCentre = ST3215_TORQUE_SET_CENTRE;
    bus().writeRegisters(id, ST3215_REG_LOCK, &unlock, 1);
    const bool centred = bus().writeRegisters(id, ST3215_REG_TORQUE_ENABLE, &setCentre, 1);
    delay(100);
    bus().writeRegisters(id, ST3215_REG_LOCK, &lock, 1);
    bus().setTorqueEnabled(&id, 1, false);
    St3215State state;
    if (!centred || !bus().readState(id, state)) {
        return false;
    }
    position = state.position;
    return true;
}

void Console::printCalibration(uint8_t id) {
    int offset, low, high;
    if (!bus().readWord(id, ST3215_REG_POSITION_OFFSET, offset, 11) ||
        !bus().readWord(id, ST3215_REG_MIN_ANGLE_LIMIT, low) || !bus().readWord(id, ST3215_REG_MAX_ANGLE_LIMIT, high)) {
        Serial.printf("Servo %d: no reply\n", id);
        return;
    }
    Serial.printf("Servo %d (%s): offset %+d steps (%+.1f deg), ", id, jointName(id), offset,
                  stepsToDegreesSpan(offset));
    if (low == 0 && high == ST3215_STEPS_PER_REV - 1) {
        Serial.println("no limits");
    } else {
        Serial.printf("limits %+.1f to %+.1f deg (%d to %d steps)\n", Servos::stepsToDegrees(low),
                      Servos::stepsToDegrees(high), low, high);
    }
}

void Console::centre(uint8_t id) {
    int low, high, position;
    if (!bus().readWord(id, ST3215_REG_MIN_ANGLE_LIMIT, low) || !bus().readWord(id, ST3215_REG_MAX_ANGLE_LIMIT, high)) {
        Serial.printf("Servo %d didn't reply. Run scan.\n", id);
        return;
    }
    if (!positionByHand(id, "the pose that should read 0 (the leg laid out flat)", position)) {
        return;
    }
    if (!centreHere(id, position)) {
        Serial.printf("Servo %d didn't acknowledge centring\n", id);
        return;
    }
    Serial.printf("Centred: it now reads %+.1f deg (%d steps). Torque is off.\n", Servos::stepsToDegrees(position),
                  position);
    printCalibration(id);
    if (!(low == 0 && high == ST3215_STEPS_PER_REV - 1)) {
        Serial.printf("Its limits were set before centring, so they've moved with it: run calibrate %d.\n", id);
    }
}

/**
 * @brief Record the joint's travel by hand, then its centre, and write the centre and the limits
 * @note The limits go margin steps inside the ends of the recorded travel
*/
void Console::calibrate(uint8_t id, int margin) {
    const int maxSteps = ST3215_STEPS_PER_REV - 1;
    if (margin < 0) {
        Serial.println("The margin must be 0 or more steps");
        return;
    }
    St3215State start;
    if (!bus().setTorqueEnabled(&id, 1, false) || !bus().readState(id, start)) {
        Serial.printf("Servo %d didn't reply. Run scan.\n", id);
        return;
    }

    // Position unwrapped from where it started, so a range across 4095 -> 0 still works
    int raw = start.position;
    int unwrapped = 0, lowest = 0, highest = 0;
    auto track = [&]() {
        St3215State state;
        if (!bus().readState(id, state)) {
            return false;
        }
        unwrapped += unwrapStep(raw, state.position);
        raw = state.position;
        return true;
    };

    Serial.printf("Torque off on servo %d (%s).\n", id, jointName(id));
    Serial.println("Step 1: move the joint slowly through its FULL travel, then press Enter (q cancels).");
    if (!waitForEnter([&] {
            if (!track()) {
                Serial.print("\r  no reply                                        ");
                return;
            }
            lowest = std::min(lowest, unwrapped);
            highest = std::max(highest, unwrapped);
            Serial.printf("\r  at %4d steps   travel %4d steps (%5.1f deg)   ", raw, highest - lowest,
                          stepsToDegreesSpan(highest - lowest));
        }, 30)) {
        Serial.println("Cancelled, nothing written");
        return;
    }
    const int travel = highest - lowest;
    if (travel < CALIBRATE_MIN_TRAVEL) {
        Serial.printf("Travel too small (%d steps): nothing written\n", travel);
        return;
    }
    if (travel > maxSteps) {
        Serial.println("Travel is more than a full turn, which angle limits can't hold: nothing written");
        return;
    }

    Serial.println("Step 2: hold the joint in its centre pose (the leg laid out flat), then press Enter.");
    if (!waitForEnter([&] {
            if (!track()) {
                Serial.print("\r  no reply                                                ");
                return;
            }
            Serial.printf("\r  at %4d steps   %6.1f deg from the lower end   %6.1f deg from the upper end   ", raw,
                          stepsToDegreesSpan(unwrapped - lowest), stepsToDegreesSpan(highest - unwrapped));
        }, 30)) {
        Serial.println("Cancelled, nothing written");
        return;
    }
    if (unwrapped < lowest || unwrapped > highest) {
        Serial.println("The centre is outside the travel you recorded: nothing written");
        return;
    }

    int low = ST3215_CENTER_POSITION + (lowest - unwrapped) + margin;
    int high = ST3215_CENTER_POSITION + (highest - unwrapped) - margin;
    if (low >= high) {
        Serial.printf("A margin of %d steps leaves no travel between the limits: nothing written\n", margin);
        return;
    }
    if (low < 0 || high > maxSteps) {
        Serial.println("The centre is more than 180 deg from one end of the travel: nothing written");
        return;
    }
    Serial.printf("Travel %.1f deg; this pose becomes the centre, with limits %+.1f to %+.1f deg (%d steps inside "
                  "each end).\n", stepsToDegreesSpan(travel), Servos::stepsToDegrees(low),
                  Servos::stepsToDegrees(high), margin);
    if (!askYes("Keep the joint still. Write the centre and limits to the servo's EEPROM?")) {
        Serial.println("Nothing written");
        return;
    }

    track();  // catch any drift while the question was open
    int centrePosition;
    if (!centreHere(id, centrePosition)) {
        Serial.printf("Servo %d didn't acknowledge centring: nothing written\n", id);
        return;
    }
    low = std::max(0, centrePosition + (lowest - unwrapped) + margin);
    high = std::min(maxSteps, centrePosition + (highest - unwrapped) - margin);
    const uint8_t limits[4] = {static_cast<uint8_t>(low & 0xFF), static_cast<uint8_t>(low >> 8),
                               static_cast<uint8_t>(high & 0xFF), static_cast<uint8_t>(high >> 8)};
    if (!bus().writeEeprom(id, ST3215_REG_MIN_ANGLE_LIMIT, limits, 4)) {
        Serial.printf("Centred, but servo %d didn't acknowledge the limits. Run calibrate again.\n", id);
        return;
    }
    Serial.printf("Centred: it now reads %+.1f deg. Torque is off.\n", Servos::stepsToDegrees(centrePosition));
    printCalibration(id);
}

void Console::offset(uint8_t id, bool set, int steps) {
    if (set) {
        if (std::abs(steps) > 2047) {
            Serial.println("The offset must be within +-2047 steps");
            return;
        }
        const uint16_t raw = St3215Bus::encodeSigned(steps, 11);
        const uint8_t data[2] = {static_cast<uint8_t>(raw & 0xFF), static_cast<uint8_t>(raw >> 8)};
        if (!bus().writeEeprom(id, ST3215_REG_POSITION_OFFSET, data, 2)) {
            Serial.printf("Servo %d didn't acknowledge the write\n", id);
            return;
        }
    }
    printCalibration(id);
}

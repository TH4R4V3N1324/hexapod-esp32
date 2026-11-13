#ifndef _CONSOLE_H_
#define _CONSOLE_H_

#include <cstddef>
#include <cstdint>
#include <functional>

#include "Servos.h"

/**
 * @brief What the console asks the main loop to do
*/
enum class ConsoleRequest {
    None,
    StopGait,    // 'stop': the gait stops and the legs go limp
    EnterSetup,  // a servo command ran while the gait wasn't powering the servos: stop the gait
};

/**
 * @brief Servo setup over USB serial: set IDs, check directions, calibrate centres and limits
 * @note Centres, limits and IDs are stored in the servos' own EEPROM. Servo commands only run while
 *       the gait isn't powering the servos; they stop it and switch servo power on for setup.
*/
class Console {
public:
    explicit Console(Servos& servos) : servos(servos) {}

    /**
     * @brief Read any input and run a command once a whole line has arrived
     * @param[in] gaitRunning Whether the gait is powering the servos
    */
    ConsoleRequest poll(bool gaitRunning);

    std::function<void()> printStatus;  // for 'status'

private:
    Servos& servos;
    St3215Bus& bus() { return servos.bus(); }
    char line[96];
    size_t lineLength = 0;

    ConsoleRequest run(char* text, bool gaitRunning);
    bool readLine();
    bool waitForEnter(const std::function<void()>& tick, uint32_t periodMs);
    bool askYes(const char* question);

    void help();
    void scan(bool all);
    void positions();
    void setId(int oldId, int newId);
    void wiggle(uint8_t id, float degrees);
    void move(uint8_t id, float degrees);
    void limp(int id);
    void centre(uint8_t id);
    void calibrate(uint8_t id, int margin);
    void offset(uint8_t id, bool set, int steps);
    void printCalibration(uint8_t id);

    bool holdCurrentPosition(uint8_t id, int& position);
    bool moveTo(uint8_t id, int goal);
    bool positionByHand(uint8_t id, const char* what, int& position);
    bool centreHere(uint8_t id, int& position);
};

#endif

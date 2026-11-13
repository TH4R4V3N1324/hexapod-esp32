# hexapod-esp32

Embedded control firmware for the hexapod on an ESP32, driving 18 **ST3215 serial bus servos**. The
gait comes from the [hexapod-gait-core](https://github.com/TH4R4V3N1324/hexapod-gait-core) library.

Because the ST3215s report their position and load, the gait runs **closed loop**: it starts from
where the legs really are, and it feels the ground, so **load balancing and terrain adaptation work**.

The robot, its build, and how the stacks fit together are documented in
[hexapod-docs](https://github.com/TH4R4V3N1324/hexapod-docs). This README only covers building
and working on the firmware.

## Quick start

You need [PlatformIO](https://platformio.org/) (the VS Code extension or the CLI). The project pins
the `espressif32` platform at 6.11.0 (Arduino-ESP32 2.0.17).

```sh
git clone --recursive https://github.com/TH4R4V3N1324/hexapod-esp32.git
cd hexapod-esp32
pio run -t upload
pio device monitor
```

`--recursive` matters: gait core is a submodule, and Eigen is a submodule inside it. In an existing
clone, run `git submodule update --init --recursive`.

**Before powering the servos,** read [Safety](https://github.com/TH4R4V3N1324/hexapod-docs/blob/main/docs/getting-started/safety.md)
and support the robot so its legs are clear of the ground. New servos must be given their IDs and
centres first (see [Servo setup](#servo-setup)). Five seconds after boot, once all 18 servos answer,
the firmware turns their torque on where they are, moves the legs into the seated pose, then stands up.

## Hardware

| | ESP32 pin | Notes |
| --- | --- | --- |
| Servo bus RX | GPIO 16 | To the bus adapter's TX. UART2 at 1 Mbaud |
| Servo bus TX | GPIO 17 | To the bus adapter's RX |
| IMU SDA | GPIO 21 | ISM330DHCX at `0x6A`, 400 kHz |
| IMU SCL | GPIO 22 | |

- **Board:** any ESP32 dev board (`esp32dev`).
- **Servos:** 18 ST3215s, IDs **leg × 10 + joint** (coxa 1, femur 2, tibia 3):
  11–13 for leg 1 up to 61–63 for leg 6.
- **Bus adapter:** a Waveshare Bus Servo Adapter (A) turns the ESP32's UART into
  the servos' half-duplex bus. If yours echoes what it sends back to RX, set `SERVO_ECHO`.
- **IMU:** mounted with its axes along the body's (x forward, y left, z up). Without it, the robot
  walks without leveling, and the firmware looks for it again once a second.
- **Controller:** the handheld controller talks to the ESP32 directly over ESP-NOW.

Share a ground between the ESP32, the adapter and the servo supply. Every pin, address and setting is
in `include/RobotConfig.h`.

## Repository layout

```text
include/, src/
  main.cpp           Entry point, start-up and the 100 Hz control loop
  RobotConfig.h      Pins, geometry, GaitParams, servo IDs and directions, timings, IMU settings
  GaitController     Controller packets → GaitEngine inputs; servo feedback in, joint targets out
  Servos             Joint angles ↔ ST3215 steps, feedback and torque for the 18 servos
  St3215Bus          Feetech STS protocol on an ESP32 UART
  Console            Servo setup over USB serial
  Imu, Ism330dhcx    ISM330DHCX driver and Madgwick filter
  DataPacket         ESP-NOW packets exchanged with the controller
  ConfigManager      The controller's calibration offsets, in NVS flash
external/
  hexapod-gait-core  Submodule, pinned to a release tag
```

## How it works

Each 100 Hz tick, `main.cpp` takes the latest packet from the controller, reads the IMU, and calls
`GaitController::update()`. That reads every servo's position and load in one sync read, gives them
to the engine as a joint state, runs `GaitEngine::update()`, and writes the 18 joint targets back in
one sync write. Reading and writing the bus takes about 4 ms of the 10 ms tick. An overrun tick skips
ahead rather than catching up. The firmware answers each controller packet with a status packet.

Gait settings not set in `robot::gaitParams()` keep gait core's defaults. The only change is
`stand_on_startup`, since the controller may have no sit button.

### Joint-to-servo mapping

Each servo's centre (2048 steps) is the joint's 0, which is the leg laid out flat:

```text
servo° from centre = JOINT_DIRECTION × joint° + controller offset
steps              = 2048 + servo° × 4096 / 360, clamped to 0–4095
effort (N·m)       = JOINT_DIRECTION × load / 1000 × 2.94
```

`JOINT_DIRECTION` is `{1, 1, −1}`: the tibia servo turns the other way to its joint. The ST3215s
turn ±180°, so gait core's joint limits are used as they are, and the servos' own angle limits guard
the end stops.

A servo that misses a reply keeps its last reading, and a warning is printed if
it stays silent for 0.5 s.

## Servo setup

New ST3215s all come as ID 1, and each joint's centre and limits are stored in the servo itself. The
firmware has a USB serial console for this (115200 baud; `pio device monitor`).

1. **IDs:** connect the servos **one at a time** and run `scan all` to find each one, then
   `set-id 1 <id>`, for example `set-id 1 23` for leg 2's tibia.
2. **Check:** with all 18 connected, `scan` should list them all with sensible voltages.
3. **Directions:** `wiggle <id>` moves a joint +10°, back, −10° and back. If a joint turns the wrong
   way, flip its entry in `JOINT_DIRECTION`.
4. **Centres and limits:** `calibrate <id>`. Move the joint through its full travel, press Enter,
   hold it in its centre pose (coxa straight out, femur level, the foot ball in line with the femur),
   press Enter, then confirm. The limits go 20 steps inside the ends you reached.
5. **Start:** `run` restarts the firmware, which starts the gait.

Typing any servo command during the 5-second boot wait keeps the gait from starting until `run`.
While it's running, only `help`, `status`, `stop` and `run` work. `stop` turns the torque off, so
**the legs drop**. `help` lists everything, including `move`, `limp`, `centre`, `offset` and `limits`.

## Controller interface

The packets in `include/DataPacket.h` travel over ESP-NOW, and only packets from `CONTROLLER_MAC` in `RobotConfig.h` are accepted. The
firmware prints the address of any other sender, so you can copy it in.

```cpp
struct ControlPacket {           // controller → robot
    int16_t joystick1X, joystick1Y, joystick2X, joystick2Y;  // ±128, X right, Y forward
    int16_t currentHeight;       // body height in mm, 0 = unchanged
    Command command;
    int16_t commandArgs[3];
};

struct HexPacket {               // robot → controller, in reply to each packet
    int16_t legConfigs[3];       // calibration offsets requested with CMD_REQUEST_CONFIG
    int16_t currentHeight;       // mm
    int8_t currentPhase;         // always 0
    Gait currentGait;
    Mode currentMode;
};
```

| Command | Arguments |
| --- | --- |
| `CMD_SET_GAIT` | `[0]` gait |
| `CMD_SET_MODE` | `[0]` mode. `MODE_CONFIG` (every joint to 0 + offset: the legs laid out flat) is handled here, not by the engine |
| `CMD_SET_CONFIG` | `[0]` leg 0–5, `[1]` joint 0–2, `[2]` offset in degrees (clamped to ±60) |
| `CMD_HOME_STANCE` | Stand up |
| `CMD_REQUEST_CONFIG` | `[0]` leg; its offsets are returned in `legConfigs` |
| `CMD_SIT` | `[0]` 1 sit, 0 stand |

A command is applied once, when it or its arguments change. `Gait` and `Mode` values equal
gait-core's (checked with `static_assert`). If no packet arrives for 200 ms, the sticks count as
centred, so the robot stops walking. New commands must be appended so existing values don't change.

The `CMD_SET_CONFIG` offsets are kept in NVS flash and added on top of each servo's own centre. They're
for fine-tuning from the controller; the centres themselves are set with `calibrate`. `offsets` on the console shows them, and `offsets clear` resets them.

## Development notes

- **Gait changes belong in hexapod-gait-core**, not here. To update it, check
  out a newer tag in `external/hexapod-gait-core`, rebuild, and commit the submodule.
- **Include gait core before Arduino.** Arduino's `binary.h` defines macros such as `B1` that break
  Eigen if Eigen is parsed after it. Each `.cpp` includes a header that pulls in gait core (such as
  `RobotConfig.h`) before any Arduino header. Arduino also defines `DEG_TO_RAD`, `RAD_TO_DEG` and
  `PI`, so don't use those names.
- **No heap in the loop:** `St3215Bus` uses fixed buffers, and gait core doesn't allocate while
  running.
- **Packet layout:** `DataPacket.h` must stay byte-compatible with the controller.

### Troubleshooting

- **`fatal error: Eigen/Dense: No such file or directory`:** the nested submodule is missing. Run
  `git submodule update --init --recursive`.
- **Errors inside Eigen such as `expected unqualified-id before numeric constant`:** a source file
  includes an Arduino header before gait core. See above.
- **The gait never starts, and the monitor lists missing servos:** check the servo power, the
  adapter's TX and RX against GPIO 16 and 17, and the IDs with `scan all`.
- **The robot ignores the controller:** check the monitor for `Ignored a packet from ...` and put
  that address in `CONTROLLER_MAC`.
- **The robot doesn't level:** check the IMU is at `0x6A`, on GPIO 21 and 22, and mounted with its
  axes along the body's.

## License

Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE). hexapod-gait-core is Apache-2.0; Eigen,
inside it, is MPL-2.0.

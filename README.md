# Hexapod ESP32

Firmware for an ESP32-based six-legged robot controller. The project receives packed motion commands over ESP-NOW, converts joystick and mode input into gait trajectories, solves inverse kinematics for each leg, and drives 18 servos using the ESP32 MCPWM and LEDC peripherals.

The firmware also supports persistent leg calibration offsets through EEPROM, I²C communication with external controller hardware, and sensor integrations for current monitoring, IMU data, and leg-contact detection.

## Features

- Six-leg hexapod motion control
- 18-servo output: three joints per leg
- Inverse and forward kinematics
- Normal walking and strafing modes
- Tripod, ripple, and wave gaits
- Smooth startup, shutdown, and return-to-start trajectories
- Straight, arc, and Bézier trajectory generation
- Joystick-based translation and rotation control
- Collision-aware swing trajectory adjustment
- Per-leg and per-joint calibration offsets
- EEPROM persistence for calibration data
- ESP-NOW command and telemetry packets
- I²C master support
- Current, IMU, and leg-contact sensor abstractions
- PlatformIO-based build and upload workflow

## Hardware and software stack

- **Target MCU:** ESP32
- **Framework:** Arduino for ESP32
- **Build system:** PlatformIO
- **Language:** C++17
- **Servo peripherals:** ESP32 MCPWM and LEDC
- **Wireless protocol:** ESP-NOW
- **Wired protocol:** I²C
- **External library:** Adafruit INA260 Library
- **Default PlatformIO board:** `esp32dev`

The project also contains a commented ESP32-S3 board option in `platformio.ini`:

```ini
;[env:esp32-s3-devkitc-1]
;board = esp32-s3-devkitc-1
```

## Repository layout

```text
include/
  Animation.h          Gait sequencing, modes, startup, and trajectories
  Calculate.h          Kinematics, coordinate transforms, and trajectory math
  ConfigManager.h      EEPROM-backed calibration offsets
  DataPacket.h         Control and telemetry packet definitions
  Hexapod.h            Main project include aggregation
  I2CManager.h         ESP32 I²C master wrapper
  Move.h               Leg-position and servo movement interface
  Servo.h              MCPWM/LEDC servo abstraction
  sensors/
    Sensor.h           Common sensor interface
    SensorBase.h       Sensor base definitions
    CurrentSensor.h    INA260 current-sensor interface
    IMUSensor.h        IMU sensor interface
    LegContactSensor.h Leg contact sensor interface

src/
  main.cpp             Firmware setup, command FSM, state FSM, and loop
  Animation.cpp        Gait definitions and high-level walking behavior
  Calculate.cpp        Forward/inverse kinematics and trajectory generation
  ConfigManager.cpp    EEPROM calibration storage
  DataPacket.cpp       ESP-NOW packet handling and command-change detection
  I2CManager.cpp       I²C initialization, read/write, and bus scanning
  Move.cpp             Leg movement through calculated joint angles
  Servo.cpp            Servo pin mapping and PWM output
  sensors/
    CurrentSensor.cpp
    IMUSensor.cpp
    LegContactSensor.cpp

platformio.ini         PlatformIO environment and dependency configuration
```

## System architecture

The runtime is organized as a command-processing layer followed by a motion-state layer:

```text
ESP-NOW ControlPacket
          │
          ▼
   DataPacket.cpp
          │
          ├── CommandFSM()
          │     ├── SetGait()
          │     ├── SetMode()
          │     ├── SetLegConfig()
          │     ├── returnToStart()
          │     └── configuration response
          │
          └── StateFSM()
                ├── Normal()
                ├── Strafe()
                ├── Tilt() [not currently active]
                └── ConfigState()
                         │
                         ▼
                    Animation
                         │
                         ▼
                       Move
                         │
              Calculate + ConfigManager
                         │
                         ▼
                     Servo output
```

At startup, `setup()` initializes I²C on GPIO 20/21, initializes and loads EEPROM calibration offsets, sets the initial body height to `120`, waits five seconds, and runs `Animation::Startup()`.

The main loop checks `CommandChanged()` and processes new commands only once. It then dispatches the active robot mode through `StateFSM()` and advances the motion controller with a one-millisecond delay.

## Motion control

### Gaits

`Animation::GetLegConfig()` defines the supported gait groupings:

| Gait | Leg groups |
| --- | --- |
| Tripod | `{1, 3, 5}` and `{2, 4, 6}` |
| Ripple | `{3, 6}`, `{2, 4}`, and `{1, 5}` |
| Wave | `{3}`, `{2}`, `{1}`, `{4}`, `{5}`, and `{6}` |

Gait changes are deferred until the current return-to-start sequence completes. This prevents an active gait from being replaced in the middle of a phase transition.

### Modes

The state machine currently includes:

- `MODE_NORMAL` — forward/backward walking with turning
- `MODE_STRAFE` — lateral movement with rotation
- `MODE_TILT` — reserved; implementation is currently commented out
- `MODE_CONFIG` — moves legs toward a configuration position

### Trajectories

The kinematics layer provides:

- Straight trajectories for direct interpolation
- Arc trajectories with sinusoidal lift
- Bézier trajectories with elevated control points

Swing legs use Bézier trajectories to lift the foot during a step. Stance legs use straight trajectories. `Animation::GenerateTrajectories()` converts target positions into body-frame coordinates, performs a 50 mm collision-distance check, then converts targets back into each leg's coordinate frame.

### Kinematics

`Calculate::angle()` implements inverse kinematics for the coxa, femur, and tibia joints. `Calculate::position()` performs forward kinematics. Leg-specific translation offsets, mirrored-leg handling, rotation angles, and joystick direction transformations are applied through the coordinate-frame helpers.

The default link lengths are defined in `Calculate.h` as `coxaLength`, `femurLength`, and `tibiaLength`. Check that file before changing the physical geometry of the robot.

## Servo output

The firmware controls 18 servos. The default mapping in `src/Servo.cpp` uses:

- 12 MCPWM servos on GPIOs `4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22`
- 6 LEDC servos on GPIOs `23, 25, 26, 27, 32, 33`
- 50 Hz PWM for both servo backends
- A nominal pulse range of approximately 1000–2000 µs

Each leg maps to three servo channels: coxa, femur, and tibia. `Move::Position()` calculates joint angles, applies calibration offsets, and writes the resulting angles through `Servo::setAngle()`.

> Confirm the pin assignments, servo direction conventions, and mechanical limits before powering a connected robot.

## Communication

### ESP-NOW

`src/DataPacket.cpp` receives `ControlPacket` messages through ESP-NOW. Incoming packets are accepted only when their length equals `sizeof(ControlPacket)`. After updating the control state, the firmware sends the current `HexPacket` back to the configured controller MAC address.

The current controller peer address is:

```cpp
uint8_t controllerMAC[6] = {
    0x80, 0x65, 0x99, 0xE9, 0x6F, 0x56
};
```

ESP-NOW encryption is disabled in the current implementation. Update the peer address and security configuration before deploying to a production robot.

### I²C

`I2CManager` wraps the ESP-IDF I²C master driver and provides:

- `init()` and `deinit()`
- Raw device reads and writes
- Register reads and writes
- I²C bus scanning through `scanBus()`

The application initializes I²C with:

```cpp
i2cManager.init(20, 21);
```

This selects GPIO 20 for SDA and GPIO 21 for SCL. The actual bus frequency and timeout are defined in `I2CManager.h`.

## Packet processing

The firmware keeps global `ControlPacket` and `HexPacket` instances in `DataPacket.cpp`.

`CommandChanged()` compares the current command and its three arguments against the previous values. The command is processed only when one of those fields changes.

The command dispatcher supports:

| Command | Action |
| --- | --- |
| `CMD_SET_GAIT` | Selects the gait from `commandArgs[0]` |
| `CMD_SET_MODE` | Selects the active mode from `commandArgs[0]` |
| `CMD_SET_CONFIG` | Updates a leg/joint calibration offset |
| `CMD_HOME_STANCE` | Returns the robot to its start position |
| `CMD_REQUEST_CONFIG` | Copies calibration offsets into `HexPacket` |

The exact packed field layout is defined in `include/DataPacket.h`. Keep the packet definitions synchronized with the transmitter firmware.

## Persistent calibration

`ConfigManager` stores six legs × three joints of `int16_t` offsets in ESP32 EEPROM.

Calibration values are clamped to the range `-60` through `60` before being saved:

```cpp
configManager.SetLegConfig(legNum, joint, offset);
```

The startup sequence calls:

```cpp
configManager.initEEPROM();
configManager.loadLegOffsets();
```

Offsets are applied by `Calculate::ApplyOffsets()` immediately before servo angles are written.

## Sensors

The sensor layer is split into interfaces under `include/sensors/` and implementations under `src/sensors/`:

- `CurrentSensor` — current and power monitoring using the Adafruit INA260 library
- `IMUSensor` — inertial measurement support
- `LegContactSensor` — leg contact detection
- `Sensor` and `SensorBase` — shared sensor abstractions

The PlatformIO dependency is declared in `platformio.ini`:

```ini
lib_deps = adafruit/Adafruit INA260 Library
```

## Building and uploading

### Prerequisites

Install:

- Git
- PlatformIO CLI or PlatformIO for VS Code
- An ESP32 development board compatible with the selected PlatformIO board definition
- A suitable USB cable and serial drivers
- A separate power supply appropriate for the servo system

### Build

```bash
git clone https://github.com/TH4R4V3N1324/hexapod-esp32.git
cd hexapod-esp32
pio run
```

The default environment is:

```text
Environment: esp32dev
Framework:   Arduino
C++ standard: GNU++17
```

### Upload

```bash
pio run --target upload
```

If PlatformIO does not detect the board automatically, specify the upload port:

```bash
pio run --target upload --upload-port PORT
```

Examples:

```bash
pio run --target upload --upload-port COM4
pio run --target upload --upload-port /dev/ttyUSB0
```

### Serial monitor

```bash
pio device monitor --baud 115200
```

## First boot and safety checklist

The firmware waits five seconds during `setup()` before starting the startup motion sequence. On first boot:

1. Keep the robot raised or mechanically supported.
2. Confirm the servo power supply is disconnected or current-limited.
3. Verify all servo GPIO assignments.
4. Confirm leg numbering and mirrored-leg configuration.
5. Confirm the controller MAC address.
6. Check I²C wiring on GPIO 20 and GPIO 21.
7. Test one movement mode at a time.
8. Verify that calibration offsets are correct before loading the robot.

`Animation::Startup()` first performs a shutdown/deactivation sequence on its first invocation, then interpolates all legs from the home position to their configured start positions.

## Troubleshooting

### The project does not compile

- Confirm that PlatformIO is using the `esp32dev` environment.
- Confirm that the ESP32 Arduino framework is installed.
- Make sure the project is compiled as C++17; this is configured by `-std=gnu++17`.
- Check that the `include/sensors` include path is active.

### ESP-NOW commands are not received

- Verify the transmitter uses the same packed `ControlPacket` layout.
- Confirm the controller MAC address in `src/DataPacket.cpp`.
- Ensure both devices use a compatible Wi-Fi channel.
- Confirm ESP-NOW initialization succeeds and inspect serial output.

### The robot moves unpredictably

- Verify the leg numbering and mirrored-leg settings in `Calculate.h`.
- Check coxa, femur, and tibia lengths against the physical robot.
- Confirm the servo GPIO mapping in `src/Servo.cpp`.
- Clear or update invalid EEPROM calibration offsets.
- Test with the robot lifted off the ground.

### Servos do not move

- Verify the servo power supply separately from ESP32 USB power.
- Check that the configured GPIOs are connected to the intended servo channels.
- Confirm the servo signal ground is shared with the ESP32 ground.
- Check the generated angles and calibration offsets.
- Confirm the selected servo backend matches the configured channel.

### I²C devices are not detected

Call `i2cManager.scanBus()` after initialization and inspect the serial output. Check SDA/SCL wiring, pull-ups, device addresses, and the bus frequency defined in `I2CManager.h`.

## Development workflow

1. Make changes in `include/` and `src/`.
2. Build with PlatformIO:

   ```bash
   pio run
   ```

3. Test communication and motion with the robot mechanically supported.
4. Verify packet sizes and command behavior with the transmitter firmware.
5. Test calibration persistence after power cycling.
6. Document hardware or pin-mapping changes in the pull request.

## Known implementation notes

- `MODE_TILT` is present in the state machine but its call is currently commented out.
- ESP-NOW encryption is disabled.
- The peer MAC address is hard-coded.
- The default upload target is `esp32dev`; ESP32-S3 support is present only as commented configuration.
- Motion code uses fixed trajectory resolutions in several animation routines, commonly `50` steps.
- `Calculate::GenerateArcTrajectory()` terminates the process for invalid resolutions rather than returning an error.
- Calibration indexing assumes valid leg and joint indices; validate external command data before using it in safety-critical deployments.
- This repository does not currently include a license file.

## License

No license file is currently present. Unless a license is added, the source should be treated as all rights reserved.

## Contributing

1. Create a feature branch.
2. Keep packet definitions compatible with the controller and receiver projects.
3. Build the firmware with PlatformIO.
4. Test on hardware with the robot supported safely.
5. Include board, wiring, and test details in pull requests.

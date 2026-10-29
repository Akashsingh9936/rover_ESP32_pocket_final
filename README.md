# Rover ESP32 Firmware — RadioMaster Pocket

Differential-drive rover controlled by an ESP32, using a RadioMaster Pocket transmitter with RP1 receiver (ExpressLRS/CRSF protocol). Supports manual RC control and autonomous operation via NVIDIA Jetson AGX (ROS2).

## Current Status

- **Manual control** — complete and tested
- **Autonomous ESP32 firmware** — AGX serial parser, watchdog, source select
- **ROS2 bridge node** — bidirectional, deployed and tested on AGX
- **Encoder odometry** — I2C slave ESP32, $ODM packets to AGX at 20 Hz
- **Status telemetry** — $STS packets to AGX at 5 Hz
- **Dual mixer modes** — asymmetric (with outer wheel boost) and scaled yaw
- **Beacon & buzzer** — armed indicator + AUTO mode alert

## Architecture

```
MANUAL:  RadioMaster Pocket → ELRS → RP1 → CRSF (Serial2) → ESP32 → MCP4922 DAC → KLE Motors
AUTO:    Jetson AGX (ROS2) → bridge node → Serial1 → ESP32 → MCP4922 DAC → KLE Motors

Feedback:  Encoder ESP32 → I2C → Main ESP32 → $ODM/$STS → Serial1 → AGX → /odom, /rover_status
```

## Hardware

| Component          | Details                                              |
|--------------------|------------------------------------------------------|
| MCU                | ESP32 DevKitV1                                       |
| Encoder MCU        | ESP32 (I2C slave, 2x quadrature encoders)            |
| DAC                | MCP4922 dual 12-bit SPI DAC                          |
| RC Transmitter     | RadioMaster Pocket                                   |
| RC Receiver        | RadioMaster RP1 (ExpressLRS / CRSF)                  |
| Motor Drivers      | KLE driver per side (0–3.3V analog + 12V direction)  |
| Autonomy Computer  | NVIDIA Jetson AGX (ROS2 Humble)                      |

### Pin Map

| GPIO | Function              |
|------|-----------------------|
| 16   | CRSF RX (Serial2)    |
| 17   | CRSF TX (Serial2)    |
| 5    | MCP4922 CS (SPI)     |
| 18   | MCP4922 SCK (SPI)    |
| 23   | MCP4922 SDI (SPI)    |
| 27   | Left direction relay  |
| 25   | Right direction relay |
| 13   | Left brake relay      |
| 26   | Right brake relay     |
| 14   | LED relay             |
| 19   | Buzzer (AUTO mode)    |
| 32   | Beacon (armed)        |
| 21   | I2C SDA (encoder)     |
| 22   | I2C SCL (encoder)     |
| 4    | AGX Serial1 RX        |
| 33   | AGX Serial1 TX        |

### RC Channel Assignments (RadioMaster Pocket)

| CH  | Function          | Values                                             |
|-----|-------------------|----------------------------------------------------|
| CH3 | Throttle          | 174 (full rev) — 992 (center) — 1811 (full fwd)   |
| CH1 | Yaw               | 174 (full left) — 992 (center) — 1811 (full right)|
| CH5 | Mode toggle       | 191 = MANUAL, 1792 = AUTO                          |
| CH7 | LED relay         | 1004 = OFF, 1792 = ON                              |
| CH8 | ARM / DISARM      | 191 = DISARM, 1792 = ARM                           |
| CH9 | Dead man's switch | 191 = released, 1792 = pressed (manual only)       |

## Differential Mixer

Two selectable modes via `#define MIXER_MODE`:

**Mode 0 — Asymmetric (default):** Outer wheel gets throttle + 20% yaw boost (`OUTER_BOOST`), inner wheel slows. Avoids choke point at mid-throttle turns. Spin-in-place when throttle = 0.

**Mode 1 — Scaled Yaw:** Standard mixer with `YAW_SCALE` factor (default 0.5).

```
Sign      → direction relay (negative = reverse)
|magnitude| × 4095 → DAC value (0–3.3V)
```

## Serial Protocol

### AGX → ESP32 (commands)
```
$CMD,<throttle>,<yaw>\n       20 Hz, floats -1.0 to +1.0
```

### ESP32 → AGX (feedback)
```
$ODM,<leftTicks>,<rightTicks>,<timestampMs>\n    20 Hz encoder odometry
$STS,<mode>,<armed>,<rcLink>,<led>,<deadman>\n   5 Hz status telemetry
```

## ROS2 Bridge (AGX)

Bidirectional Python bridge deployed on Jetson AGX (`/dev/ttyTHS1`):

| Direction | Flow | ROS2 Topic |
|-----------|------|------------|
| ROS2 → ESP32 | `/cmd_vel` → `$CMD` | geometry_msgs/Twist |
| ESP32 → ROS2 | `$ODM` → `/odom` + TF | nav_msgs/Odometry |
| ESP32 → ROS2 | `$STS` → `/rover_status` | std_msgs/String |

## Safety Chain

Movement requires **ALL** of:
1. RC link up (CRSF)
2. Armed (CH8 = 1792)
3. **MANUAL:** Dead man pressed (CH9 = 1792) — OR — **AUTO:** AGX alive (valid `$CMD` within 500ms)

| Event              | Result                                        |
|--------------------|-----------------------------------------------|
| RC link lost       | Full stop, brakes ON, LED OFF, beacon/buzzer OFF |
| Disarmed           | Full stop, brakes ON, beacon OFF              |
| Dead man released  | Full stop, brakes ON (manual)                 |
| AGX timeout        | Full stop, brakes ON (auto)                   |

**Kill methods:** Disarm (CH8), flip to Manual (CH5), or turn off transmitter (link loss).

**Indicators:** Beacon ON = armed. Buzzer beeping = AUTO mode active.

## Build & Upload

Requires [PlatformIO](https://platformio.org/).

```bash
pio run -e esp32dev  -t upload   # production firmware (manual + autonomous)
pio run -e esp32copy -t upload   # manual-only backup (tested, working)
pio device monitor -b 115200     # serial monitor
```

## Project Structure

```
rover_ESP32_pocket_final/
├── README.md
├── CLAUDE.md
├── platformio.ini                  # 2 build envs: esp32dev, esp32copy
└── src/
    ├── main_rover_pocket.cpp       # Production firmware (manual + autonomous)
    └── main_rover_pocket_copy.cpp  # Manual-only backup
```

## Roadmap

- [x] Manual control with RadioMaster Pocket
- [x] Autonomous ESP32 firmware (AGX serial parser, watchdog, source select)
- [x] Dual mixer modes (asymmetric with outer boost + scaled yaw)
- [x] I2C encoder integration (slave ESP32, 20 Hz polling)
- [x] $ODM odometry sender (ESP32 → AGX)
- [x] $STS status telemetry sender (ESP32 → AGX)
- [x] ROS2 bridge node — bidirectional, deployed on AGX
- [x] Beacon (armed indicator) and buzzer (AUTO mode alert)
- [x] Outer wheel boost for better turn response
- [ ] Hardware assembly with final circuit
- [ ] Calibration (wheel_radius, wheel_base, max_speed, max_yaw_rate)
- [ ] End-to-end autonomous test with Nav2

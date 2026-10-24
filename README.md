# Rover ESP32 Firmware — RadioMaster Pocket

Differential-drive rover controlled by an ESP32, using a RadioMaster Pocket transmitter with RP1 receiver (ExpressLRS/CRSF protocol).

## Current Status

- **Manual control** — complete and tested
- **Autonomous ESP32 firmware** — implemented (AGX serial parser, watchdog, source select)
- **ROS2 bridge node** — implemented, tested on RPi5, portable to AGX
- **Dual mixer modes** — asymmetric (wider turns) and scaled yaw (adjustable sensitivity)

## Architecture

```
MANUAL:  RadioMaster Pocket → ELRS → RP1 → CRSF (Serial2) → ESP32 → MCP4922 DAC → KLE Motors
AUTO:    Jetson AGX (ROS2) → bridge node → Serial1 → ESP32 → MCP4922 DAC → KLE Motors
```

## Hardware

| Component        | Details                                              |
|------------------|------------------------------------------------------|
| MCU              | ESP32 DevKitV1                                       |
| DAC              | MCP4922 dual 12-bit SPI DAC                          |
| RC Transmitter   | RadioMaster Pocket                                   |
| RC Receiver      | RadioMaster RP1 (ExpressLRS / CRSF)                  |
| Motor Drivers    | KLE driver per side (0–3.3V analog + 12V direction)  |
| Autonomy Computer  | NVIDIA Jetson AGX (ROS2)                           |

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

**Mode 0 — Asymmetric (default):** Outer wheel stays at throttle, only inner slows. Spin-in-place when throttle = 0.

**Mode 1 — Scaled Yaw:** Standard mixer with `YAW_SCALE` factor (default 0.5).

```
Sign      → direction relay (negative = reverse)
|magnitude| × 4095 → DAC value (0–3.3V)
```

## Safety Chain

Movement requires **ALL** of:
1. RC link up (CRSF)
2. Armed (CH8 = 1792)
3. **MANUAL:** Dead man pressed (CH9 = 1792) — OR — **AUTO:** AGX alive (valid `$CMD` within 500ms)

| Event              | Result                        |
|--------------------|-------------------------------|
| RC link lost       | Full stop, brakes ON, LED OFF |
| Disarmed           | Full stop, brakes ON          |
| Dead man released  | Full stop, brakes ON (manual) |
| AGX timeout        | Full stop, brakes ON (auto)   |

**Kill methods:** Disarm (CH8), flip to Manual (CH5), or turn off transmitter (link loss).

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
- [x] ROS2 bridge node — tested on RPi5, portable to AGX
- [x] Dual mixer modes (asymmetric + scaled yaw)
- [ ] Hardware assembly with final circuit
- [ ] Calibration (max_speed, max_yaw_rate)
- [ ] End-to-end autonomous testing

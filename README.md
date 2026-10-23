# Rover ESP32 Firmware — RadioMaster Pocket

Differential-drive rover controlled by an ESP32, using a RadioMaster Pocket transmitter with RP1 receiver (ExpressLRS/CRSF protocol).

## Current Status

- **Manual control** — complete and tested
- **Autonomous control** — to be added (Jetson AGX via serial)

## Architecture

```
MANUAL:  RadioMaster Pocket → ELRS → RP1 → CRSF (Serial2) → ESP32 → MCP4922 DAC → KLE Motors
AUTO:    Jetson AGX (ROS2) → bridge node → Serial1 → ESP32 → MCP4922 DAC → KLE Motors  [PLANNED]
```

## Hardware

| Component        | Details                                              |
|------------------|------------------------------------------------------|
| MCU              | ESP32 DevKitV1                                       |
| DAC              | MCP4922 dual 12-bit SPI DAC                          |
| RC Transmitter   | RadioMaster Pocket                                   |
| RC Receiver      | RadioMaster RP1 (ExpressLRS / CRSF)                  |
| Motor Drivers    | KLE driver per side (0–3.3V analog + 12V direction)  |
| Autonomy (planned) | NVIDIA Jetson AGX (ROS2)                           |

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

```
Left  = clamp(throttle + yaw, -1.0, +1.0)
Right = clamp(throttle - yaw, -1.0, +1.0)

Sign      → direction relay (negative = reverse)
|magnitude| × 4095 → DAC value (0–3.3V)
```

## Safety Chain

Movement requires **ALL** of:
1. RC link up (CRSF)
2. Armed (CH8 = 1792)
3. Dead man pressed (CH9 = 1792) — manual mode

| Event              | Result                        |
|--------------------|-------------------------------|
| RC link lost       | Full stop, brakes ON, LED OFF |
| Disarmed           | Full stop, brakes ON          |
| Dead man released  | Full stop, brakes ON          |

**Kill methods:** Disarm (CH8) or turn off transmitter (link loss).

## Build & Upload

Requires [PlatformIO](https://platformio.org/).

```bash
pio run -t upload               # build and flash to ESP32
pio device monitor -b 115200    # serial monitor
```

## Project Structure

```
rover_ESP32_pocket_final/
├── README.md
├── CLAUDE.md
├── platformio.ini
└── src/
    └── main_rover_pocket.cpp
```

## Roadmap

- [x] Manual control with RadioMaster Pocket
- [ ] Autonomous mode — AGX serial parser, watchdog, source select
- [ ] ROS2 bridge node on Jetson AGX
- [ ] End-to-end autonomous testing

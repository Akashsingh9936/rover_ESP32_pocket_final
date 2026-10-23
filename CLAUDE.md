# Rover ESP32 Firmware — RadioMaster Pocket

Differential-drive rover controlled by an ESP32, using a RadioMaster Pocket transmitter with RP1 receiver (ExpressLRS/CRSF). Two planned operating modes — manual (working) and autonomous (to be added).

## Architecture

```
MANUAL:  RadioMaster Pocket → ELRS → RP1 → CRSF (Serial2) → ESP32 → MCP4922 DAC → KLE Motors
AUTO:    Jetson AGX (ROS2) → bridge node → Serial1 → ESP32 → MCP4922 DAC → KLE Motors  [NOT YET IMPLEMENTED]
```

## Hardware

- **MCU:** ESP32 DevKitV1
- **DAC:** MCP4922 dual 12-bit SPI DAC (VOUTA → left motor, VOUTB → right motor)
- **RC Transmitter:** RadioMaster Pocket
- **RC Receiver:** RadioMaster RP1 (ExpressLRS / CRSF protocol)
- **Motor Drivers:** KLE driver per side (0–3.3V analog speed + 12V direction relay)
- **Autonomy Computer:** NVIDIA Jetson AGX (ROS2) — planned

### Pin Map

| GPIO | Function             | Notes                                   |
|------|----------------------|-----------------------------------------|
| 16   | CRSF RX (Serial2)   | RP1 TX → ESP32                          |
| 17   | CRSF TX (Serial2)   | Telemetry back to RP1                   |
| 5    | MCP4922 CS (SPI)    | Active LOW                              |
| 18   | MCP4922 SCK (SPI)   |                                         |
| 23   | MCP4922 SDI (SPI)   |                                         |
| 27   | Left direction relay |                                         |
| 25   | Right direction relay|                                         |
| 13   | Left brake relay    |                                         |
| 26   | Right brake relay   |                                         |
| 14   | LED relay           |                                         |

### RC Channel Assignments (RadioMaster Pocket)

| CH  | Function          | Values                                          |
|-----|-------------------|-------------------------------------------------|
| CH3 | Throttle          | 174 (full rev) — 992 (center) — 1811 (full fwd) |
| CH1 | Yaw               | 174 (full left) — 992 (center) — 1811 (full right) |
| CH5 | Mode toggle       | 191 = MANUAL, 1792 = AUTO                       |
| CH7 | LED relay         | 1004 = OFF, 1792 = ON                           |
| CH8 | ARM / DISARM      | 191 = DISARM, 1792 = ARM                        |
| CH9 | Dead man's switch | 191 = released, 1792 = pressed (manual only)    |

## Code Structure

```
rover_ESP32_pocket_final/
├── platformio.ini              # Build config (esp32dev env)
├── CLAUDE.md                   # This file — project context
└── src/
    └── main_rover_pocket.cpp   # Rover firmware (manual + autonomous planned)
```

### Build & Upload

```bash
pio run -t upload               # build + flash to ESP32
pio device monitor -b 115200    # serial monitor
```

### Key Functions in main_rover_pocket.cpp

| Function           | Purpose                                              |
|--------------------|------------------------------------------------------|
| `normalizeStick()` | CRSF value (174–1811) → float (-1.0 to +1.0)        |
| `computeOutputs()` | Safety gate → source select → differential mix → outputs |
| `writeMCP4922()`   | SPI write to MCP4922 DAC channel                     |
| `applyOutputs()`   | Write DAC values and set all relay GPIOs              |

### Differential Mixer

```
Inputs:  throttle (-1.0 to +1.0), yaw (-1.0 to +1.0)
         — from CRSF sticks (manual) or AGX serial packets (auto, planned)

Left  = clamp(throttle + yaw, -1.0, +1.0)
Right = clamp(throttle - yaw, -1.0, +1.0)
Sign → direction relay (negative = reverse)
|magnitude| × 4095 → DAC value (0–3.3V)
```

## Safety Chain

Movement requires ALL of:
1. RC link up (CRSF)
2. Armed (CH8 = 1792)
3. **MANUAL:** dead man pressed (CH9 = 1792)
   — OR —
   **AUTO:** not yet implemented (currently `canMove = false`)

### Fail-Safes

| Event              | Result                                |
|--------------------|---------------------------------------|
| RC link lost       | Full stop, brakes ON, LED OFF         |
| Disarmed           | Full stop, brakes ON                  |
| Dead man released  | Full stop, brakes ON (manual mode)    |

### Operator Kill Methods (always available)
1. Disarm (CH8)
2. Turn off TX (link loss)

## Conventions

- CRSF values: 174–1811 range, center ~992, switch threshold 1500
- DAC range: 0–4095 (12-bit), maps to 0–3.3V
- Relay polarity controlled by `RELAY_ACTIVE_HIGH` define
- All safe defaults: brakes ON, DACs zero, relays OFF, LED OFF

## Project Status

- [x] Manual control firmware — complete, tested with RadioMaster Pocket
- [ ] Autonomous ESP32 firmware (AGX serial parser, watchdog, source select)
- [ ] ROS2 bridge node on AGX
- [ ] Hardware assembly with final circuit
- [ ] Calibration (max_speed, max_yaw_rate)
- [ ] End-to-end autonomous test

## Related

- Original multi-file project (Zorro TX + autonomous): `TX_RX/` repo
- Bridge design doc for AGX team: `TX_RX/ESP-AGX_Bridge_Doc.md`

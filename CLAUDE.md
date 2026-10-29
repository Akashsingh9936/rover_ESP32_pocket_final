# Rover ESP32 Firmware — RadioMaster Pocket

Differential-drive rover controlled by an ESP32, using a RadioMaster Pocket transmitter with RP1 receiver (ExpressLRS/CRSF). Two operating modes share one motor-control pipeline — only the input source changes.

## Architecture

```
MANUAL:  RadioMaster Pocket → ELRS → RP1 → CRSF (Serial2) → ESP32 → MCP4922 DAC → KLE Motors
AUTO:    Jetson AGX (ROS2) → bridge node → Serial1 → ESP32 → MCP4922 DAC → KLE Motors
```

CH5 (RC switch) selects which source feeds the differential mixer at runtime. Everything downstream — mixing, DAC, relays, brakes, safety — is identical for both modes.

## Hardware

- **MCU:** ESP32 DevKitV1
- **DAC:** MCP4922 dual 12-bit SPI DAC (VOUTA → left motor, VOUTB → right motor)
- **RC Transmitter:** RadioMaster Pocket
- **RC Receiver:** RadioMaster RP1 (ExpressLRS / CRSF protocol)
- **Motor Drivers:** KLE driver per side (0–3.3V analog speed + 12V direction relay)
- **Autonomy Computer:** NVIDIA Jetson AGX (ROS2)

### Pin Map

| GPIO | Function              | Notes                                   |
|------|-----------------------|-----------------------------------------|
| 16   | CRSF RX (Serial2)    | RP1 TX → ESP32                          |
| 17   | CRSF TX (Serial2)    | Telemetry back to RP1                   |
| 5    | MCP4922 CS (SPI)     | Active LOW                              |
| 18   | MCP4922 SCK (SPI)    |                                         |
| 23   | MCP4922 SDI (SPI)    |                                         |
| 27   | Left direction relay  |                                         |
| 25   | Right direction relay |                                         |
| 13   | Left brake relay      |                                         |
| 26   | Right brake relay     |                                         |
| 14   | LED relay             |                                         |
| 19   | Buzzer                | Beeps 1s ON/OFF in AUTO mode            |
| 32   | Beacon                | ON when armed, OFF when disarmed        |
| 21   | I2C SDA               | Encoder ESP32 (slave 0x10)              |
| 22   | I2C SCL               | Encoder ESP32 (slave 0x10)              |
| 4    | AGX Serial1 RX       | ESP32 receives from AGX                 |
| 33   | AGX Serial1 TX       | ESP32 sends $ODM/$STS to AGX            |

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
├── platformio.ini                  # 2 build envs: esp32dev, esp32copy
├── CLAUDE.md                       # This file — project context
├── README.md                       # GitHub-facing project overview
└── src/
    ├── main_rover_pocket.cpp       # Production firmware (manual + autonomous)
    └── main_rover_pocket_copy.cpp  # Manual-only backup (tested, working)
```

### Build & Upload

```bash
pio run -e esp32dev  -t upload   # manual + autonomous firmware (production)
pio run -e esp32copy -t upload   # manual-only firmware (tested backup)
pio device monitor -b 115200     # serial monitor
```

Note: PlatformIO CLI may not be in PATH. Use full path if needed: `~/.platformio/penv/bin/pio`

### Key Functions in main_rover_pocket.cpp

| Function           | Purpose                                              |
|--------------------|------------------------------------------------------|
| `parseAGXSerial()` | Reads Serial1, parses `$CMD,t,y\n` packets, runs watchdog |
| `normalizeStick()` | CRSF value (174–1811) → float (-1.0 to +1.0)        |
| `computeOutputs()` | Safety gate → source select → differential mix → outputs |
| `writeMCP4922()`   | SPI write to MCP4922 DAC channel                     |
| `applyOutputs()`   | Write DAC values and set all relay GPIOs              |
| `readEncoderI2C()` | I2C master request from encoder slave ESP32            |
| `sendODM()`        | Send `$ODM,leftTicks,rightTicks,timestamp\n` to AGX   |
| `sendSTS()`        | Send `$STS,mode,armed,rcLink,led,deadman\n` to AGX    |

### Differential Mixer

Two selectable modes controlled by `#define MIXER_MODE` (0 or 1):

**Mode 0 — Asymmetric (default):** Outer wheel gets throttle + 20% yaw boost (`OUTER_BOOST`), inner wheel slows. Avoids choke point at mid-throttle turns. Spin-in-place preserved when throttle = 0.

```
Driving + turning:
  Right turn (yaw > 0): left = throttle + OUTER_BOOST × yaw, right = throttle - yaw × sign(throttle)
  Left turn  (yaw < 0): left = throttle + yaw × sign(throttle), right = throttle + OUTER_BOOST × |yaw|

Spin in place (throttle = 0):
  left = +yaw, right = -yaw   (standard symmetric)
```

**Mode 1 — Scaled Yaw:** Standard differential mixer with reduced yaw sensitivity. Turn rate controlled by `#define YAW_SCALE` (0.0–1.0, default 0.5).

```
left  = throttle + yaw × YAW_SCALE
right = throttle - yaw × YAW_SCALE
```

Both modes: sign → direction relay, |magnitude| × 4095 → DAC value (0–3.3V)

## AGX Serial Bridge (Autonomous Mode)

Status: **ESP32 side implemented** in `main_rover_pocket.cpp`. **Bridge node implemented** in `~/ros2_ws/src/rover_bridge/` — tested on RPi5, portable to AGX.

### What's Implemented (ESP32 Side)

- **Serial parser** (`parseAGXSerial()`): reads `$CMD` packets from Serial1 (GPIO 4/33, 115200 baud), extracts throttle and yaw floats, clamps to -1.0...+1.0
- **Watchdog**: tracks last valid packet timestamp. If no packet within 500ms → `agxAlive = false` → throttle/yaw zeroed → rover stops
- **Source select** in `computeOutputs()`: CH5=AUTO + agxAlive → uses AGX values; CH5=MANUAL → uses CRSF sticks. Same mixer downstream
- **Debug output**: in AUTO mode, serial monitor shows `AGX:OK t=0.50 y=0.30` or `AGX:DEAD t=0.00 y=0.00`

### Serial Protocol (AGX → ESP32)

```
Format:   $CMD,<throttle>,<yaw>\n
Baud:     115200 (8N1, no flow control)
Rate:     20 Hz (every 50ms)
Throttle: -1.000 to +1.000  (negative = reverse)
Yaw:      -1.000 to +1.000  (negative = left, positive = right)
```

### Serial Protocol (ESP32 → AGX)

```
$ODM,<leftTicks>,<rightTicks>,<timestampMs>\n    — 20 Hz encoder odometry
$STS,<mode>,<armed>,<rcLink>,<led>,<deadman>\n   — 5 Hz status telemetry
```

| $STS Field | Values         |
|------------|----------------|
| mode       | 0=MANUAL, 1=AUTO |
| armed      | 0/1            |
| rcLink     | 0/1            |
| led        | 0/1            |
| deadman    | 0/1            |

### ROS2 Bridge Node (AGX Side)

**Status:** Implemented and tested on AGX (`~/ros2_ws/src/rover_bridge/`).

Bidirectional Python bridge:
- `/cmd_vel` → `$CMD` packets to ESP32 (20 Hz)
- `$ODM` packets → `/odom` topic + TF broadcast (odom → base_link)
- `$STS` packets → `/rover_status` topic

Parameters: `serial_port` (/dev/ttyTHS1), `baud_rate`, `max_speed`, `max_yaw_rate`, `wheel_radius`, `wheel_base`, `ticks_per_rev` (2400)

### Encoder ESP32 (I2C Slave)

Separate ESP32 reads two quadrature encoders (600 PPR × 4 = 2400 counts/rev). Connected to main ESP32 via I2C (SDA=21, SCL=22, slave address 0x10). Main ESP32 polls at 20 Hz.

### Calibration (NOT yet done)

- `wheel_radius` — currently 0.05m (placeholder)
- `wheel_base` — currently 0.30m (placeholder)
- `max_speed` (m/s) and `max_yaw_rate` (rad/s) — measure after assembly

## Safety Chain

Movement requires ALL of:
1. RC link up (CRSF)
2. Armed (CH8 = 1792)
3. **MANUAL:** dead man pressed (CH9 = 1792)
   — OR —
   **AUTO:** agxAlive (valid `$CMD` within 500ms)

### Fail-Safes

| Event              | Result                                |
|--------------------|---------------------------------------|
| RC link lost       | Full stop, brakes ON, LED OFF, beacon OFF, buzzer OFF |
| Disarmed           | Full stop, brakes ON, beacon OFF      |
| AUTO + AGX timeout | Full stop, brakes ON                  |
| CH5 → MANUAL       | Instant AGX cut, dead man required, buzzer OFF |

### Operator Kill Methods (always available)
1. Disarm (CH8)
2. Flip to Manual (CH5)
3. Turn off TX (link loss)

### Dual Watchdog

- **Layer 1 — AGX bridge node**: if no `/cmd_vel` in 500ms → sends `$CMD,0,0` (stop)
- **Layer 2 — ESP32 firmware**: if no valid `$CMD` in 500ms → `agxAlive = false` → brakes ON

Both layers are independent. Even if the bridge crashes, the ESP32 watchdog catches it.

## Conventions

- Production firmware is `main_rover_pocket.cpp` (build env: `esp32dev`)
- Backup manual-only firmware is `main_rover_pocket_copy.cpp` (build env: `esp32copy`)
- CRSF values: 174–1811 range, center ~992, switch threshold 1500
- DAC range: 0–4095 (12-bit), maps to 0–3.3V
- Relay polarity controlled by `RELAY_ACTIVE_HIGH` define
- All safe defaults: brakes ON, DACs zero, relays OFF, LED OFF
- AGX serial: `$CMD,<throttle>,<yaw>\n` on Serial1, 115200 baud, 500ms watchdog
- Arm logic: `ch8 > SWITCH_THRESH` = armed (inverted from original Zorro firmware)
- Mixer mode: `MIXER_MODE 0` = asymmetric (default, `OUTER_BOOST 0.2`), `1` = scaled yaw (`YAW_SCALE 0.5`)
- Beacon (GPIO 32): ON when armed, OFF when disarmed/link lost
- Buzzer (GPIO 19): 1s ON/OFF beep in AUTO mode only (SPI MISO reassigned to -1)

## GitHub

- **Repo:** https://github.com/Akashsingh9936/rover_ESP32_pocket_final
- **Branch:** main
- **Remote:** already connected (`origin`)
- **Push workflow:**
  ```bash
  git add <files>
  git commit -m "your message"
  git push
  ```
- **gh CLI:** available at `~/.local/bin/gh`, auth may need a fresh PAT each session
- **Contributor:** only Akash Singh — do NOT add Claude as co-author in commits

## Project Status

- [x] Manual control firmware — complete, tested with RadioMaster Pocket
- [x] Autonomous ESP32 firmware (AGX serial parser, watchdog, source select)
- [x] I2C encoder integration (slave ESP32 → main ESP32, 20 Hz)
- [x] $ODM odometry sender (ESP32 → AGX, 20 Hz)
- [x] $STS status telemetry sender (ESP32 → AGX, 5 Hz)
- [x] ROS2 bridge node — bidirectional, deployed and tested on AGX
- [x] Beacon (armed indicator) and buzzer (AUTO mode alert)
- [x] Outer wheel boost (OUTER_BOOST 0.2) for better turn response
- [ ] Hardware assembly with final circuit
- [ ] Calibration (wheel_radius, wheel_base, max_speed, max_yaw_rate)
- [ ] End-to-end autonomous test with Nav2

## Related

- Original multi-file project (Zorro TX + autonomous): `TX_RX/` repo
- Bridge design doc for AGX team: `TX_RX/ESP-AGX_Bridge_Doc.md`

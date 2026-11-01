# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

IRXBot is a 4WD industrial robot with a three-layer compute architecture:

- **Jetson Orin Nano** — ROS2 Humble, Nav2/SLAM, cameras, LiDAR, audio
- **STM32 NUCLEO-F446RE** — real-time motor control via CAN, safety logic, IMU/ToF sensors, bumpers, RC override, horn, contactor enable
- **ESP32-S3** — environmental/air-quality sensors, power monitoring, WS2812B lighting, ST7789 LCD display

## Workflow

Commit and push to `main` from the PC only when asked; the user pulls on the Jetson and runs the scripts. See `docs/workflow.md`, and say after each push which of its steps are required.

## Build & Flash

### STM32 Firmware (`firmware/stm32/irx_drive/`)
Open with **STM32CubeIDE**; build produces `Debug/irx_drive.elf`.

Flash from the Jetson (or any machine with `st-stlink`):
```bash
arm-none-eabi-objcopy -O binary Debug/irx_drive.elf Debug/irx_drive.bin
st-flash --reset --connect-under-reset write Debug/irx_drive.bin 0x08000000
```

Or build and flash on the Jetson without CubeIDE:
```bash
./scripts/flash_stm32.sh
```

Check the hardware from the Jetson (never moves the motors; see `docs/hardware-check-plan.md`):
```bash
python3 scripts/robot_check.py
```

### ESP32 Firmware
- **hw_test/** — standalone Arduino sketches for per-sensor hardware validation only. Not production code.
- **Main firmware** (`firmware/esp32/irxbot_esp32/`) — **ESP-IDF + FreeRTOS** (not Arduino). One task per job: sensors, lighting, UART from STM32, telemetry to Jetson over USB serial.

Build and flash on the Jetson (needs ESP-IDF at `~/apps/esp-idf`):
```bash
./scripts/flash_esp32.sh
```

To do it by hand, activate the environment first:
```bash
. ~/apps/esp-idf/export.sh
```

Build and flash:
```bash
cd firmware/esp32/irxbot_esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
```

### ROS2 Package (`ros2/`)
```bash
./scripts/build_ros2.sh
source ros2/install/setup.bash
ros2 run irx_driver motor_driver_node
```

### Motor smoke test (serial)
```bash
python3 scripts/motor_spin.py   # sends CMD 3000 3000 to /dev/ttyACM0
```

## Architecture

### STM32 ↔ Jetson UART Protocol (USART2, 115200 8N1)

The ROS2 `irx_driver` node communicates with the STM32 over `/dev/ttyACM0`.

| Direction | Format | Meaning |
|---|---|---|
| Jetson → STM32 | `CMD <left_rpm> <right_rpm>\n` | Set wheel ERPM, ±6000. Replies `CMD_OK <l> <r>`; a partial or out-of-range command gets `ERR:CMD` and is ignored |
| Jetson → STM32 | `CAN\n` | Print CAN error state: `CAN LEC= BOFF= EPVF= EWGF= TEC= REC= TSR= MSR=` |
| Jetson → STM32 | `STATUS\n` | Hardware check: `STATUS VESC:<age1>,<age2>,<age3>,<age4> TOF:<mask> IMU:<0/1> RC:<0/1> BMP:<mask>`; ages are ms since last heard, -1 = never |
| Jetson → STM32 | `HORN <ms>\n` | Sound horn for ms (max 5000) |
| Jetson → STM32 | `LIGHT <target> <R> <G> <B> <mode>\n` | Forwarded to ESP32-S3 via USART3 |
| STM32 → Jetson | `IMU qw qx qy qz ax ay az gx gy gz\r\n` | BNO085 data @ ~3 s (debug; tune for 50 Hz) |
| STM32 → Jetson | `TOF FL:<mm> FR:<mm> L:<mm> R:<mm> REAR:<mm>\r\n` | VL53L1X distances @ 3 s (debug; tune for 10 Hz) |
| STM32 → Jetson | `BMP <FL> <FR> <RL> <RR>\r\n` | Bumper state on change (1=pressed) |
| STM32 → Jetson | `ODO <left_rpm> <right_rpm>\r\n` | Actual VESC ERPM feedback @ 20 Hz, left from VESC 2 (FL), right from VESC 1 (FR); both positive for forward motion — right side NOT negated |
| STM32 → Jetson | `RC_ON\r\n` / `RC_OFF\r\n` | RC override state change |
| STM32 → Jetson | `CAN_OK\r\n` / `ERR:CAN\r\n` | CAN init result on boot |
| STM32 → Jetson | `IMU_OK\r\n` / `ERR:IMU\r\n` | IMU init result on boot |
| STM32 → Jetson | `TOF_OK <n>/5\r\n` | ToF init result on boot |
| STM32 → Jetson | `CONTACTOR_ON\r\n` | Contactor energized on boot |

A 500 ms watchdog in the ROS2 node sends `CMD 0 0` if no `/cmd_vel` arrives.

**CMD watchdog:** STM32 resends the last CMD every 200 ms to keep VESCs alive within their 1 s timeout. `last_cmd_tick = 0` disables the resend (used when RC override takes over). If no valid CMD arrives for 1 s the STM32 stops the motors, disables the resend and sends `ERR:CMD_TIMEOUT`, so anything driving the robot must keep sending CMD (the ROS2 node's 500 ms watchdog does).

### STM32 → ESP32-S3 (USART3, PC10 TX / PC5 RX, 115200 8N1)

Also forwarded from Jetson → STM32 → ESP32-S3 for voice/navigation states.

#### LIGHT command

```
LIGHT <target> <R> <G> <B> <mode>\n
```

| Field | Values |
|---|---|
| `target` | `ALL`, `LEFT`, `RIGHT`, `FRONT`, `REAR`, `SIDE` |
| `R G B` | 0–255 each |
| `mode` | `SOLID`, `BLINK`, `PULSE` |

**Target mapping:**

| Target | Lights affected |
|---|---|
| `ALL` | All lights |
| `LEFT` | Left side strip + left front amber + left rear red |
| `RIGHT` | Right side strip + right front amber + right rear red |
| `FRONT` | Both front ambers |
| `REAR` | Both rear reds |
| `SIDE` | Both WS2812B side strips |

**Standard color conventions (not enforced by ESP32):**

| Situation | Command |
|---|---|
| Left turn signal | `LIGHT LEFT 255 90 0 BLINK` |
| Right turn signal | `LIGHT RIGHT 255 90 0 BLINK` |
| Hazard (bumper hit, RC override) | `LIGHT ALL 255 0 0 BLINK` |
| Brake / stop | `LIGHT REAR 255 0 0 SOLID` |
| Robot listening (voice) | `LIGHT ALL 0 0 255 PULSE` |
| Robot speaking | `LIGHT ALL 0 255 255 PULSE` |
| All off | `LIGHT ALL 0 0 0 SOLID` |

#### Other messages

| Format | Meaning |
|---|---|
| `SPEED <rpm>\n` | Current wheel ERPM (for speed-dependent lighting) |
| `BMP <bitmask>\n` | Bumper state (FL=1, FR=2, RL=4, RR=8) — proximity warning |

### ESP32-S3 ↔ Jetson (native USB serial)

| Direction | Format | Meaning |
|---|---|---|
| ESP32 → Jetson | `ENV key=value ...\n` | Sensor telemetry every 5 s |
| Jetson → ESP32 | `IP <address>\n` or `IP NONE\n` | Jetson IP for the LCD, every 5 s; the LCD shows `NO JETSON` after 30 s without one |

The USB port carries only these lines. ESP32 logs go to UART0, the board's other USB port.

### CAN Bus (STM32 → 4× Flipsky 75100)
- 500 kbps — prescaler=6, BS1=11TQ, BS2=2TQ on 84 MHz APB1
- VESC IDs: **FR=1, FL=2, RR=3, RL=4** (matches `robot_drive.h`)
- `vesc_set_rpm(id, rpm)` / `vesc_get_rpm(id, &rpm)` in `Core/Inc/vesc_can.h`
- `robot_set_rpm(left_rpm, right_rpm)` fans out to all four VESCs; right side is NOT negated (all motors face the same physical direction)
- CAN status messages configured at **50 Hz** (Status 1–4) in VESC Tool → App Settings → General
- Exactly **two** 120 Ω terminators, at the two physical ends of the bus. Most transceiver breakouts carry an onboard one — count them before adding more.
- GND must run in the CAN cable alongside CANH/CANL. It is a three-wire bus.

**Firmware target caveat:** the boards are Flipsky 75100 but ship flashed with the
`75_300_R2` target, which assumes high-side current shunts and phase filters that
the 75100 does not have. VESC Tool only offers `75_300_R2` because that is what the
board reports. Consequence: current sensing and phase-voltage assumptions are
slightly wrong, which shows up as unreliable FOC detection (flux linkage railing to
exactly `100.000` mWb is the tell — a clamp, not a measurement).

### RC Override (USART1, PA9 TX / PA10 RX, 115200 8N1)
- Receiver: **FlySky FS-IA10B** bound to **FS-i6X** transmitter
- Protocol: **iBUS** — 32-byte packets, interrupt-driven ring buffer (`ibus.c` / `ibus.h`)
- Connect to the **iBUS** port on the receiver (not PWM, not SBUS)
- Override active when **CH5 (index 4) > 1700**; RC_ON/RC_OFF sent on transition
- **CH1 (index 0)** = steering, **CH2 (index 1)** = throttle; deadband ±30 around 1500
- ERPM = `(thr ± str) × 6000 / 500`, capped at ±6000
- RC commands sent at **20 Hz**; no command sent when both thr=0 and str=0 (VESCs coast on 1 s timeout)
- Signal-loss timeout: 500 ms without valid packet → `valid = 0`
- **RC_OFF** fires via signal-loss timeout (turn off transmitter → 500 ms → RC_OFF). Switch-based RC_OFF not implemented — the switch position does not reliably bring CH5 below 1700 on this transmitter.

**Power-cycle note:** if only the VESCs are rebooted while the STM32 keeps running, the STM32 must also be reset (NRST button) — otherwise ToF I2C addresses desync and all sensors read 0.

### Drivetrain
- Motor: **57BL9524** (LK57BL9524) — 24 V, 100 W, 3000 RPM rated, **4 poles** (2 pole pairs), hall sensors
  - Rated **6.6 A**, rated torque **0.33 N·m**, peak torque 0.99 N·m, Kt 0.063 N·m/A, Ke 4.27 V/Krpm
  - Motor nameplate (authoritative): `24V / 100W / 3000RPM / 0.33N.m / Pole : 4`, phases U=Yellow,
    V=Green, W=Blue. Photo + datasheet in `docs/spec/motor - LK57BL9524/`.
  - **Ignore the AliExpress listing title.** It claims "0.6 N·m" and "8 Poles"; the nameplate and
    the manufacturer table independently say **0.33 N·m** and **4 poles**. The title is wrong on
    both. `极数` in the table means pole count, not "number of phases" as its English label says.
- Gearbox: 57 mm/NEMA 23 planetary, **50:1**, Ø8 input / Ø14 output
- Wheel: **250-4**, measured 201.5 mm OD loaded
- Top speed (nameplate): 3000 motor RPM ÷ 50 = 60 wheel RPM ≈ **0.63 m/s**; ERPM ceiling = 3000 × 2 = **6000**

**Unresolved — the motor may not reach 3000 RPM on 24 V.** Detected flux linkage is ~31.5 mWb
across the motors. Back-substituting (`V_ll_peak = λ·ω_e·√3` must stay under the 24 V bus) caps
mechanical speed near **2100 RPM**, i.e. ~0.44 m/s, not 0.63. Either λ detection reads high or the
3000 RPM nameplate figure assumes more than 24 V. Resolve by measurement, not arithmetic: mark the
output shaft, command a steady speed, film it and count real revolutions against VESC Tool's
displayed RPM. That same test also re-confirms the pole count — if the display reads half what you
count, the motor is 8-pole after all.

VESC current limits are derived from the 6.6 A rating, **not** from the first-pass
`57BL_VESC1_motor.xml` (which carried `l_current_max 55.07` — about 8× rated, meaningless
for this motor):

| VESC setting | Value | Basis |
|---|---|---|
| Motor current max | **10 A** | ≈1.5× rated 6.6 A; phase current, does not sum at the battery |
| Motor current brake | **−10 A** | symmetric |
| Battery current max | **5 A** | **4 VESCs × 5 A = 20 A = the pack's entire 1C continuous rating** |
| Battery current regen | **−2.5 A** | 4 × 2.5 = 10 A = the pack's max charge current (0.5C) |

Battery current limits are **per VESC but constrained by the pack**, because all four draw
simultaneously when driving straight. Each motor needs only ~3.9 A from a 25.6 V bus at its full
100 W, so 5 A/VESC gives up nothing.

### Battery pack
2 × 12.8 V 20 Ah LiFePO₄ in **series** → 25.6 V, **20 Ah** (capacity does not add in series), 512 Wh.

| Limit | Value |
|---|---|
| Max continuous discharge | **20 A (1C)** — the binding constraint on all four VESCs combined |
| Peak pulse discharge | 40 A for 3–5 s |
| Max charge current | 10 A (0.5C) — this is also the regen ceiling |
| Internal resistance | ≤40 mΩ/block, ~80 mΩ in series → ~1.6 V sag at 20 A |
| Discharge cut-off | 10.0 V/block = 20 V pack (2.5 V/cell) |
| Charge temp range | **0 °C to 45 °C**, vs −20 °C to 60 °C discharge |

VESC battery type is `BATTERY_TYPE_LIIRON_2_6__3_6` ("LIIRON" is VESC Tool's typo for LiFe) with
**8 cells**. That combination produces `l_battery_cut_start 23.2` (8 × 2.9 V) and
`l_battery_cut_end 20.8` (8 × 2.6 V) — slightly conservative against the 20 V pack cut-off.

**Regen below freezing damages LiFePO₄.** Charge range starts at 0 °C but the robot may discharge
down to −20 °C, and regen braking *is* charging. Outdoor operation needs regen disabled below 0 °C.

If the pack carries a BMS (a 20 Ah LiFePO₄ brick almost certainly does), exceeding 20 A will trip
it and cut all four motors instantly, with no software warning.

VESC sensor mode is **Hall Sensors**. A valid detected hall table has `255` at
indices 0 and 7 (states 000 and 111 are electrically impossible) and six distinct
values between. VESC Tool 6.06 can report "Bad FOC Hall Detection Result Received"
while returning a perfectly valid table — check the table itself, not the message.

### IMU (BNO085)
- I2C2: SCL=PB10, SDA=PC12 (separate bus from ToF)
- Hardware reset pin: **PC8 (IMU_RESET)** — pulled HIGH by default; STM32 pulses LOW 10 ms then HIGH before `BNO085_Init()` on every boot
- SH-2 library in `firmware/stm32/irx_drive/Core/lib/sh2/`

### ToF Sensors (5× VL53L1X)
- I2C1: SCL=PB8, SDA=PB9
- All share default address 0x29 → STM32 holds all XSHUT LOW, enables one at a time and reassigns address sequentially
- Boot: up to 5 retry attempts (200 ms apart); runtime drop recovery (re-init every 3 s, max 5 attempts)
- If a sensor fails address assignment, XSHUT is pulled LOW to prevent bus collision

| Index | Name constant | XSHUT pin | Assigned address |
|---|---|---|---|
| 0 | `TOF_RIGHT` | PB0 | 0x30 |
| 1 | `TOF_LEFT` | PA8 | 0x31 |
| 2 | `TOF_FRONT_LEFT` | PC7 | 0x32 |
| 3 | `TOF_FRONT_RIGHT` | PB6 | 0x33 |
| 4 | `TOF_REAR` | PC9 | 0x34 |

### ESP32-S3 (DevKitC-1 N16R8) Pin Assignments

| GPIO | Signal | Notes |
|---|---|---|
| 35 | UART1 RX ← STM32 PC10 (USART3 TX) | 115200 8N1 |
| 36 | UART1 TX → STM32 PC5 (USART3 RX) | 115200 8N1 |
| 8 | I2C SDA | INA228, BME280, SGP30, SPS30, VEML7700 |
| 9 | I2C SCL | |
| 38 | WS2812B side strip LEFT (23 LEDs) | |
| 39 | WS2812B side strip RIGHT (23 LEDs) | |
| 4 | DS18B20 1-Wire | Both battery and DC-DC temp sensors on same bus |
| 1 | MQ-2 ADC | ADC_11db attenuation (0–3.3 V), 3 min warmup |
| 21 | Buzzer (MOSFET) | Active HIGH |
| 10 | ST7789 CS (SPI) | |
| 11 | ST7789 MOSI (SPI) | |
| 12 | ST7789 SCK (SPI) | |
| 41 | ST7789 DC | |
| 5 | ST7789 RST | |
| 19 | USB D− | Reserved — Jetson USB serial |
| 20 | USB D+ | Reserved — Jetson USB serial |

**I2C device addresses:** INA228=0x40, BME280=0x76 (or 0x77), SPS30=0x69

| 15 | Front amber turn signal LEFT (MOSFET) | Digital |
| 16 | Front amber turn signal RIGHT (MOSFET) | Digital |
| 17 | Rear red turn signal LEFT (PWM) | analogWrite: TAIL=64, BRAKE=255 |
| 18 | Rear red turn signal RIGHT (PWM) | analogWrite: TAIL=64, BRAKE=255 |
| 14 | Spotlight (MOSFET) | Digital |

**Blink rate:** 330 ms on / 330 ms off (~90 blinks/min)

| 2 | 12V rail ADC (56k/10k divider, calibrated ratio 7.55) | ADC1_CH1, DB_12 attenuation |
| 6 | 5V rail ADC (22k/10k divider, calibrated ratio 2.73) | ADC1_CH5, DB_12 attenuation |
| 7 | 3.3V rail ADC (10k/10k divider, ratio 2.0) | ADC1_CH6, DB_12 attenuation |

**TBD pins (not yet wired/documented):** IR illuminators, panel buttons, rear light

**GPIO 26–32** — internal octal flash, do not use. **GPIO 33–37** — internal PSRAM (R8 variant), do not use. **GPIO 46** — strapping pin, must be LOW at boot.

### Safety chain (hardware — no software in path)
```
24V control → rocker/key → E-stop NC → MOSFET board VIN+ → (PB12-controlled MOSFET) → OUT+ → A1 → coil → A2 → GND
```
STM32 enable is a normally-off stage: MCU active = coil energized; IWDG reset / power loss = contactor drops. STM32 can refuse to energize but cannot hold the contactor closed against an open E-stop.

**Contactor enable: PB12 (`CONTACTOR_EN`)** — GPIO Output, default LOW. Set HIGH after all peripherals initialise; sends `CONTACTOR_ON\r\n` on boot.

**Contactor wiring (HORNAXYS XY-MOS MOSFET board):**
- VIN+ → 24 V (after E-stop NC)
- VIN− → GND
- GND → GND
- TRIG → STM32 PB12
- OUT+ → Contactor coil A1 (+)
- A2 (−) → GND directly
- Flyback diode: cathode (stripe) → A1, anode (plain) → A2/GND

**IWDG:** direct register access (no HAL module — not included in project). Prescaler /32, reload 1500 → ~1.5 s timeout at LSI ~32 kHz. Started after `IBus_Init()`, before entering the main loop. Kicked at the top of every loop iteration: `IWDG->KR = 0xAAAAU`.

## Key Source Locations

| Path | Contents |
|---|---|
| `firmware/stm32/irx_drive/Core/Src/main.c` | STM32 main loop: CAN, UART, IMU, ToF, bumpers, RC, horn |
| `firmware/stm32/irx_drive/Core/Src/ibus.c` | iBUS RC receiver driver (interrupt-driven ring buffer) |
| `firmware/stm32/irx_drive/Core/Inc/ibus.h` | iBUS API + channel/threshold defines |
| `firmware/stm32/irx_drive/Core/Inc/vesc_can.h` | VESC CAN API |
| `firmware/stm32/irx_drive/Core/Inc/robot_drive.h` | `robot_set_rpm()` |
| `firmware/stm32/irx_drive/Core/Inc/bno085.h` | BNO085 IMU driver |
| `firmware/stm32/irx_drive/Core/Inc/vl53l1x.h` | VL53L1X ToF driver |
| `firmware/stm32/lib/sh2/` | Hillcrest SH-2 library for BNO085 |
| `ros2/src/irx_driver/src/motor_driver_node.cpp` | ROS2 node: `/cmd_vel` → serial → STM32, publishes `/odom` + TF |
| `firmware/esp32/hw_test/` | Per-sensor Arduino validation sketches |
| `firmware/esp32/irxbot_esp32/main/` | ESP32-S3 main firmware (ESP-IDF + FreeRTOS) |
| `docs/robot-spec.md` | Authoritative hardware component list |
| `firmware/config/vesc/` | VESC motor config XML |

## ROS2 Node Parameters

| Parameter | Default | Notes |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | STM32 USB-serial |
| `baud_rate` | `115200` | |
| `wheel_base` | `0.400` m | Track width (left↔right wheel centres), **not** front-to-rear |
| `wheel_radius` | `0.101` m | Measured 250-4 tyre, 201.5 mm OD loaded |
| `max_rpm` | `6000` | Motor ERPM ceiling = 3000 RPM × 2 pole pairs |
| `gear_ratio` | `50.0` | 50:1 planetary gearbox |
| `pole_pairs` | `2.0` | 57BL9524 nameplate says `Pole : 4`; ERPM = mechanical RPM × 2 |

Drivetrain conversion, used in both directions:
```
ERPM = (m/s ÷ wheel_circumference) × 60 × pole_pairs × gear_ratio
```

`wheel_base` is named for the ROS convention but is used as the differential-drive
track width. On a 4WD skid-steer the *effective* value exceeds the geometric one
because the wheels scrub when turning — if commanded rotations come up short,
increase it rather than assuming the measurement is wrong.

Odometry is dead-reckoned from `ODO` feedback; `odom → base_link` TF is broadcast each cycle.

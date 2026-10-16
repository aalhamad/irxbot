# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository Overview

IRXBot is a 4WD industrial robot with a three-layer compute architecture:

- **Jetson Orin Nano** — ROS2 Humble, Nav2/SLAM, cameras, LiDAR, GNSS/UWB, audio
- **STM32 NUCLEO-F446RE** — real-time motor control via CAN, safety logic, IMU/ToF sensors, bumpers, RC override
- **ESP32-S3** — environmental/air-quality sensors, power monitoring, WS2812B lighting, ST7789 LCD display

## Build & Flash

### STM32 Firmware (`firmware/stm32/irx_drive/`)
Open with **STM32CubeIDE**; build produces `Debug/irx_drive.elf`.

Flash from the Jetson (or any machine with `st-stlink`):
```bash
arm-none-eabi-objcopy -O binary Debug/irx_drive.elf Debug/irx_drive.bin
st-flash --reset --connect-under-reset write Debug/irx_drive.bin 0x08000000
```

Deploy ELF from dev machine to Jetson then flash remotely:
```bash
./scripts/deploy.sh
```

### ESP32 Firmware (`firmware/esp32/hw_test/`)
Each subdirectory is a standalone Arduino sketch for hardware validation. Open in **Arduino IDE** and flash via USB.

### ROS2 Package (`ros2/`)
```bash
cd ros2
colcon build --packages-select irx_driver
source install/setup.bash
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
| Jetson → STM32 | `CMD <left_rpm> <right_rpm>\n` | Set wheel RPM |
| STM32 → Jetson | `IMU qw qx qy qz ax ay az gx gy gz\r\n` | BNO085 data @ 50 Hz |
| STM32 → Jetson | `TOF <front> <rear> <left> <right>\r\n` | VL53L1X distances (mm) @ 10 Hz |
| STM32 → Jetson | `BMP <bitmask>\r\n` | Bumper state (bits: FL=1, FR=2, RL=4, RR=8) |
| STM32 → Jetson | `ODO <left_rpm> <right_rpm>\r\n` | Actual VESC feedback |

A 500 ms watchdog in the ROS2 node sends `CMD 0 0` if no `/cmd_vel` arrives.

### CAN Bus (STM32 → 4× Flipsky 75100)
- 500 kbps — prescaler=6, BS1=11TQ, BS2=2TQ on 84 MHz APB1
- VESC IDs: FL=1, FR=2, RL=3, RR=4
- `vesc_set_rpm(id, rpm)` / `vesc_get_rpm(id, &rpm)` in `Core/Inc/vesc_can.h`
- `robot_set_rpm(left_rpm, right_rpm)` fans out to all four VESCs
- Exactly **two** 120 Ω terminators, at the two physical ends of the bus. Most
  transceiver breakouts carry an onboard one — count them before adding more.
- GND must run in the CAN cable alongside CANH/CANL. It is a three-wire bus.

**Firmware target caveat:** the boards are Flipsky 75100 but ship flashed with the
`75_300_R2` target, which assumes high-side current shunts and phase filters that
the 75100 does not have. VESC Tool only offers `75_300_R2` because that is what the
board reports. Consequence: current sensing and phase-voltage assumptions are
slightly wrong, which shows up as unreliable FOC detection (flux linkage railing to
exactly `100.000` mWb is the tell — a clamp, not a measurement).

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

### ToF Sensor Init (I2C address conflict workaround)
All four VL53L1X share the default I2C address. STM32 holds all XSHUT pins low, then enables them one at a time and reassigns addresses sequentially — no multiplexer needed. XSHUT pins: FRONT=PB0, REAR=PA8, LEFT=PC7, RIGHT=PB6.

### STM32 → ESP32-S3 (dedicated UART)
Turn-signal and hazard commands are sent from STM32 to ESP32-S3 over a separate UART. ESP32-S3 reports sensor telemetry to Jetson over USB serial.

### Safety chain (hardware — no software in path)
```
24V control → rocker/key → E-stop NC → contactor coil → GND
```
STM32 enable is a normally-off stage: MCU active = coil energized; IWDG reset / power loss = contactor drops. STM32 can refuse to energize but cannot hold the contactor closed against an open E-stop.

## Key Source Locations

| Path | Contents |
|---|---|
| `firmware/stm32/irx_drive/Core/Src/main.c` | STM32 main loop: CAN, UART, IMU, ToF, bumpers |
| `firmware/stm32/irx_drive/Core/Inc/vesc_can.h` | VESC CAN API |
| `firmware/stm32/irx_drive/Core/Inc/robot_drive.h` | `robot_set_rpm()` |
| `firmware/stm32/irx_drive/Core/Inc/bno085.h` | BNO085 IMU driver |
| `firmware/stm32/irx_drive/Core/Inc/vl53l1x.h` | VL53L1X ToF driver |
| `firmware/stm32/lib/sh2/` | Hillcrest SH-2 library for BNO085 |
| `ros2/src/irx_driver/src/motor_driver_node.cpp` | ROS2 node: `/cmd_vel` → serial → STM32, publishes `/odom` + TF |
| `firmware/esp32/hw_test/` | Per-sensor Arduino validation sketches |
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

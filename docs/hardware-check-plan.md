# Hardware Check, Rewiring and Dashboard Plan

Decisions from the 2026-11-01 discussion.

## Decisions

- **Hardware check first.** `scripts/robot_check.py` runs on the Jetson and checks that parts answer and that their readings make sense.
- **The dashboard comes after rewiring and ROS2 bring-up.** It needs data that does not exist yet and is the wrong tool for finding a loose wire.
- **UWB, GNSS and the Bluetooth gamepad are removed.** This robot has none of them.
- **4G setup waits until the rewiring is finished.**

## Order of work

1. Compile and flash the `STATUS` firmware.
2. Run `python3 scripts/robot_check.py --manual --baseline` and correct any script errors from the first real output.
3. Photograph and label the wires.
4. Rewire with the power off, one section at a time: power, CAN, STM32 sensors, ESP32 sensors, USB.
5. Run the script after each section.
6. Bring up the sensors and cameras in ROS2.
7. View everything in Foxglove.
8. Build the custom dashboard if the look is wanted, starting with the System screen as the debug screen.

## First results on the robot

From one run on the old STM32 firmware, before any rewiring.

| Part | Result | Next |
|---|---|---|
| ToF right, front right, rear | Often read 0 mm, with valid readings in between | Not a wiring fault: 0 means the reading was invalid. Test with a hand using `--manual`, and find out why so many readings are invalid |
| BME280 | Not responding | Check its wires and its I²C address (0x76 or 0x77) |
| DC-DC temperature | -127; only one DS18B20 found | Check the second sensor |
| PM2.5 / PM10 | 282 / 304 µg/m³ | Real dust or smoke, or a faulty SPS30 |
| LiDAR | No reply | Confirm it is plugged in, then check the connection settings |
| ToF front left | 17 mm | Check nothing is blocking it |
| Bus current | 0.116 A | Confirm what the INA228 measures |
| SGP30 | 0 VOC, 400 eCO₂ | See if the readings change after it has been on a while |
| Wi-Fi | 44% signal | Recheck with the robot closed |

The 4G modem and RealSense also failed, probably because they are not plugged in. The thermal camera failed because it is not wired yet.

A second run changed some of these. BME280 and the second DS18B20 now pass, so the first failures may only mean the ESP32 had just started. Watch them for repeats. PM2.5 / PM10 fell to 194 / 199 µg/m³, still very high. VEML7700 reported 3 failed reads since boot, then 5, which may be a loose wire.

A third run, with the `STATUS` firmware, heard all 4 VESCs.

## Running the check

Stop the ROS2 nodes first, because they hold the serial ports.

| Command | Use |
|---|---|
| `python3 scripts/robot_check.py` | Automatic checks |
| `python3 scripts/robot_check.py --manual` | Adds hand-in-front-of-ToF, bumper, RC, E-stop, horn, light, LCD and speaker checks |
| `python3 scripts/robot_check.py --baseline` | Also saves this run as the reference; later runs list anything worse |
| `python3 scripts/robot_check.py --only stm32` | Checks only that board and what is wired to it: `stm32`, `esp32` or `jetson`, or several, such as `--only stm32 esp32`. Works with `--manual` and `--baseline`; a partial `--baseline` keeps the other boards' saved results |
| `python3 scripts/robot_check.py --spin` | Spins the wheels instead of checking, wheels off the ground; `--spin [left] [right] --time s` sets speed and time |

Only `--spin` sends `CMD`; without it the script cannot move the motors.

Each run also saves its output to `scripts/robot_check.log`, replacing the last run. Git ignores this file, so it stays on the Jetson. To compare against an earlier run, use `--baseline`.

## Coverage gaps

| Part | Status |
|---|---|
| 4 motors and hall sensors | Not checked automatically. `--spin` turns all 4 together and shows front-wheel speed feedback; watch each wheel. A test of one motor at a time is not written |
| 360° camera | Needs its USB name set in `CAMERA_360_NAME` |
| Spotlight | Firmware has no command for it |
| Pan/tilt, laser, E-stop and contactor aux contacts, panel buttons, IR lights, rear light, hazard flasher | No firmware support yet |
| Buzzer | Only sounds at startup; listen at power-on |
| Watchdog (IWDG) | Needs a special firmware build that hangs on purpose |
| Fuses, diodes, TVS protection, CAN resistance (about 60 Ω) | Check by hand |

Passing means a reading is believable, not that it is accurate. For example, the dust sensor can pass without ever being exposed to smoke.

## Not verified yet

- The script has only run against a simulated STM32, not the robot.
- The `STATUS` firmware has not been compiled or flashed.
- USB IDs, the LiDAR baud rate and the thermal camera's data format come from general knowledge of these parts, not this hardware.

## Open spec conflict

`docs/robot-spec.md` lists 4 ToF sensors, including a downward one. The firmware and `CLAUDE.md` have 5, with no downward one. The script tests the 5.

## Cameras

**360° camera:** 2 MP single-lens fisheye USB webcam module. Mount it high, pointing straight up. It sees a wide dome, not a full sphere, and its picture is a distorted circle that needs software to unwrap into a panorama.

**Thermal camera:** MLX90640BAA module, 32×24 pixels, so the picture is blocky. It can find hot areas but may miss small or distant ones. It looks like a GY-MCU90640 type board, which is not yet confirmed.

Planned wiring through a USB-serial adapter set to 3.3 V, not done yet:

| Thermal board | Adapter |
|---|---|
| `VIN` | `3.3V` (`5V` only if there is no 3.3 V pin) |
| `GND` | `GND` |
| `TX` | `RX` |
| `RX` | `TX` |

Leave `SCL`, `SDA`, `NC` and `PS` unconnected, and do not bridge the `SET I2C` pads.

## Dashboard notes

Foxglove comes first because it is much quicker. A custom dashboard would follow the IRX mockups; the Fleetly picture is for style only.

Corrections the mockups need:

| In the design | On the real robot |
|---|---|
| Max speed 2.0 m/s | About 0.63 m/s at best, possibly 0.44 m/s |
| VESC current and temperature | Firmware reads speed only |
| VESC 1 = front left | VESC 1 = front right |
| Rear camera button | No rear camera; the 360° camera covers the rear |
| Inspection camera with zoom | Not in the spec |
| On-screen E-stop | Software stop only; the red hardware E-stop stays the safety device |
| Detailed thermal picture | 32×24 pixels |
| Flat 360° panorama | Needs unwrapping software |

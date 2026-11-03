# Development Workflow

Code is written on the PC and runs on the Jetson, STM32 and ESP32.

## Steps

| Step | Where | What |
|---|---|---|
| 1 | PC | Say "commit and push"; the work goes to `main` on GitHub, with no branches or pull requests |
| 2 | Jetson | `cd ~/irxbot && git pull` |
| 3 | Jetson | `./scripts/flash_stm32.sh`, if required |
| 4 | Jetson | `./scripts/flash_esp32.sh`, if required |
| 5 | Jetson | `./scripts/build_ros2.sh`, if required |
| 6 | Jetson | `python3 scripts/robot_check.py`, if required |

Each push comes with a note saying which of steps 3 to 6 are required.

## Why a pull is not enough

`git pull` only updates the source files on the Jetson. Each part keeps running its old version until the new code is put on it.

| Changed | Step | What it does |
|---|---|---|
| `firmware/stm32/` | `flash_stm32.sh` | Builds the firmware and writes it to the STM32 |
| `firmware/esp32/` | `flash_esp32.sh` | Builds the firmware and writes it to the ESP32 |
| `ros2/` | `build_ros2.sh` | Compiles the ROS2 nodes; restart any running nodes afterwards |
| Docs or `robot_check.py` | None | The pull is enough |

## Before running them

- **Serial ports:** stop ROS2 nodes and `minicom` first, because only one program can use a port at a time. The flash scripts stop without flashing if the port is busy. `robot_check.py` reports the port as in use.
- **Boot service:** if `irx-esp32-telemetry` is installed, stop it with `sudo systemctl stop irx-esp32-telemetry` before `flash_esp32.sh` or `robot_check.py`.
- **STM32 flash:** it resets the STM32, so the contactor clicks off and on. Keep the RC transmitter off and the E-stop within reach.
- **ESP32 flash:** it needs ESP-IDF at `~/apps/esp-idf` on the Jetson.

## Scripts

| Script | Use |
|---|---|
| `scripts/flash_stm32.sh` | Build and flash the STM32; builds in `firmware/stm32/irx_drive/build/` and keeps the build before it as `previous.bin` there |
| `scripts/flash_esp32.sh` | Build and flash the ESP32 |
| `scripts/build_ros2.sh` | Build the ROS2 code |
| `scripts/robot_check.py` | Check the hardware; add `--manual` for hands-on checks and `--baseline` to save a reference. See `docs/hardware-check-plan.md`. `--spin [left] [right] [--time s]` spins the wheels instead: wheels off the ground; it asks first and is limited to ERPM 3000 and 60 s |
| `scripts/irx-esp32-telemetry.service` | Starts the ESP32 telemetry node at boot, so the LCD shows the Jetson's IP. Install steps are at the top of the file |

## Current state

- **Run on the robot:** `robot_check.py`, once, on the old STM32 firmware.
- **Stopped by a size limit that was too high:** `flash_stm32.sh`. The limit is fixed on the PC but not yet pushed, and the `STATUS` firmware is not yet flashed.
- **Never run:** `flash_esp32.sh`, `build_ros2.sh` and the boot service.
- **Never compiled:** the ESP32 firmware that shows the IP on the LCD. ESP-IDF is not on the PC, and its location on the Jetson is unconfirmed.

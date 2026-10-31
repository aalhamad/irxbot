# IRX Robot — Complete Component List (Current Design)

Rev 2026-10-11 · Source of truth for hardware

---

## 1. Jetson

| Component | Purpose |
|---|---|
| Jetson Orin Nano | Main AI computer |
| Dedicated isolated 24V→19V (or required Vin) DC/DC supply for Jetson | Own power rail, not a shared general-purpose converter |
| Jetson Branch Fuse | Protect Jetson power wiring |
| Powered USB Hub (+ supply) | USB expansion; prevents voltage-drop issues |
| Remote Dashboard (Foxglove, over Wi-Fi/Ethernet) | Map, cameras, LiDAR, nav state, ROS2 diagnostics — viewed on laptop/tablet/phone (no on-robot Jetson display) |
| ReSpeaker USB Mic Array | Voice input |
| TPA3116D2 Amplifier | Audio output |
| 2 × Passive Speakers | Voice/audio (stereo pair, front enclosure) |
| Front RGB-D Camera (RealSense D435i) | AI vision & depth (own IMU for VIO) |
| Front Thermal Camera | Fire/heat detection |
| 360° Camera | Situational awareness (also covers rear view) |
| 2D LiDAR (RPLIDAR C1) | SLAM / navigation / obstacle detection |
| GNSS Receiver (u-blox NEO-M8N) | Outdoor localization (USB/UART; not RTK/cm-level) |
| UWB Tag | Indoor positioning (USB/UART/network to Jetson) |
| Bluetooth Gamepad | Manual teleop / testing |
| Ethernet | Networking |
| Wi-Fi | Networking |

## 2. STM32

| Component | Purpose |
|---|---|
| STM32 NUCLEO-F446RE | Real-time controller |
| STM32 Independent Watchdog (IWDG) | Resets STM32 if firmware stops responding → enable output drops → contactor drops |
| CAN Bus | VESC communication |
| SN65HVD230 CAN Transceiver | STM32 CAN interface |
| 4 × VESC 75100 | Motor controllers |
| 4 × BLDC Motors | Drive motors |
| E-Stop Auxiliary Contact | Status/feedback only (does not gate power) |
| Bumper Switches | Collision detection |
| BNO085 IMU | Chassis orientation / rollover (I2C1, UART-forwarded to Jetson) |
| 5 × VL53L1X ToF | Close obstacle / cliff detection |
| 5 × XSHUT GPIO connections | Sequential ToF init & address assignment |
| RC Receiver (FlySky FS-IA10B, iBUS protocol, 5V regulated supply) | Manual override; bound to FS-i6X transmitter |
| Pan/Tilt Servos (×2, 6V regulated servo supply) | Laser aiming |
| Laser Module (Class 2, <1 mW; MOSFET enable + hardware inhibit) | Object pointing |
| Horn (MOSFET/relay + suppression + branch fuse) | Immediate safety warning |
| Turn-Signal Command → ESP32-S3 (dedicated UART) | Sends L/R/hazard |
| Contactor Coil Enable (normally-off HW stage) | In series with hardware chain; cannot override E-stop |

## 3. ESP32-S3

| Component | Purpose |
|---|---|
| ESP32-S3 DevKitC-1 (N16R8) | Sensor, environmental, power-monitoring & lighting controller |
| WS2812B LEDs (+ 74AHCT125/74HCT14 level shifter, 330Ω series, bulk cap, common GND) | Robot lighting |
| Turn-Signal Lamps (L/R/hazard, MOSFET drivers) | Driven on UART command from STM32 |
| Sensor telemetry → Jetson (USB serial) | Sends environmental/power data up |
| Buzzer (12V, MOSFET + branch fuse) | Startup beeps / status tones (non-critical) |
| Panel Push Buttons (1–2) | Mode / start-stop |
| ST7789 2.4" TFT LCD (SPI: CS 10, DC 41, RST 5, MOSI 11, SCK 12) | Sole on-robot display — battery V/I/%, temperatures, air/gas, faults, mode, Jetson link status; works with Jetson off |

## 4. Sensors

| Sensor | MCU | Internal / External |
|---|---|---|
| BNO085 IMU | STM32 | Internal |
| Front ToF | STM32 | External |
| Left ToF | STM32 | External |
| Right ToF | STM32 | External |
| Downward ToF | STM32 | External |
| INA228 (also the 24V bus voltage source) | ESP32-S3 | Internal |
| 12V Rail Voltage Divider (56k/10k, ratio 6.6, 100nF at ADC) | ESP32-S3 | Internal |
| 5V Rail Voltage Divider (22k/10k, ratio 3.2, 100nF at ADC) | ESP32-S3 | Internal |
| 3.3V Rail Voltage Divider (10k/10k, ratio 2.0, 100nF at ADC) | ESP32-S3 | Internal |
| DS18B20 Battery | ESP32-S3 | Internal |
| DS18B20 DC-DC | ESP32-S3 | Internal |
| MQ-2 Smoke/Gas | ESP32-S3 | Internal (enclosure; not a certified alarm) |
| SPS30 PM2.5/Smoke Sensor | ESP32-S3 | External (unobstructed inlet/outlet) |
| SGP30 VOC/eCO₂ | ESP32-S3 | External (ventilated chamber; eCO₂ is estimated) |
| BME280 Temp/Humidity/Pressure | ESP32-S3 | External (chamber; humidity → SGP30 compensation) |
| Light Sensor (VEML7700) | ESP32-S3 | External (separate optical window, faces ambient) |
| Front RGB-D Camera | Jetson | External |
| Front Thermal Camera | Jetson | External |
| 360° Camera | Jetson | External |
| ReSpeaker Microphone | Jetson | External |
| 2D LiDAR | Jetson | External |
| GNSS Receiver | Jetson | External |
| UWB Tag | Jetson | External |
| Bumper Switches | STM32 | External |

**ToF init:** all 5 share default address 0x29 → STM32 holds all XSHUT LOW, enables one at a time, reassigns address sequentially (0x30–0x34). No TCA9548A needed.

**External air-sampling assembly:** SPS30 (unobstructed inlet/outlet) + SGP30 & BME280 (ventilated protected chamber). Rain hood, insect mesh, splash protection; away from battery/VESC/converter/fan exhaust/motor heat.

**Separate optical location:** VEML7700 behind a clear window facing ambient light — not in the air duct or behind dark mesh.

## 5. Lighting

| Component | MCU | Driver |
|---|---|---|
| Spotlight (GPIO 14) | ESP32-S3 | MOSFET + branch fuse |
| Rear Light | ESP32-S3 | MOSFET + branch fuse |
| Left Side Light | ESP32-S3 | MOSFET + branch fuse |
| Right Side Light | ESP32-S3 | MOSFET + branch fuse |
| Turn Signals | ESP32-S3 | MOSFET drivers |
| IR Illuminators (850/940 nm) | ESP32-S3 | MOSFET drivers |
| WS2812B LED Strip(s) | ESP32-S3 | Level shifter + 330Ω + bulk cap |

**IR note:** control separately; test for D435i active-depth interference (850nm = fainter glow + stronger camera response; 940nm = less visible, lower sensitivity).

## 6. Power System

- 2 × 12.8 V LiFePO₄ Batteries
- XT90 Main Disconnect (anti-spark — see precharge note)
- Main Fuse
- 24 V Contactor
- Contactor Auxiliary Contact — confirms actual contactor state
- Flyback Diode — main contactor coil
- Flyback Diode — safety relay coil (if fitted)
- Flyback suppression — aux relays / inductive horn
- TVS Diode (SMBJ30A, 24V bus)
- CAN TVS protection (per external CAN interface)
- USB surge/ESD protection on external USB ports (optional; recommended for outdoor use)
- Precharge arrangement (if required — depends on XT90 anti-spark + VESC input capacitance)
- Positive / Negative Busbars
- Mean Well DDR-120B-24 (16.8–33.6V in, 24V/5A out) → sensors, electronics, sensitive accessories (not VESCs)
- DDR-120B-24 Branch Fuse
- Mean Well DDR-120B-12 (16.8–33.6V in, 12V/10A out) → lighting/peripherals
- DDR-120B-12 Branch Fuse
- 24→5 V Converter → logic
- 24→5 V Converter Branch Fuse
- Jetson Branch Fuse
- 4 × Individual VESC branch fuses from the raw battery bus (each VESC separately fused, not one fuse for all four)
- Voltage dividers: 12V (56k/10k) / 5V (22k/10k) / 3.3V (10k/10k), each with 100nF at the ADC pin, on ESP32-S3 ADC1 (24V bus measured by INA228 — no 24V divider)
- Terminal Blocks, Branch Fuse Holders, Wiring Ducts, DIN Rails

## 7. Communication

- **UART:** Jetson ↔ STM32 (also carries BNO085 data up to Jetson)
- **USB serial:** Jetson ↔ ESP32-S3
- **UART (dedicated):** STM32 → ESP32-S3 — turn-signal / hazard commands
- **CAN Bus:** SN65HVD230 transceiver · 2 × 120 Ω terminators (physical ends only → ~60 Ω across H-L, power off) · twisted-pair (CAN-H/CAN-L) · common ground/reference (CAN TVS listed under Power/Protection)
- I²C, Ethernet, Wi-Fi

## 8. Audio

| Component | MCU |
|---|---|
| ReSpeaker USB Mic | Jetson |
| TPA3116D2 Amplifier | Jetson |
| 2 × Speakers (stereo pair, front enclosure) | Jetson |
| Buzzer (12V) | ESP32-S3 |
| Horn (12V) | STM32 |

## 9. Safety

- Emergency Stop (NC) — directly interrupts contactor/relay coil circuit; independent of STM32
- E-Stop Auxiliary Contact — to STM32 for status/feedback only
- Main Contactor — hardware power cut
- Contactor Auxiliary Contact — confirms actual state to STM32
- STM32 IWDG — resets the MCU if firmware hangs; contactor enable output de-asserts, causing the contactor to drop
- Safety Relay / Redundant Contactor Control (only if a safety-rated relay is actually installed)
- Main Fuse / Branch Fuses
- Bumper Switches
- 4 × ToF (close-range hazard)
- Horn
- Polycarbonate Battery Cover
- Hazard-flasher fallback (planned) — independent STM32-triggered 555/relay flasher so hazards work even if ESP32-S3 is dead

**E-stop coil chain (hardware, no software in the path):**

24 V control supply → rocker/key switch → E-stop NC → safety relay / contact interlock → contactor coil → ground

- STM32 enable is a normally-off stage: MCU active → coil chain energizes; MCU reset/power-loss/disconnect/IWDG-reset → contactor drops.
- STM32 can refuse to energize but cannot hold the contactor closed if the E-stop opens.

## 10. Positioning

- GNSS Receiver (NEO-M8N) — outdoor, USB/UART to Jetson (not RTK)
- UWB Tag (on robot) — USB/UART/network to Jetson
- UWB Anchors (3–6, fixed & powered in environment, off-robot)

---

## Final Controller Responsibility

**Jetson:** AI / ROS2 · cameras (RGB-D, thermal, 360°) · LiDAR · GNSS/UWB · audio/voice · nav/SLAM · object detection · remote Foxglove dashboard

**STM32:** Motor control/CAN · safety logic (contactor enable, not e-stop authority) · IWDG · e-stop + contactor status readback · bumpers · IMU · ToF · RC override · pan/tilt + laser · horn · turn-signal command (UART → ESP32-S3)

**ESP32-S3:** Environmental, air-quality and power-monitoring sensors · WS2812B/spotlights/IR/turn-signal lamps · buzzer · panel buttons · local status LCD (ST7789) · sensor telemetry → Jetson

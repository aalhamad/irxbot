# IRXBot To-Do

## Now

### Jetson connection
- [ ] Jetson unreachable at `192.168.8.191`: check it is on and get its IP with `hostname -I`

### SIM / 4G (Quectel EC25-EUX)
- [ ] Fit SIM and antenna (`MAIN`, plus `DIV` if available)
- [ ] Connect modem to Jetson through the powered USB hub
- [ ] Turn on `ModemManager` and confirm the modem appears in `mmcli -L`
- [ ] Add rules so `ModemManager` ignores the STM32 and ESP32. **Do not drive while it is on without these**
- [ ] Set up the connection (needs carrier APN and SIM PIN status)
- [ ] Decide what it is for: remote access, Foxglove away from Wi-Fi, or internet only

## Next

### Audio
- [x] Confirm the ReSpeaker has a 3.5 mm output
- [x] Power the XH-M139 from the 12 V rail through a 5 A fuse, never the raw battery bus
- [x] Wire ReSpeaker to amp to speakers. Leave the subwoofer output empty and never connect a speaker `−` to ground
- [x] Run the record and playback tests on the Jetson (`plughw:ArrayUAC10,0`)
- [ ] Confirm both speakers play: run `speaker-test -D plughw:ArrayUAC10,0 -c 2 -t wav -l 2` and listen to each one
- [ ] Check for motor noise in the speakers and microphone while driving (the audio cable is unshielded)
- [ ] Mount the speakers and amp inside the robot
- [ ] Write the voice command software (none exists yet)

### ESP32
- [ ] Connect the native `USB` port (`303a:1001`) to the Jetson; leave the 5V pin and other USB port empty
- [ ] Power the LED strips from the 5 V rail, not through the ESP32 board
- [ ] Set up fixed port names for the STM32 and ESP32

### Close up the robot
- [ ] Tug-test the front left VESC CAN connector
- [ ] Strap down the USB cables
- [ ] Confirm the E-stop can be reached with the robot closed
- [ ] Check Wi-Fi signal with the robot closed (`nmcli device wifi list`, `SIGNAL` above about 50)

### Driving
- [ ] Measure the first drive against `/odom` (it reported 3.04 m forward, 0.94 m right): did it really veer?
- [ ] If it veers, find out why the front right wheel is slow to start
- [ ] Confirm the node's single `ERR:CMD_TIMEOUT` was left over from testing and does not repeat

## Later
- [ ] Publish IMU and ToF as ROS2 topics at proper rates (still every 3 s)
- [ ] Reinstall `sllidar_ros2` for the LiDAR
- [ ] Set up SLAM, then Nav2
- [ ] Disable regen below 0 °C for outdoor use
- [ ] Fix `CLAUDE.md` or restore `scripts/` (`deploy.sh` and `motor_spin.py` no longer exist)
- [ ] Decide what to do with `docs/Robot_Spec_2026.md` (conflicts with the hardware: 4 ToF, SBUS)
- [ ] Delete `~/irx_fw_diag` and `~/irx_test.log` on the Jetson

## Only if needed
- [ ] Wi-Fi upgrade if the signal is weak when closed: try external antennas on the built-in card first, then the AC1200 adapter (kept in the parts box, not in the spec)

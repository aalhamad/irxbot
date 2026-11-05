#!/usr/bin/env python3
"""Check every part in docs/robot-spec.md and print one result per part.

Run on the Jetson with the ROS2 nodes stopped (they hold the serial ports):

    python3 scripts/robot_check.py              # automatic checks, about 30 s
    python3 scripts/robot_check.py --manual     # also asks you to press, wave, listen
    python3 scripts/robot_check.py --baseline   # also save this run as the reference
    python3 scripts/robot_check.py --only stm32         # just one board: stm32, esp32, jetson
    python3 scripts/robot_check.py --only stm32 esp32   # or several

Every run lists anything that got worse since the saved reference. Needs only
the Python standard library.

These checks never send CMD, so they cannot move the motors. Exit code is 1 if
anything FAILs.

Spinning the wheels is a separate job, run instead of the checks. Wheels must be
off the ground; it asks first, and Ctrl+C stops the motors:

    python3 scripts/robot_check.py --spin                 # all wheels, ERPM 1000, 5 s
    python3 scripts/robot_check.py --spin 1500            # both sides at ERPM 1500
    python3 scripts/robot_check.py --spin 1000 -1000      # left, right
    python3 scripts/robot_check.py --spin 1000 --time 10  # for 10 s
"""
import argparse
import array
import glob
import json
import math
import os
import re
import select
import shutil
import subprocess
import sys
import tempfile
import termios
import time

# Fill these in when the models are known. Text is matched against the camera's
# name, shown by: v4l2-ctl --list-devices
CAMERA_360_NAME = None

STM32_VENDOR = "0483"
ESP32_ID = "303a:1001"
RESPEAKER_ID = "2886:0018"
MODEM_ID = "2c7c:0125"
REALSENSE_ID = "8086:0b3a"
USB_SERIAL_VENDORS = ("10c4", "1a86", "0403")  # CP210x, CH340, FTDI adapters
THERMAL_HEADER = bytes((0x5A, 0x5A, 0x02, 0x06))
THERMAL_PIXELS = 32 * 24
THERMAL_BAUDS = (115200, 460800, 9600)

ALSA_DEVICE = "plughw:ArrayUAC10,0"
BASELINE_FILE = os.path.expanduser("~/.irx_check_baseline.json")

OK, WARN, FAIL, SETUP, NOT_BUILT, NOT_TESTED = (
    "OK", "WARN", "FAIL", "SETUP", "NOT BUILT", "NOT TESTED")

# Index in the firmware's TOF mask, label in the "TOF" line, place on the robot
TOF_SENSORS = (
    (0, "R", "right"),
    (1, "L", "left"),
    (2, "FL", "front left"),
    (3, "FR", "front right"),
    (4, "REAR", "rear"),
)
VESCS = ((1, "front right"), (2, "front left"), (3, "rear right"), (4, "rear left"))
BUMPERS = ("front left", "front right", "rear left", "rear right")
VESC_SILENT_MS = 1000

# Field, name, lowest and highest believable value
ENV_LIMITS = (
    ("vbat", "24 V bus (INA228)", 20.0, 30.0),
    ("i", "Bus current (INA228)", -20.0, 25.0),
    ("v12", "12 V rail", 11.0, 13.0),
    ("v5", "5 V rail", 4.5, 5.5),
    ("v33", "3.3 V rail", 3.0, 3.6),
    ("t", "Air temperature (BME280)", -20.0, 60.0),
    ("h", "Humidity (BME280)", 1.0, 99.0),
    ("p", "Air pressure (BME280)", 850.0, 1100.0),
    ("tvoc", "VOC (SGP30)", 0.0, 60000.0),
    ("eco2", "eCO2 (SGP30)", 400.0, 60000.0),
    ("pm25", "PM2.5 (SPS30)", 0.0, 1000.0),
    ("pm10", "PM10 (SPS30)", 0.0, 1000.0),
    ("lux", "Light (VEML7700)", 0.0, 120000.0),
    ("tb", "Battery temperature (DS18B20)", -20.0, 60.0),
    ("tr", "DC-DC temperature (DS18B20)", -20.0, 85.0),
    ("mq2", "Smoke/gas (MQ-2)", 0.01, 3.3),
)
# Same order as the firmware's ok bits and err counters
ENV_CHIPS = ("INA228", "BME280", "SGP30", "SPS30", "VEML7700", "DS18B20")
ENV_FIELDS_BY_CHIP = {
    "INA228": ("vbat", "i"), "BME280": ("t", "h", "p"), "SGP30": ("tvoc", "eco2"),
    "SPS30": ("pm25", "pm10"), "VEML7700": ("lux",), "DS18B20": ("tb", "tr"),
}
MQ2_WARMED_BIT = 6
PM_SUSPECT = 150.0  # ug/m3; clean indoor air is far below this

SPIN_MAX_ERPM = 3000
SPIN_MAX_SECONDS = 60
SPIN_SEND_PERIOD_S = 0.2  # the STM32 stops the motors after 1 s without a command

results = []

# Colours on the screen only; the log file stays plain text
USE_COLOUR = sys.stdout.isatty() and "NO_COLOR" not in os.environ
COLOURS = {OK: "32", WARN: "33", FAIL: "1;31", SETUP: "36",
           NOT_BUILT: "90", NOT_TESTED: "90", "TITLE": "1;36"}
ANSI_CODE = re.compile(r"\x1b\[[0-9;]*m")


def paint(text, kind):
    if not USE_COLOUR or kind not in COLOURS:
        return text
    return f"\x1b[{COLOURS[kind]}m{text}\x1b[0m"


def report(name, status, detail=""):
    results.append((name, status, detail))
    print(f"{name:<32} {paint(f'{status:<10}', status)} {detail}", flush=True)


def section(title):
    print(f"\n{paint(title, 'TITLE')}", flush=True)


# ---------------------------------------------------------------- Parsing

def parse_env(line):
    """'ENV a=1 b=2 ...' to a dict. err stays a list and ok becomes an int."""
    if not line.startswith("ENV "):
        return None
    env = {}
    for item in line.split()[1:]:
        key, _, value = item.partition("=")
        try:
            if key == "ok":
                env[key] = int(value, 16)
            elif key == "err":
                env[key] = [int(v) for v in value.split(",")]
            else:
                env[key] = float(value)
        except ValueError:
            return None
    return env


def parse_status(line):
    match = re.match(
        r"STATUS VESC:(-?\d+),(-?\d+),(-?\d+),(-?\d+) "
        r"TOF:(\d+) IMU:(\d) RC:(\d) BMP:(\d+)\s*$", line)
    if not match:
        return None
    g = [int(v) for v in match.groups()]
    return {"vesc": g[:4], "tof": g[4], "imu": g[5], "rc": g[6], "bmp": g[7]}


def parse_tof(line):
    if not line.startswith("TOF "):
        return None
    pairs = re.findall(r"(\w+):(\d+)", line)
    return {k: int(v) for k, v in pairs} if pairs else None


def parse_imu(line):
    if not line.startswith("IMU "):
        return None
    try:
        values = [float(v) for v in line.split()[1:]]
    except ValueError:
        return None
    return values if len(values) == 10 else None


def judge_imu(samples):
    """Status and detail from parsed IMU lines, taken with the robot still."""
    if not samples:
        return FAIL, "no IMU data"
    latest = samples[-1]
    norm = math.sqrt(sum(v * v for v in latest[:4]))
    if not 0.95 <= norm <= 1.05:
        return FAIL, f"bad orientation, quaternion length {norm:.3f}, should be 1"
    accel = math.sqrt(sum(v * v for v in latest[4:7]))
    if accel > 30:
        return FAIL, f"acceleration {accel:.1f} m/s2 with the robot still"
    if len(samples) >= 2 and all(s == samples[0] for s in samples):
        return FAIL, "values are frozen"
    return OK, f"quaternion length {norm:.3f}, acceleration {accel:.1f} m/s2"


def judge_tof(readings, online):
    """The firmware reports 0 for any reading it judged invalid, such as nothing in
    range, so a 0 alone does not mean the sensor failed. online is None if unknown."""
    if online is not None and not online:
        return FAIL, "offline: check XSHUT and I2C wires"
    if not readings:
        return FAIL, "no reading"
    valid = [r for r in readings if r > 0]
    if not valid:
        if online:
            return WARN, "online, but no valid distance: use --manual to test with a hand"
        return FAIL, "reads 0 mm"
    if max(valid) > 4000:
        return WARN, f"reads {max(valid)} mm, beyond its 4 m range"
    if len(valid) < len(readings):
        return WARN, f"{min(valid)} to {max(valid)} mm, {len(valid)} of {len(readings)} readings valid"
    return OK, f"{min(valid)} to {max(valid)} mm"


def judge_env_value(value, low, high):
    return low <= value <= high


def judge_thermal(raw):
    """Status and detail from serial bytes of the MLX90640 module.

    Expected frame, assuming a GY-MCU90640 type board (not yet confirmed on the
    real module): 5A 5A 02 06, then 768 pixels as little-endian int16 in 0.01 C.
    Returns None if the bytes contain no complete frame.
    """
    start = raw.find(THERMAL_HEADER)
    if start < 0 or len(raw) < start + 4 + THERMAL_PIXELS * 2:
        return None
    pixels = array.array("h")
    pixels.frombytes(raw[start + 4:start + 4 + THERMAL_PIXELS * 2])
    if sys.byteorder == "big":
        pixels.byteswap()
    temps = [p / 100.0 for p in pixels]
    low, high = min(temps), max(temps)
    if low < -40 or high > 300:
        return FAIL, f"impossible temperatures, {low:.1f} to {high:.1f} C"
    if low == high:
        return FAIL, f"every pixel reads {low:.1f} C"
    return OK, f"{low:.1f} to {high:.1f} C"


def audio_peak(raw):
    samples = array.array("h")
    samples.frombytes(raw[:len(raw) - len(raw) % 2])
    return max((abs(s) for s in samples), default=0)


# ---------------------------------------------------------------- Serial

BAUD = {9600: termios.B9600, 115200: termios.B115200, 460800: termios.B460800}


class Serial:
    def __init__(self, port, baud):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[3] = 0
        attrs[4] = BAUD[baud]
        attrs[5] = BAUD[baud]
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.pending = b""

    def write(self, text):
        os.write(self.fd, text if isinstance(text, bytes) else text.encode())

    def read_raw(self, seconds, want=None):
        end = time.monotonic() + seconds
        data = b""
        while time.monotonic() < end and (want is None or len(data) < want):
            ready, _, _ = select.select([self.fd], [], [], 0.05)
            if ready:
                try:
                    data += os.read(self.fd, 4096)
                except (BlockingIOError, OSError):
                    pass
        return data

    def lines(self, seconds):
        """Complete lines received during the next `seconds`."""
        self.pending += self.read_raw(seconds)
        *complete, self.pending = self.pending.split(b"\n")
        return [c.decode("ascii", "replace").strip() for c in complete]

    def close(self):
        os.close(self.fd)


def tty_usb_id(tty):
    """Return ('vendor', 'product') of the USB device behind a tty, if any."""
    path = os.path.realpath(f"/sys/class/tty/{os.path.basename(tty)}/device")
    while path not in ("/", ""):
        vendor = os.path.join(path, "idVendor")
        if os.path.exists(vendor):
            with open(vendor) as f:
                vid = f.read().strip()
            with open(os.path.join(path, "idProduct")) as f:
                return vid, f.read().strip()
        path = os.path.dirname(path)
    return None, None


def all_ttys():
    return sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))


def find_tty(vendor, product=None):
    for tty in all_ttys():
        vid, pid = tty_usb_id(tty)
        if vid == vendor and (product is None or pid == product):
            return tty
    return None


def port_busy(port):
    """Name of the process holding a port, or None."""
    if not shutil.which("fuser"):
        return None
    out = subprocess.run(["fuser", port], capture_output=True, text=True)
    pids = out.stdout.split()
    if not pids:
        return None
    try:
        with open(f"/proc/{pids[0]}/comm") as f:
            return f.read().strip()
    except OSError:
        return pids[0]


def usb_devices():
    found = {}
    for path in glob.glob("/sys/bus/usb/devices/*"):
        try:
            with open(os.path.join(path, "idVendor")) as f:
                vid = f.read().strip()
            with open(os.path.join(path, "idProduct")) as f:
                pid = f.read().strip()
            with open(os.path.join(path, "speed")) as f:
                speed = int(f.read().strip())
        except (OSError, ValueError):
            continue
        found[f"{vid}:{pid}"] = speed
    return found


def run(cmd, timeout=15):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return None


def ask(question):
    answer = input(f"  {question} [y/n] ").strip().lower()
    return answer.startswith("y")


# ---------------------------------------------------------------- STM32

class Stm32:
    def __init__(self, port):
        self.serial = Serial(port, 115200)

    def status(self):
        # Retried because the first command after opening the port can be lost
        for _ in range(3):
            self.serial.write("STATUS\n")
            for _ in range(4):
                for line in self.serial.lines(0.25):
                    parsed = parse_status(line)
                    if parsed:
                        return parsed
        return None

    def close(self):
        self.serial.close()


def check_stm32(port_arg):
    section("STM32")
    port = port_arg or find_tty(STM32_VENDOR)
    names = ["IMU (BNO085)", "CAN bus"]
    names += [f"VESC {i} ({where})" for i, where in VESCS]
    names += [f"ToF {where}" for _, _, where in TOF_SENSORS]
    if not port:
        report("STM32 (NUCLEO-F446RE)", FAIL, "not found on USB")
        for name in names:
            report(name, FAIL, "not checked, STM32 missing")
        return None
    busy = port_busy(port)
    if busy:
        report("STM32 (NUCLEO-F446RE)", FAIL, f"{port} in use by {busy}, stop it first")
        for name in names:
            report(name, FAIL, "not checked, port in use")
        return None

    stm = Stm32(port)
    print("  listening for 10 s...", flush=True)
    imu, tof, odo, errors = [], [], 0, set()
    status = stm.status()
    stm.serial.write("CAN\n")
    can_line = None
    start = time.monotonic()
    while time.monotonic() - start < 10:
        for line in stm.serial.lines(0.2):
            if line.startswith("ODO "):
                odo += 1
            elif line.startswith("ERR:"):
                errors.add(line)
            elif line.startswith("CAN LEC"):
                can_line = line
            elif parse_imu(line):
                imu.append(parse_imu(line))
            elif parse_tof(line):
                tof.append(parse_tof(line))

    rate = odo / 10.0
    if rate < 15:
        report("STM32 (NUCLEO-F446RE)", FAIL, f"ODO at {rate:.1f} Hz, should be 20")
    else:
        report("STM32 (NUCLEO-F446RE)", OK, f"ODO at {rate:.1f} Hz")
    if errors:
        report("STM32 error messages", WARN, " ".join(sorted(errors)))

    status_ok = status is not None
    if not status_ok:
        print("  STM32 did not answer STATUS: flash the current firmware for full results")

    # IMU
    state, detail = judge_imu(imu)
    if status_ok and not status["imu"]:
        state, detail = FAIL, "no data since boot"
    report("IMU (BNO085)", state, detail)

    # CAN
    if can_line is None:
        report("CAN bus", WARN, "no CAN status received")
    else:
        fields = dict(re.findall(r"(\w+)=(\w+)", can_line))
        if fields.get("BOFF") == "1":
            report("CAN bus", FAIL, f"bus off: {can_line}")
        elif int(fields.get("TEC", "0")) or int(fields.get("REC", "0")):
            report("CAN bus", WARN, f"TEC={fields.get('TEC')} REC={fields.get('REC')}")
        else:
            report("CAN bus", OK, "no errors")

    # VESCs
    for index, (vesc_id, where) in enumerate(VESCS):
        name = f"VESC {vesc_id} ({where})"
        if not status_ok:
            report(name, NOT_TESTED, "needs STATUS firmware")
        elif status["vesc"][index] < 0:
            report(name, FAIL, "never heard on CAN: check power and CAN wires")
        elif status["vesc"][index] > VESC_SILENT_MS:
            report(name, FAIL, f"silent for {status['vesc'][index]} ms")
        else:
            report(name, OK, f"heard {status['vesc'][index]} ms ago")

    # ToF
    for bit, label, where in TOF_SENSORS:
        online = status["tof"] & (1 << bit) if status_ok else None
        state, detail = judge_tof([s[label] for s in tof if label in s], online)
        report(f"ToF {where}", state, detail)
    return stm


def manual_stm32(stm):
    section("STM32: hands-on checks")
    status = stm.status()
    if status is None:
        print("  skipped, needs STATUS firmware")
        return

    # ToF: also proves each sensor sits where its name says
    for _, label, where in TOF_SENSORS:
        name = f"ToF {where} responds"
        if not ask(f"Hold your hand about 10 cm in front of the {where} ToF, keep it there. Ready?"):
            report(name, NOT_TESTED, "skipped")
            continue
        seen = []
        for line in stm.serial.lines(7):
            parsed = parse_tof(line)
            if parsed and label in parsed:
                seen.append(parsed[label])
        valid = [d for d in seen if d > 0]
        if valid and min(valid) < 300:
            report(name, OK, f"saw {min(valid)} mm")
        else:
            report(name, FAIL, f"readings {seen}: hand not seen, sensor may be swapped")

    # Bumpers: the bit must match the bumper being pressed
    for bit, where in enumerate(BUMPERS):
        name = f"Bumper {where}"
        print(f"  Press and hold the {where} bumper (10 s)...", flush=True)
        end = time.monotonic() + 10
        mask = 0
        while time.monotonic() < end and not mask:
            status = stm.status()
            mask = status["bmp"] if status else 0
        if mask == 1 << bit:
            report(name, OK, "pressed")
        elif mask:
            report(name, FAIL, f"another bumper reported (mask {mask}): wires swapped")
        else:
            report(name, FAIL, "no press detected")
        print("  Release it.", flush=True)
        time.sleep(2)
        stm.serial.lines(0.1)

    # RC: the receiver only has to see the transmitter, nothing is commanded here
    print("  Do not touch the sticks. Leave the override switch off.")
    print("  Turn the RC transmitter on (10 s)...", flush=True)
    end = time.monotonic() + 10
    seen = False
    while time.monotonic() < end and not seen:
        status = stm.status()
        seen = bool(status and status["rc"])
    report("RC receiver (FS-IA10B)", OK if seen else FAIL,
           "signal received" if seen else "no iBUS signal")
    print("  Turn the transmitter off.")

    # E-stop: VESCs lose power when the contactor opens
    if ask("Press the E-stop now. Pressed?"):
        time.sleep(2)
        status = stm.status()
        if status and all(age < 0 or age > VESC_SILENT_MS for age in status["vesc"]):
            report("E-stop cuts VESC power", OK, "all VESCs went silent")
        else:
            report("E-stop cuts VESC power", FAIL, "VESCs still heard with E-stop pressed")
        input("  Release the E-stop, then press Enter.")
        time.sleep(4)
        status = stm.status()
        back = status and all(0 <= age <= VESC_SILENT_MS for age in status["vesc"])
        report("Contactor closes again", OK if back else FAIL,
               "all VESCs back" if back else "not all VESCs came back")
    else:
        report("E-stop cuts VESC power", NOT_TESTED, "skipped")

    # Outputs with no feedback, so a person has to confirm them
    stm.serial.write("HORN 300\n")
    report("Horn", OK if ask("Did the horn sound?") else FAIL)
    stm.serial.write("LIGHT ALL 0 255 0 SOLID\n")
    time.sleep(1)
    report("STM32 to ESP32 link and side LEDs",
           OK if ask("Are both side LED strips green?") else FAIL)
    for target, question, name in (
        ("FRONT", "Are both front amber lights on?", "Front turn signals"),
        ("REAR", "Are both rear red lights on?", "Rear turn signals"),
    ):
        stm.serial.write("LIGHT ALL 0 0 0 SOLID\n")
        time.sleep(0.3)
        stm.serial.write(f"LIGHT {target} 255 255 255 SOLID\n")
        time.sleep(1)
        report(name, OK if ask(question) else FAIL)
    stm.serial.write("LIGHT ALL 0 0 0 SOLID\n")


# ---------------------------------------------------------------- ESP32

def check_esp32(port_arg, manual):
    section("ESP32")
    port = port_arg or find_tty(*ESP32_ID.split(":"))
    if not port:
        report("ESP32-S3", FAIL, "not found on USB")
        return
    busy = port_busy(port)
    if busy:
        report("ESP32-S3", FAIL, f"{port} in use by {busy}, stop it first")
        return
    print("  listening for 12 s...", flush=True)
    esp = Serial(port, 115200)
    env = None
    for line in esp.lines(12):
        env = parse_env(line) or env
    esp.close()
    if env is None:
        report("ESP32-S3", FAIL, "no ENV telemetry in 12 s")
        return
    report("ESP32-S3", OK, "telemetry received")
    judge_env(env)
    if manual:
        report("Status LCD (ST7789)", OK if ask("Is the LCD showing live data?") else FAIL)
        report("Buzzer", NOT_TESTED, "only sounds at startup: listen when powering on")
    else:
        report("Status LCD (ST7789)", NOT_TESTED, "use --manual")
        report("Buzzer", NOT_TESTED, "only sounds at startup: listen when powering on")


def judge_env(env):
    failed_fields = set()
    for bit, chip in enumerate(ENV_CHIPS):
        if not env.get("ok", 0) & (1 << bit):
            report(chip, FAIL, "not responding: check its wires")
            failed_fields.update(ENV_FIELDS_BY_CHIP[chip])
            continue
        errors = env.get("err", [0] * 6)[bit]
        if errors:
            report(chip, WARN, f"{errors} failed reads since boot: loose wire?")
        else:
            report(chip, OK, "responding")
    for field, name, low, high in ENV_LIMITS:
        if field in failed_fields:
            continue
        value = env.get(field)
        if value is None:
            report(name, FAIL, "missing from telemetry")
        elif field == "mq2" and not env.get("ok", 0) & (1 << MQ2_WARMED_BIT):
            report(name, WARN, f"{value:g} V, still warming up (3 min after power on)")
        elif field in ("pm25", "pm10") and value > PM_SUSPECT:
            report(name, WARN, f"{value:g} ug/m3 is very high: smoke, dust or a faulty sensor")
        elif judge_env_value(value, low, high):
            report(name, OK, f"{value:g}")
        else:
            report(name, FAIL, f"{value:g}, expected {low:g} to {high:g}")


# ---------------------------------------------------------------- Jetson

def check_jetson_usb(devices, manual):
    section("Jetson: audio, 4G, networks")
    if RESPEAKER_ID not in devices and not has_alsa_card("ArrayUAC10"):
        report("ReSpeaker microphone", FAIL, "not found on USB")
    else:
        check_microphone(manual)
    if manual:
        report("Amplifier and left speaker", check_speaker(0))
        report("Amplifier and right speaker", check_speaker(1))
    else:
        report("Amplifier and speakers", NOT_TESTED, "use --manual")
    check_modem(devices)
    check_networks()


def has_alsa_card(name):
    try:
        with open("/proc/asound/cards") as f:
            return name in f.read()
    except OSError:
        return False


def check_microphone(manual):
    if manual:
        input("  Press Enter, then talk for 2 seconds.")
    with tempfile.NamedTemporaryFile(suffix=".raw") as tmp:
        result = run(["arecord", "-q", "-D", ALSA_DEVICE, "-f", "S16_LE", "-r", "16000",
                      "-c", "1", "-t", "raw", "-d", "2", tmp.name])
        if result is None or result.returncode != 0:
            report("ReSpeaker microphone", FAIL, "cannot record")
            return
        peak = audio_peak(tmp.read())
    if peak == 0:
        # Its processed output can be exactly 0 in a quiet room, so only a
        # silent recording made while someone talks counts as a fault
        if manual:
            report("ReSpeaker microphone", FAIL, "records silence while you talk")
        else:
            report("ReSpeaker microphone", WARN, "silent, may just be a quiet room: use --manual")
    else:
        report("ReSpeaker microphone", OK, f"recording, peak level {peak}")


def check_speaker(channel):
    side = "left" if channel == 0 else "right"
    print(f"  Playing on the {side} channel, volume knob must be up...", flush=True)
    run(["speaker-test", "-q", "-D", ALSA_DEVICE, "-c", "2", "-t", "sine", "-f", "440",
         "-s", str(channel + 1), "-l", "1"], timeout=10)
    return OK if ask(f"Did only the {side} speaker play a tone?") else FAIL


def check_modem(devices):
    name = "4G modem (EC25-EUX)"
    if MODEM_ID not in devices:
        report(name, FAIL, "not found on USB")
        return
    if not shutil.which("mmcli"):
        report(name, WARN, "on USB, mmcli not installed")
        return
    listed = run(["mmcli", "-L"])
    match = re.search(r"/Modem/(\d+)", listed.stdout) if listed else None
    if not match:
        report(name, WARN, "on USB, ModemManager does not see it (is it running?)")
        return
    info = run(["mmcli", "-m", match.group(1), "--output-keyvalue"])
    text = info.stdout if info else ""
    state = re.search(r"modem\.generic\.state\s*:\s*(\S+)", text)
    signal = re.search(r"modem\.generic\.signal-quality\.value\s*:\s*(\d+)", text)
    state = state.group(1) if state else "unknown"
    signal = signal.group(1) if signal else "?"
    good = state in ("registered", "connected", "connecting")
    report(name, OK if good else WARN, f"state {state}, signal {signal}%")


def check_networks():
    devices = run(["nmcli", "-t", "-f", "TYPE,STATE", "device"])
    if devices is None or devices.returncode != 0:
        report("Wi-Fi", NOT_TESTED, "nmcli not available")
        report("Ethernet", NOT_TESTED, "nmcli not available")
        return
    states = {}
    for line in devices.stdout.splitlines():
        kind, _, state = line.partition(":")
        if kind in ("wifi", "ethernet") and states.get(kind) != "connected":
            states[kind] = state
    if states.get("wifi") == "connected":
        signal = None
        wifi = run(["nmcli", "-t", "-f", "ACTIVE,SIGNAL", "device", "wifi"])
        for line in (wifi.stdout.splitlines() if wifi else []):
            if line.startswith("yes:"):
                signal = int(line.split(":")[1])
        if signal is not None and signal < 50:
            report("Wi-Fi", WARN, f"connected, weak signal {signal}%")
        else:
            report("Wi-Fi", OK, f"connected, signal {signal}%")
    else:
        report("Wi-Fi", FAIL, f"state {states.get('wifi', 'no adapter')}")
    state = states.get("ethernet", "no adapter")
    if state == "connected":
        report("Ethernet", OK, "connected")
    else:
        report("Ethernet", NOT_TESTED, f"state {state}")


def video_nodes(name_part):
    nodes = []
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            with open(os.path.join(path, "name")) as f:
                name = f.read().strip()
        except OSError:
            continue
        if name_part.lower() in name.lower():
            nodes.append("/dev/" + os.path.basename(path))
    return nodes


def grab_frame(node):
    """Bytes of one captured frame, or None if this node gives no image."""
    with tempfile.NamedTemporaryFile() as tmp:
        result = run(["v4l2-ctl", "-d", node, "--stream-mmap", "--stream-skip=5",
                      "--stream-count=1", f"--stream-to={tmp.name}"], timeout=15)
        if result is None or result.returncode != 0:
            return None
        return tmp.read() or None


def check_camera(name, name_part):
    if name_part is None:
        report(name, SETUP, "set the camera name at the top of this script")
        return
    nodes = video_nodes(name_part)
    if not nodes:
        report(name, FAIL, "not found")
        return
    for node in nodes:
        frame = grab_frame(node)
        if not frame:
            continue
        if len(set(frame[:200000])) <= 4:
            report(name, FAIL, f"{node} gives a blank picture")
        else:
            report(name, OK, f"{node} gives a picture")
        return
    report(name, FAIL, "found, but no picture")


def check_cameras(devices):
    section("Jetson: cameras")
    if not shutil.which("v4l2-ctl"):
        print("  v4l2-ctl missing, install with: sudo apt install v4l-utils")
    name = "RealSense D435i"
    if REALSENSE_ID not in devices:
        report(name, FAIL, "not found on USB")
    elif devices[REALSENSE_ID] < 5000:
        report(name, WARN, f"running at {devices[REALSENSE_ID]} Mb/s: needs a USB 3 port and cable")
    elif shutil.which("v4l2-ctl"):
        check_camera(name, "RealSense")
    else:
        report(name, OK, "on USB 3, picture not checked")
    if shutil.which("v4l2-ctl"):
        check_camera("360 camera", CAMERA_360_NAME)
    else:
        report("360 camera", NOT_TESTED, "needs v4l2-ctl")


def check_lidar_on(port):
    """True if an RPLIDAR answers on port. Returns (found, health status)."""
    try:
        lidar = Serial(port, 460800)
    except OSError:
        return False, None
    try:
        lidar.write(bytes((0xA5, 0x25)))  # stop any scan
        lidar.read_raw(0.1)
        lidar.write(bytes((0xA5, 0x52)))  # get health
        reply = lidar.read_raw(1.0, want=10)
    finally:
        lidar.close()
    if len(reply) >= 10 and reply[:2] == bytes((0xA5, 0x5A)):
        return True, reply[7]
    return False, None


def find_thermal(candidates):
    """Listen only, sending nothing. Returns (port, status, detail) or None."""
    for port in candidates:
        if port_busy(port):
            continue
        for baud in THERMAL_BAUDS:
            try:
                link = Serial(port, baud)
            except OSError:
                continue
            try:
                judged = judge_thermal(link.read_raw(4 if baud == 9600 else 1.5))
            finally:
                link.close()
            if judged:
                return (port,) + judged
    return None


def check_serial_devices(stm_port, esp_port):
    section("Jetson: thermal camera, LiDAR")
    # USB serial adapters look alike, so identify them by what they answer
    candidates = [t for t in all_ttys()
                  if tty_usb_id(t)[0] in USB_SERIAL_VENDORS
                  and t not in (stm_port, esp_port)]

    # First, because it only listens: the LiDAR check below sends commands
    thermal = find_thermal(candidates)
    if thermal:
        thermal_port, status, detail = thermal
        report("Thermal camera (MLX90640)", status, f"{thermal_port}, {detail}")
        candidates.remove(thermal_port)
    else:
        report("Thermal camera (MLX90640)", FAIL, "no thermal frames on any USB serial port")

    lidar_port = None
    for port in candidates:
        if port_busy(port):
            continue
        found, health = check_lidar_on(port)
        if found:
            lidar_port = port
            if health == 0:
                report("LiDAR (RPLIDAR C1)", OK, f"{port}, health good")
            else:
                report("LiDAR (RPLIDAR C1)", FAIL, f"{port}, reports health {health}")
            break
    if lidar_port is None:
        report("LiDAR (RPLIDAR C1)", FAIL, "no reply on any USB serial port")


# ---------------------------------------------------------------- Summary

def not_available_yet(manual):
    section("In the spec, but not on this robot's firmware yet")
    for name in ("Pan/tilt servos", "Laser", "E-stop auxiliary contact",
                 "Contactor auxiliary contact", "Panel buttons",
                 "IR illuminators", "Rear light", "Hazard-flasher fallback"):
        report(name, NOT_BUILT, "no firmware or wiring map to test against")
    report("4 motors and hall sensors", NOT_TESTED, "run --spin with the wheels off the ground")
    if not manual:
        report("Bumpers, RC, E-stop, horn, lights", NOT_TESTED, "use --manual")
    print("\nCheck by hand, software cannot see these:")
    print("  - Power off: resistance between CAN-H and CAN-L is about 60 ohm")
    print("  - Every branch fuse fitted and the right size")
    print("  - Flyback diodes, TVS diodes and CAN ground wire in place")
    print("  - Spotlight: no command reaches it through the LIGHT protocol yet")


def compare_with_baseline(baseline):
    rank = {OK: 3, WARN: 2, NOT_TESTED: 1, SETUP: 1, NOT_BUILT: 1, FAIL: 0}
    worse = [(name, baseline[name], status) for name, status, _ in results
             if name in baseline and rank.get(status, 1) < rank.get(baseline[name], 1)]
    section("Worse than the saved reference")
    if not worse:
        print("  nothing")
    for name, before, now in worse:
        print(f"  {name}: was {paint(before, before)}, now {paint(now, now)}")


def stop_wheels(link):
    # Sent twice in case one is lost; the STM32's 1 s timeout is the backstop
    for _ in range(2):
        link.write("CMD 0 0\n")
        time.sleep(0.05)


def spin_wheels(left, right, duration, port_arg):
    if max(abs(left), abs(right)) > SPIN_MAX_ERPM:
        print(paint(f"STOPPED: speed must be within +/-{SPIN_MAX_ERPM} ERPM", FAIL))
        return 1
    if not 0 < duration <= SPIN_MAX_SECONDS:
        print(paint(f"STOPPED: time must be above 0 and at most {SPIN_MAX_SECONDS} s", FAIL))
        return 1
    port = port_arg or find_tty(STM32_VENDOR)
    if not port:
        print(paint("STOPPED: STM32 not found on USB", FAIL))
        return 1
    busy = port_busy(port)
    if busy:
        print(paint(f"STOPPED: {port} in use by {busy}, stop it first", FAIL))
        return 1

    print(f"Left {left}, right {right} ERPM for {duration:g} s. Ctrl+C stops.")
    if not ask("Wheels off the ground, RC off, E-stop in reach?"):
        print("Not started.")
        return 1

    link = Serial(port, 115200)
    command = f"CMD {left} {right}\n"
    acknowledged = False
    interrupted = False
    feedback = None
    end = time.monotonic() + duration
    try:
        while time.monotonic() < end:
            link.write(command)
            for line in link.lines(SPIN_SEND_PERIOD_S):
                if line.startswith("CMD_OK"):
                    acknowledged = True
                elif line.startswith("ODO "):
                    feedback = line
                elif line.startswith("ERR:") or line.startswith("RC_ON"):
                    print(line)
    except KeyboardInterrupt:
        interrupted = True
        print("\nStopping.")
    finally:
        stop_wheels(link)
        link.close()

    print(paint("Stop sent.", OK))
    if feedback:
        print(f"Last speed feedback (left, right): {feedback}")
    if not acknowledged and not interrupted:
        print(paint("WARNING: the STM32 never confirmed the command", FAIL))
        return 1
    return 0


class Tee:
    """Writes everything printed to both the screen and a log file."""

    def __init__(self, screen, log):
        self.screen = screen
        self.log = log

    def write(self, text):
        self.screen.write(text)
        self.log.write(ANSI_CODE.sub("", text))

    def flush(self):
        self.screen.flush()
        self.log.flush()


def open_log():
    """Starts logging to robot_check.log beside this script, replacing the last run."""
    folder = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(folder, "robot_check.log")
    sys.stdout = Tee(sys.stdout, open(path, "w"))
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manual", action="store_true", help="include hands-on checks")
    parser.add_argument("--baseline", action="store_true", help="save this run as the reference")
    parser.add_argument("--stm-port", help="default: found by USB ID")
    parser.add_argument("--esp-port", help="default: found by USB ID")
    parser.add_argument("--spin", nargs="*", type=int, metavar="ERPM",
                        help="spin the wheels instead of checking: [left] [right], default 1000")
    parser.add_argument("--time", type=float, default=5.0, help="seconds for --spin, default 5")
    parser.add_argument("--only", nargs="+", choices=("stm32", "esp32", "jetson"),
                        help="check only these boards and what is wired to them")
    args = parser.parse_args()

    if args.spin is not None:
        if len(args.spin) > 2:
            parser.error("--spin takes at most left and right")
        left = args.spin[0] if args.spin else 1000
        right = args.spin[1] if len(args.spin) == 2 else left
        return spin_wheels(left, right, args.time, args.stm_port)

    log_path = open_log()
    print(time.strftime("Robot check %Y-%m-%d %H:%M:%S"))
    print("Motors will not move: this script never sends CMD.")
    boards = set(args.only or ("stm32", "esp32", "jetson"))
    if args.only:
        print("Only checking: " + ", ".join(args.only))
    stm = check_stm32(args.stm_port) if "stm32" in boards else None
    if "esp32" in boards:
        check_esp32(args.esp_port, args.manual)
    if "jetson" in boards:
        devices = usb_devices()
        check_jetson_usb(devices, args.manual)
        check_cameras(devices)
        stm_port = args.stm_port or find_tty(STM32_VENDOR)
        esp_port = args.esp_port or find_tty(*ESP32_ID.split(":"))
        check_serial_devices(stm_port, esp_port)
    if stm:
        if args.manual:
            manual_stm32(stm)
        stm.close()
    if not args.only:
        not_available_yet(args.manual)

    saved = {}
    if os.path.exists(BASELINE_FILE):
        with open(BASELINE_FILE) as f:
            saved = json.load(f)
        compare_with_baseline(saved)
    if args.baseline:
        # With --only, keep the saved results of the boards not checked this time
        if not args.only:
            saved = {}
        saved.update({name: status for name, status, _ in results})
        with open(BASELINE_FILE, "w") as f:
            json.dump(saved, f, indent=1)
        print(f"\nSaved as the reference: {BASELINE_FILE}")

    count = {}
    for _, status, _ in results:
        count[status] = count.get(status, 0) + 1
    print("\nTOTAL  " + "  ".join(paint(f"{k}: {v}", k) for k, v in sorted(count.items())))
    failed = [name for name, status, _ in results if status == FAIL]
    if failed:
        print(paint("FAILED: " + ", ".join(failed), FAIL))
    else:
        print(paint("No failures", OK))
    print(f"Log: {log_path}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())

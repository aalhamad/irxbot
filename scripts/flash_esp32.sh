#!/usr/bin/env bash
# Build the ESP32-S3 firmware and flash it. Run on the Jetson, with the ESP32 on USB:
#
#     ./scripts/flash_esp32.sh             # build and flash
#     ./scripts/flash_esp32.sh --monitor   # then watch the output; Ctrl+] quits
#
# The monitor shows the debug logs if the ESP32's UART port is plugged in too,
# otherwise the sensor data from its USB port.
set -euo pipefail

MONITOR=0
case "${1:-}" in
    "") ;;
    --monitor) MONITOR=1 ;;
    *) echo "Usage: $0 [--monitor]" >&2; exit 1 ;;
esac

REPO=$(cd "$(dirname "$0")/.." && pwd)
# ESP-IDF 5.3 is at ~/Apps/esp-idf on the Jetson; IDF_PATH overrides it
IDF_EXPORT=""
for dir in "${IDF_PATH:-}" "$HOME/Apps/esp-idf" "$HOME/apps/esp-idf"; do
    if [ -n "$dir" ] && [ -f "$dir/export.sh" ]; then
        IDF_EXPORT=$dir/export.sh
        break
    fi
done

fail() {
    echo "STOPPED: $1. Nothing was flashed." >&2
    exit 1
}

[ -n "$IDF_EXPORT" ] || fail "ESP-IDF was not found in ~/Apps/esp-idf or ~/apps/esp-idf"

PORT=$(ls /dev/serial/by-id/usb-Espressif* 2>/dev/null | head -n 1 || true)
[ -n "$PORT" ] || fail "the ESP32 was not found on USB"
if fuser "$PORT" >/dev/null 2>&1; then
    fail "the ESP32 port is in use: run 'sudo systemctl stop irx-esp32-telemetry' or stop the ROS2 node"
fi

# shellcheck disable=SC1090
. "$IDF_EXPORT" >/dev/null
cd "$REPO/firmware/esp32/irxbot_esp32"
idf.py build || fail "build failed"
idf.py -p "$PORT" flash
echo "Done."

if [ "$MONITOR" = 1 ]; then
    LOG_PORT=$(ls /dev/serial/by-id/usb-1a86* 2>/dev/null | head -n 1 || true)
    if [ -n "$LOG_PORT" ]; then
        echo "Monitoring the debug logs (UART port). Ctrl+] quits."
        idf.py -p "$LOG_PORT" monitor
    else
        echo "Monitoring the sensor data (USB port); plug in the UART port for logs. Ctrl+] quits."
        sleep 2  # the USB port drops while the ESP32 restarts after flashing
        idf.py -p "$PORT" monitor
    fi
fi

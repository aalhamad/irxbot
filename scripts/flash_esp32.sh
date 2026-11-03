#!/usr/bin/env bash
# Build the ESP32-S3 firmware and flash it. Run on the Jetson, with the ESP32 on USB:
#
#     ./scripts/flash_esp32.sh
set -euo pipefail

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

#!/usr/bin/env bash
# Build the STM32 firmware and flash it. Run on the Jetson, with the STM32 on USB:
#
#     ./scripts/flash_stm32.sh
#
# Flashing resets the STM32, so the contactor drops and closes again.
# Before overwriting anything, the current build is kept as previous.bin.
set -euo pipefail

REPO=$(cd "$(dirname "$0")/.." && pwd)
PORT=/dev/ttyACM0
BUILD=$HOME/irx_fw_build
OBJ=$BUILD/obj
OUT=$BUILD/current
MIN_SIZE=50000

fail() {
    echo "STOPPED: $1. Nothing was flashed." >&2
    exit 1
}

command -v arm-none-eabi-gcc >/dev/null || fail "arm-none-eabi-gcc is not installed"
command -v st-flash >/dev/null || fail "st-flash is not installed"
if fuser "$PORT" >/dev/null 2>&1; then
    fail "$PORT is in use: stop the ROS2 nodes or minicom first"
fi

cd "$REPO/firmware/stm32/irx_drive"
CPU="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
CFLAGS="$CPU -DUSE_HAL_DRIVER -DSTM32F446xx -DDEBUG -Og -g3 -ffunction-sections -fdata-sections -Wall --specs=nano.specs"
CFLAGS="$CFLAGS -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy"
CFLAGS="$CFLAGS -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include -I../lib/sh2"

echo "Building..."
mkdir -p "$BUILD"
rm -rf "$OBJ"
mkdir -p "$OBJ"
objects=()
for src in Core/Src/*.c Drivers/STM32F4xx_HAL_Driver/Src/*.c; do
    case "$src" in *_template.c) continue ;; esac
    obj=$OBJ/$(basename "$src" .c).o
    # shellcheck disable=SC2086
    arm-none-eabi-gcc $CFLAGS -c "$src" -o "$obj" 2>"$obj.log" || {
        cat "$obj.log" >&2
        fail "build failed in $src"
    }
    objects+=("$obj")
done
# shellcheck disable=SC2086
arm-none-eabi-gcc $CFLAGS -x assembler-with-cpp -c Core/Startup/startup_stm32f446retx.s \
    -o "$OBJ/startup.o" || fail "build failed in the startup file"

[ -s "$OUT.bin" ] && cp "$OUT.bin" "$BUILD/previous.bin"

# shellcheck disable=SC2086
arm-none-eabi-gcc $CPU --specs=nosys.specs --specs=nano.specs -u _printf_float \
    -T STM32F446RETX_FLASH.ld -Wl,--gc-sections \
    -Wl,--start-group "${objects[@]}" "$OBJ/startup.o" -lc -lm -Wl,--end-group \
    -o "$OUT.elf" || fail "link failed"
arm-none-eabi-objcopy -O binary "$OUT.elf" "$OUT.bin" || fail "objcopy failed"
sync

size=$(stat -c%s "$OUT.bin")
[ "$size" -ge "$MIN_SIZE" ] || fail "firmware is only $size bytes"

echo "Flashing $size bytes..."
st-flash --connect-under-reset write "$OUT.bin" 0x08000000
st-flash --connect-under-reset reset
echo "Done. Previous build: $BUILD/previous.bin"

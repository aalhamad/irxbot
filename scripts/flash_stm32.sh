#!/bin/bash

cd ~/irx_drive
arm-none-eabi-objcopy -O binary Debug/irx_drive.elf Debug/irx_drive.bin
st-flash --reset --connect-under-reset write Debug/irx_drive.bin 0x08000000
echo "Flash complete!"

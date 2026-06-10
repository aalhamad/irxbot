#!/bin/bash
# Build and deploy to Jetson
scp /home/aalhamad/Code/irxbot/firmware/stm32/irx_drive/Debug/irx_drive.elf \
  aalhamad@192.168.8.191:/home/aalhamad/irxbot/firmware/stm32/irx_drive/Debug/

ssh aalhamad@192.168.8.191 "/home/aalhamad/irxbot/scripts/flash_stm32.sh"
echo "Deployed!"

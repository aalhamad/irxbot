# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/aalhamad/apps/esp-idf/components/bootloader/subproject"
  "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader"
  "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader-prefix"
  "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader-prefix/tmp"
  "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader-prefix/src/bootloader-stamp"
  "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader-prefix/src"
  "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/aalhamad/Code/irxbot/firmware/esp32/irxbot_esp32/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

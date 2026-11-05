// Finds a USB serial device by its fixed name in /dev/serial/by-id, so the nodes
// still find the STM32 and ESP32 when ttyACM0 and ttyACM1 swap after a reboot.

#pragma once

#include <algorithm>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace irx_driver
{

inline constexpr const char *kStm32Prefix = "usb-STMicroelectronics";
inline constexpr const char *kEsp32Prefix = "usb-Espressif";

// Returns the first /dev/serial/by-id entry whose name starts with prefix,
// or an empty string if there is none.
inline std::string find_serial_by_id(const std::string &prefix)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<std::string> matches;
    for (fs::directory_iterator it("/dev/serial/by-id", ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        if (name.rfind(prefix, 0) == 0) matches.push_back(it->path().string());
    }
    if (matches.empty()) return "";
    std::sort(matches.begin(), matches.end());
    return matches.front();
}

}  // namespace irx_driver

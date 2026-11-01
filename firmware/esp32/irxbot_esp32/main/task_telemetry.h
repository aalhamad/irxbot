#pragma once

#include <stddef.h>

void task_telemetry_start(void);

/* Text for the LCD: the Jetson's IP address, "NO NETWORK", or "NO JETSON" */
void telemetry_get_ip_text(char *out, size_t len);

#include "task_telemetry.h"
#include "task_sensors.h"

#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define TAG "telemetry"

#define TELEMETRY_PERIOD_MS 5000
#define USB_WRITE_TIMEOUT_MS 100   /* don't stall when no host is reading */
#define IP_STALE_MS         30000  /* the Jetson repeats its IP every 5 s */
#define IP_TEXT_LEN         16     /* "255.255.255.255" */
#define RX_LINE_LEN         32

static portMUX_TYPE _ip_mux = portMUX_INITIALIZER_UNLOCKED;
static char         _ip[IP_TEXT_LEN];  /* empty = Jetson reports no network */
static bool         _ip_seen;
static TickType_t   _ip_tick;

/* Sends one "ENV ..." line to the Jetson. The USB port carries nothing else:
   logs go to UART0 only. */
static void telemetry_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    char line[320];

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));

        int len = snprintf(line, sizeof(line),
               "ENV vbat=%.2f i=%.3f v12=%.2f v5=%.2f v33=%.2f "
               "t=%.1f h=%.0f p=%.1f tvoc=%u eco2=%u pm25=%.1f pm10=%.1f lux=%.0f "
               "tb=%.1f tr=%.1f tf=%.1f mq2=%.3f st=%u ok=0x%02X "
               "err=%u,%u,%u,%u,%u,%u\n",
               g_sensors.bus_v, g_sensors.current_a,
               g_sensors.v_12v, g_sensors.v_5v, g_sensors.v_3v3,
               g_sensors.temp_c, g_sensors.humidity_pct, g_sensors.pressure_hpa,
               g_sensors.tvoc_ppb, g_sensors.eco2_ppm,
               g_sensors.pm2_5, g_sensors.pm10, g_sensors.lux,
               g_sensors.temp_batt_c, g_sensors.temp_elec_rear_c, g_sensors.temp_elec_front_c,
               g_sensors.mq2_v, g_sensors.mq2_status, g_sensors.ok,
               g_sensors.err[0], g_sensors.err[1], g_sensors.err[2],
               g_sensors.err[3], g_sensors.err[4], g_sensors.err[5]);
        if (len > 0 && len < (int)sizeof(line)) {
            usb_serial_jtag_write_bytes(line, len, pdMS_TO_TICKS(USB_WRITE_TIMEOUT_MS));
        }
    }
}

static bool valid_ipv4(const char *text)
{
    unsigned a, b, c, d;
    char extra;
    return sscanf(text, "%3u.%3u.%3u.%3u%c", &a, &b, &c, &d, &extra) == 4 &&
           a <= 255 && b <= 255 && c <= 255 && d <= 255;
}

/* "IP 192.168.8.191" or "IP NONE" */
static void handle_line(const char *line)
{
    if (strncmp(line, "IP ", 3) != 0) return;

    const char *value = line + 3;
    bool none = strcmp(value, "NONE") == 0;
    if (!none && !valid_ipv4(value)) return;

    taskENTER_CRITICAL(&_ip_mux);
    strlcpy(_ip, none ? "" : value, sizeof(_ip));
    _ip_seen = true;
    _ip_tick = xTaskGetTickCount();
    taskEXIT_CRITICAL(&_ip_mux);
}

static void usb_rx_task(void *arg)
{
    uint8_t buf[64];
    char    line[RX_LINE_LEN];
    int     line_len = 0;
    bool    overflow = false;

    while (1) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\n' || c == '\r') {
                if (line_len > 0 && !overflow) {
                    line[line_len] = '\0';
                    handle_line(line);
                }
                line_len = 0;
                overflow = false;
            } else if (line_len < (int)sizeof(line) - 1) {
                line[line_len++] = c;
            } else {
                overflow = true;
            }
        }
    }
}

void telemetry_get_ip_text(char *out, size_t len)
{
    taskENTER_CRITICAL(&_ip_mux);
    bool seen = _ip_seen && (xTaskGetTickCount() - _ip_tick) < pdMS_TO_TICKS(IP_STALE_MS);
    char ip[IP_TEXT_LEN];
    strlcpy(ip, _ip, sizeof(ip));
    taskEXIT_CRITICAL(&_ip_mux);

    if (!seen) {
        strlcpy(out, "NO JETSON", len);
    } else if (ip[0] == '\0') {
        strlcpy(out, "NO NETWORK", len);
    } else {
        strlcpy(out, ip, len);
    }
}

void task_telemetry_start(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 256,
    };
    esp_err_t err = usb_serial_jtag_driver_install(&cfg);
    if (err != ESP_OK) {
        /* A USB fault must not take the sensors, lights or alarm down with it */
        ESP_LOGE(TAG, "USB driver failed: %s", esp_err_to_name(err));
        return;
    }
    xTaskCreate(telemetry_task, "telemetry", 4096, NULL, 2, NULL);
    xTaskCreate(usb_rx_task, "usb_rx", 2560, NULL, 2, NULL);
}

#include "task_display.h"
#include "task_sensors.h"
#include "task_telemetry.h"

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_lvgl_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define TAG "display"

#define PIN_CS    10
#define PIN_MOSI  11
#define PIN_SCK   12
#define PIN_DC    41
#define PIN_RST   5

#define LCD_W      320
#define LCD_H      240
#define BUF_LINES  30              /* partial draw buffer, in display lines */
#define BUF_PIXELS (LCD_W * BUF_LINES)

#define STARTUP_MS  20000UL        /* every sensor has been read by now */
#define OK_SENSORS  0x3F           /* ok bits 0-5 */

#define LOAD_BAR_MAX_A 20.0f       /* battery continuous discharge limit */

#define C_BG      lv_color_hex(0x10151C)
#define C_PANEL   lv_color_hex(0x1C2530)
#define C_LINE    lv_color_hex(0x33414F)
#define C_TEXT    lv_color_hex(0xF2F5F7)
#define C_MUTED   lv_color_hex(0x93A1AE)
#define C_GREEN   lv_color_hex(0x3DDC84)
#define C_YELLOW  lv_color_hex(0xFFC83D)
#define C_RED     lv_color_hex(0xE5383B)
#define C_BLACK   lv_color_hex(0x000000)
#define C_WHITE   lv_color_hex(0xFFFFFF)

static lv_obj_t *_status, *_uptime;
static lv_obj_t *_volts, *_amps, *_load;
static lv_obj_t *_v12, *_v5, *_v33;
static lv_obj_t *_t_bat, *_t_rear, *_t_front, *_t_air, *_air_name;
static lv_obj_t *_pm25, *_tvoc, *_eco2, *_lux;
static lv_obj_t *_jetson_ip;

#define AIR_CYCLE_MS 3000  /* time each BME280 value stays on the LCD */

/* ── Widgets ────────────────────────────────────────────────────────────────── */

static lv_obj_t *make_panel(lv_obj_t *parent, int x, int y, int w, int h, lv_color_t color)
{
    lv_obj_t *p = lv_obj_create(parent);
    lv_obj_remove_flag(p, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_pos(p, x, y);
    lv_obj_set_size(p, w, h);
    lv_obj_set_style_bg_color(p, color, 0);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(p, 0, 0);
    lv_obj_set_style_radius(p, 6, 0);
    lv_obj_set_style_pad_all(p, 0, 0);
    return p;
}

static lv_obj_t *make_label(lv_obj_t *parent, lv_align_t align, int x, int y,
                            const lv_font_t *font, lv_color_t color, const char *text)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_label_set_text(l, text);
    lv_obj_align(l, align, x, y);
    return l;
}

/* One "NAME  value" row in a grid panel. Returns the value label. */
static lv_obj_t *make_row(lv_obj_t *panel, int y, const char *name)
{
    make_label(panel, LV_ALIGN_TOP_LEFT, 8, y + 2, &lv_font_montserrat_14, C_MUTED, name);
    return make_label(panel, LV_ALIGN_TOP_RIGHT, -8, y, &lv_font_montserrat_20, C_TEXT, "--");
}

/* Sets the text only when it changed, so LVGL redraws only what changed */
static void set_text(lv_obj_t *label, const char *fmt, ...)
{
    char text[48];
    va_list args;
    va_start(args, fmt);
    vsnprintf(text, sizeof(text), fmt, args);
    va_end(args);

    if (strcmp(lv_label_get_text(label), text) != 0) lv_label_set_text(label, text);
}

static void build_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    /* Header */
    make_label(scr, LV_ALIGN_TOP_LEFT, 8, 4, &lv_font_montserrat_14, C_TEXT, "IRXBOT");
    _status = make_label(scr, LV_ALIGN_TOP_MID, 0, 4, &lv_font_montserrat_14, C_MUTED, "STARTING");
    _uptime = make_label(scr, LV_ALIGN_TOP_RIGHT, -8, 4, &lv_font_montserrat_14, C_MUTED, "UP 0m");

    /* Power */
    _volts = make_label(scr, LV_ALIGN_TOP_LEFT, 8, 24, &lv_font_montserrat_28, C_YELLOW, "--");
    _amps  = make_label(scr, LV_ALIGN_TOP_LEFT, 122, 30, &lv_font_montserrat_20, C_YELLOW, "--");

    _load = lv_bar_create(scr);
    lv_obj_set_pos(_load, 214, 34);
    lv_obj_set_size(_load, 96, 14);
    lv_bar_set_range(_load, 0, 100);
    lv_obj_set_style_bg_color(_load, C_LINE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(_load, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(_load, C_YELLOW, LV_PART_INDICATOR);

    _v12 = make_label(scr, LV_ALIGN_TOP_LEFT, 8, 58, &lv_font_montserrat_14, C_MUTED, "12V --");
    _v5  = make_label(scr, LV_ALIGN_TOP_MID, 0, 58, &lv_font_montserrat_14, C_MUTED, "5V --");
    _v33 = make_label(scr, LV_ALIGN_TOP_RIGHT, -8, 58, &lv_font_montserrat_14, C_MUTED, "3.3V --");

    /* Temperatures and air quality */
    lv_obj_t *left  = make_panel(scr, 4, 80, 154, 112, C_PANEL);
    lv_obj_t *right = make_panel(scr, 162, 80, 154, 112, C_PANEL);

    _t_bat   = make_row(left, 2, "BAT");
    _t_rear  = make_row(left, 29, "REAR");
    _t_front = make_row(left, 56, "FRONT");
    /* BME280 row: cycles air temperature, humidity and pressure */
    _air_name = make_label(left, LV_ALIGN_TOP_LEFT, 8, 85, &lv_font_montserrat_14, C_MUTED, "AIR");
    _t_air    = make_label(left, LV_ALIGN_TOP_RIGHT, -8, 83, &lv_font_montserrat_20, C_TEXT, "--");

    _pm25 = make_row(right, 2, "PM2.5");
    _tvoc = make_row(right, 29, "TVOC");
    _eco2 = make_row(right, 56, "eCO2");
    _lux  = make_row(right, 83, "LUX");

    /* The Jetson's IP address */
    lv_obj_t *ip_panel = make_panel(scr, 4, 196, 312, 40, C_PANEL);
    make_label(ip_panel, LV_ALIGN_LEFT_MID, 10, 0, &lv_font_montserrat_14, C_MUTED, "JETSON");
    _jetson_ip = make_label(ip_panel, LV_ALIGN_RIGHT_MID, -10, 0, &lv_font_montserrat_20, C_TEXT, "NO JETSON");
}

/* DS18B20 values of -127 mean the sensor was not read */
static void set_temp(lv_obj_t *label, float c)
{
    if (c <= -100.0f) set_text(label, "--");
    else              set_text(label, "%.1f\xC2\xB0", c);
}

/* ── Updates ────────────────────────────────────────────────────────────────── */

static void update_status(uint32_t up_ms, uint8_t ok)
{
    if (up_ms < STARTUP_MS) {
        set_text(_status, "STARTING");
        lv_obj_set_style_text_color(_status, C_MUTED, 0);
    } else if ((ok & OK_SENSORS) == OK_SENSORS) {
        set_text(_status, LV_SYMBOL_OK " OK");
        lv_obj_set_style_text_color(_status, C_GREEN, 0);
    } else {
        static const char *const names[SENSOR_ERR_COUNT] = {"INA", "BME", "SGP", "SPS", "VEML", "DS"};
        char text[32] = LV_SYMBOL_WARNING;
        for (int i = 0; i < SENSOR_ERR_COUNT; i++) {
            if (ok & (1 << i)) continue;
            strlcat(text, " ", sizeof(text));
            strlcat(text, names[i], sizeof(text));
        }
        set_text(_status, "%s", text);
        lv_obj_set_style_text_color(_status, C_RED, 0);
    }
}

static void update_ui(void)
{
    uint32_t up_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint8_t  ok    = g_sensors.ok;
    float    amps  = g_sensors.current_a;

    update_status(up_ms, ok);
    set_text(_uptime, "UP %lum", (unsigned long)(up_ms / 60000UL));

    set_text(_volts, "%.2f V", g_sensors.bus_v);
    set_text(_amps, "%.2f A", amps);
    lv_bar_set_value(_load, (int32_t)(amps / LOAD_BAR_MAX_A * 100.0f), LV_ANIM_OFF);

    set_text(_v12, "12V  %.1f", g_sensors.v_12v);
    set_text(_v5, "5V  %.2f", g_sensors.v_5v);
    set_text(_v33, "3.3V  %.2f", g_sensors.v_3v3);

    set_temp(_t_bat, g_sensors.temp_batt_c);
    set_temp(_t_rear, g_sensors.temp_elec_rear_c);
    set_temp(_t_front, g_sensors.temp_elec_front_c);
    switch ((up_ms / AIR_CYCLE_MS) % 3) {
    case 0:
        set_text(_air_name, "AIR");
        set_text(_t_air, "%.1f\xC2\xB0", g_sensors.temp_c);
        break;
    case 1:
        set_text(_air_name, "HUM");
        set_text(_t_air, "%.0f%%", g_sensors.humidity_pct);
        break;
    default:
        set_text(_air_name, "PRES");
        set_text(_t_air, "%.0f hPa", g_sensors.pressure_hpa);
        break;
    }

    set_text(_pm25, "%.0f", g_sensors.pm2_5);
    set_text(_tvoc, "%u", g_sensors.tvoc_ppb);
    set_text(_eco2, "%u", g_sensors.eco2_ppm);
    set_text(_lux, "%.0f", g_sensors.lux);

    char ip_text[16];
    telemetry_get_ip_text(ip_text, sizeof(ip_text));
    set_text(_jetson_ip, "%s", ip_text);
}

/* ── Setup ──────────────────────────────────────────────────────────────────── */

static esp_err_t init_lcd(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num     = PIN_MOSI,
        .miso_io_num     = -1,
        .sclk_io_num     = PIN_SCK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = BUF_PIXELS * 2,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .dc_gpio_num       = PIN_DC,
        .cs_gpio_num       = PIN_CS,
        .pclk_hz           = 20 * 1000 * 1000,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .spi_mode          = 0,
        .trans_queue_depth = 4,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io);
    if (err != ESP_OK) return err;

    esp_lcd_panel_handle_t panel;
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(io, &panel_cfg, &panel);
    if (err != ESP_OK) return err;

    if ((err = esp_lcd_panel_reset(panel)) != ESP_OK) return err;
    if ((err = esp_lcd_panel_init(panel)) != ESP_OK) return err;
    /* This panel shows true colors with inversion off */
    if ((err = esp_lcd_panel_invert_color(panel, false)) != ESP_OK) return err;
    if ((err = esp_lcd_panel_disp_on_off(panel, true)) != ESP_OK) return err;

    /* Below the sensor task's priority, so drawing can't delay sensor reads */
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 1;
    err = lvgl_port_init(&port_cfg);
    if (err != ESP_OK) return err;

    lvgl_port_display_cfg_t disp_cfg = {
        .io_handle     = io,
        .panel_handle  = panel,
        .buffer_size   = BUF_PIXELS,
        .double_buffer = false,
        .hres          = LCD_W,
        .vres          = LCD_H,
        .color_format  = LV_COLOR_FORMAT_RGB565,
        /* Landscape, rotated 180 degrees to suit how the panel is mounted */
        .rotation      = { .swap_xy = true, .mirror_x = false, .mirror_y = true },
        .flags         = { .buff_dma = true, .swap_bytes = true },
    };
    return lvgl_port_add_disp(&disp_cfg) ? ESP_OK : ESP_FAIL;
}

static void display_task(void *arg)
{
    esp_err_t err = init_lcd();
    if (err != ESP_OK) {
        /* A display fault must not take the sensors or alarm down with it */
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
    }

    lvgl_port_lock(0);
    build_ui();
    lvgl_port_unlock();

    ESP_LOGI(TAG, "started");
    while (1) {
        lvgl_port_lock(0);
        update_ui();
        lvgl_port_unlock();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_display_start(void)
{
    xTaskCreate(display_task, "display", 4608, NULL, 1, NULL);
}

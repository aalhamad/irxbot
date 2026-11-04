#include "task_sensors.h"

#include "driver/i2c_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "onewire_bus.h"
#include "ds18b20.h"
#include <string.h>
#include <math.h>

#define TAG "sensors"

/* ── I2C bus ────────────────────────────────────────────────────────────────── */

#define I2C_PORT    I2C_NUM_0
#define PIN_SDA     8
#define PIN_SCL     9
#define I2C_TIMEOUT 100   /* ms */

static i2c_master_bus_handle_t _bus;

static i2c_master_dev_handle_t _ina;
static i2c_master_dev_handle_t _bme;
static i2c_master_dev_handle_t _sgp;
static i2c_master_dev_handle_t _sps;
static i2c_master_dev_handle_t _veml;

/* ── Shared sensor data ─────────────────────────────────────────────────────── */

volatile sensor_data_t g_sensors = {0};

/* ── Read health ────────────────────────────────────────────────────────────── */

#define OK_BIT_INA  0
#define OK_BIT_BME  1
#define OK_BIT_SGP  2
#define OK_BIT_SPS  3
#define OK_BIT_VEML 4
#define OK_BIT_DS   5
#define OK_BIT_MQ2  6

#define FAILS_TO_CLEAR_OK 3   /* consecutive failed reads before an ok bit clears */

static portMUX_TYPE _ok_mux = portMUX_INITIALIZER_UNLOCKED;

/* Every failed read is counted. The ok bit clears after 3 failures in a row
   and sets again on the next good read. */
static void track_read(int bit, bool good)
{
    static const char *const names[SENSOR_ERR_COUNT] = {"INA", "BME", "SGP", "SPS", "VEML", "DS"};
    static uint8_t fails[SENSOR_ERR_COUNT] = {0};
    bool lost = false, recovered = false;

    taskENTER_CRITICAL(&_ok_mux);
    uint8_t mask = (uint8_t)(1 << bit);
    if (good) {
        recovered = (fails[bit] >= FAILS_TO_CLEAR_OK);
        fails[bit] = 0;
        g_sensors.ok |= mask;
    } else {
        g_sensors.err[bit]++;
        if (fails[bit] < 255) fails[bit]++;
        if (fails[bit] == FAILS_TO_CLEAR_OK) {
            g_sensors.ok &= (uint8_t)~mask;
            lost = true;
        }
    }
    taskEXIT_CRITICAL(&_ok_mux);

    if (lost)      ESP_LOGW(TAG, "%s lost: %d failed reads in a row", names[bit], FAILS_TO_CLEAR_OK);
    if (recovered) ESP_LOGW(TAG, "%s recovered", names[bit]);
}

/* ── I2C helpers ────────────────────────────────────────────────────────────── */

static esp_err_t i2c_wr1(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(dev, buf, 2, I2C_TIMEOUT);
}

static esp_err_t i2c_rd(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, I2C_TIMEOUT);
}

/* Send raw bytes (for 16-bit addressed sensors like SGP30, SPS30) */
static esp_err_t i2c_cmd(i2c_master_dev_handle_t dev, const uint8_t *cmd, size_t len)
{
    return i2c_master_transmit(dev, cmd, len, I2C_TIMEOUT);
}

static esp_err_t i2c_recv(i2c_master_dev_handle_t dev, uint8_t *buf, size_t len)
{
    return i2c_master_receive(dev, buf, len, I2C_TIMEOUT);
}

/* ── INA228 ─────────────────────────────────────────────────────────────────── */

#define INA_ADDR      0x40
#define INA_REG_VSHUNT 0x04
#define INA_REG_VBUS   0x05
#define INA_SHUNT_OHM  0.002f

static bool init_ina228(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = INA_ADDR,
        .scl_speed_hz    = 10000,
    };
    if (i2c_master_bus_add_device(_bus, &cfg, &_ina) != ESP_OK) return false;

    /* Verify device is present — retry and log, since this read fails intermittently */
    uint8_t buf[2];
    bool id_ok = false;
    for (int i = 1; i <= 5 && !id_ok; i++) {
        esp_err_t err = i2c_rd(_ina, 0x3E, buf, 2);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "INA ID try %d: OK 0x%02X%02X", i, buf[0], buf[1]);
            id_ok = true;
        } else {
            ESP_LOGW(TAG, "INA ID try %d: %s", i, esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    if (!id_ok) return false;

    /* Software reset */
    uint8_t rst[3] = {0x00, 0x80, 0x00};
    i2c_master_transmit(_ina, rst, 3, I2C_TIMEOUT);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* ADC_CONFIG (0x01): continuous shunt+bus+temp, 1052µs conversion, no averaging */
    uint8_t adc[3] = {0x01, 0xFB, 0x68};
    i2c_master_transmit(_ina, adc, 3, I2C_TIMEOUT);
    vTaskDelay(pdMS_TO_TICKS(20));   /* wait for first conversion to complete */

    return true;
}

static bool read_ina228(void)
{
    uint8_t buf[3];

    if (i2c_rd(_ina, INA_REG_VBUS, buf, 3) != ESP_OK) return false;
    uint32_t raw20 = (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]) >> 4;
    float vbus = raw20 * 0.0001953125f;

    if (i2c_rd(_ina, INA_REG_VSHUNT, buf, 3) != ESP_OK) return false;
    int32_t rs = (int32_t)((((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]) >> 4);
    if (rs & 0x80000) rs |= (int32_t)0xFFF00000;
    float vshunt = rs * 0.0000003125f;
    float current = -vshunt / INA_SHUNT_OHM;

    g_sensors.bus_v     = vbus;
    g_sensors.current_a = current;
    g_sensors.power_w   = vbus * current;
    return true;
}

/* ── BME280 ─────────────────────────────────────────────────────────────────── */

#define BME_ADDR 0x76

typedef struct {
    uint16_t T1; int16_t T2, T3;
    uint16_t P1; int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t  H1; int16_t H2; uint8_t H3;
    int16_t  H4, H5; int8_t H6;
    int32_t  t_fine;
} bme280_cal_t;

static bme280_cal_t _bme_cal;

static void bme280_read_cal(void)
{
    uint8_t b[24];
    i2c_rd(_bme, 0x88, b, 24);
    _bme_cal.T1 = (uint16_t)(b[0]  | (b[1]  << 8));
    _bme_cal.T2 = (int16_t) (b[2]  | (b[3]  << 8));
    _bme_cal.T3 = (int16_t) (b[4]  | (b[5]  << 8));
    _bme_cal.P1 = (uint16_t)(b[6]  | (b[7]  << 8));
    _bme_cal.P2 = (int16_t) (b[8]  | (b[9]  << 8));
    _bme_cal.P3 = (int16_t) (b[10] | (b[11] << 8));
    _bme_cal.P4 = (int16_t) (b[12] | (b[13] << 8));
    _bme_cal.P5 = (int16_t) (b[14] | (b[15] << 8));
    _bme_cal.P6 = (int16_t) (b[16] | (b[17] << 8));
    _bme_cal.P7 = (int16_t) (b[18] | (b[19] << 8));
    _bme_cal.P8 = (int16_t) (b[20] | (b[21] << 8));
    _bme_cal.P9 = (int16_t) (b[22] | (b[23] << 8));

    uint8_t h1;
    i2c_rd(_bme, 0xA1, &h1, 1);
    _bme_cal.H1 = h1;

    uint8_t hb[7];
    i2c_rd(_bme, 0xE1, hb, 7);
    _bme_cal.H2 = (int16_t)(hb[0] | (hb[1] << 8));
    _bme_cal.H3 = hb[2];
    _bme_cal.H4 = (int16_t)((hb[3] << 4) | (hb[4] & 0x0F));
    _bme_cal.H5 = (int16_t)((hb[5] << 4) | (hb[4] >> 4));
    _bme_cal.H6 = (int8_t)hb[6];
}

static float bme280_comp_temp(int32_t adc_T)
{
    int32_t v1 = ((adc_T >> 3) - ((int32_t)_bme_cal.T1 << 1)) * (int32_t)_bme_cal.T2 >> 11;
    int32_t v2 = (((adc_T >> 4) - (int32_t)_bme_cal.T1) *
                  ((adc_T >> 4) - (int32_t)_bme_cal.T1) >> 12) * (int32_t)_bme_cal.T3 >> 14;
    _bme_cal.t_fine = v1 + v2;
    return ((float)((_bme_cal.t_fine * 5 + 128) >> 8)) / 100.0f;
}

static float bme280_comp_pressure(int32_t adc_P)
{
    int64_t v1 = (int64_t)_bme_cal.t_fine - 128000;
    int64_t v2 = v1 * v1 * (int64_t)_bme_cal.P6;
    v2 += (v1 * (int64_t)_bme_cal.P5) << 17;
    v2 += (int64_t)_bme_cal.P4 << 35;
    v1  = ((v1 * v1 * (int64_t)_bme_cal.P3) >> 8) + ((v1 * (int64_t)_bme_cal.P2) << 12);
    v1  = (((int64_t)1 << 47) + v1) * (int64_t)_bme_cal.P1 >> 33;
    if (v1 == 0) return 0.0f;
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = ((int64_t)_bme_cal.P9 * (p >> 13) * (p >> 13)) >> 25;
    v2 = ((int64_t)_bme_cal.P8 * p) >> 19;
    p  = ((p + v1 + v2) >> 8) + ((int64_t)_bme_cal.P7 << 4);
    return (float)((uint32_t)p) / 25600.0f;   /* hPa */
}

static float bme280_comp_humidity(int32_t adc_H)
{
    int32_t x = _bme_cal.t_fine - 76800;
    x = (((adc_H << 14) - ((int32_t)_bme_cal.H4 << 20) - ((int32_t)_bme_cal.H5 * x) + 16384) >> 15)
        * (((((((x * (int32_t)_bme_cal.H6) >> 10)
              * (((x * (int32_t)_bme_cal.H3) >> 11) + 32768)) >> 10) + 2097152)
              * (int32_t)_bme_cal.H2 + 8192) >> 14);
    x -= (((x >> 15) * (x >> 15)) >> 7) * (int32_t)_bme_cal.H1 >> 4;
    if (x < 0) x = 0;
    if (x > 419430400) x = 419430400;
    return (float)(x >> 12) / 1024.0f;
}

static bool init_bme280(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = BME_ADDR,
        .scl_speed_hz    = 100000,
    };
    if (i2c_master_bus_add_device(_bus, &cfg, &_bme) != ESP_OK) return false;

    /* Check chip ID */
    uint8_t id;
    if (i2c_rd(_bme, 0xD0, &id, 1) != ESP_OK || (id != 0x60 && id != 0x58)) {
        /* Try 0x77 */
        i2c_master_bus_rm_device(_bme);
        cfg.device_address = 0x77;
        if (i2c_master_bus_add_device(_bus, &cfg, &_bme) != ESP_OK) return false;
        if (i2c_rd(_bme, 0xD0, &id, 1) != ESP_OK || (id != 0x60 && id != 0x58)) return false;
    }

    bme280_read_cal();
    return true;
}

static bool read_bme280(void)
{
    /* Forced mode: humidity x1, temp x1, pressure x1 */
    if (i2c_wr1(_bme, 0xF2, 0x01) != ESP_OK) return false;
    if (i2c_wr1(_bme, 0xF4, 0x25) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(20));   /* a 10 ms delay can be under one tick; conversion needs ~10 ms */

    uint8_t raw[8];
    if (i2c_rd(_bme, 0xF7, raw, 8) != ESP_OK) return false;

    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = ((int32_t)raw[6] << 8)  |  raw[7];

    g_sensors.temp_c        = bme280_comp_temp(adc_T);
    g_sensors.pressure_hpa  = bme280_comp_pressure(adc_P);
    g_sensors.humidity_pct  = bme280_comp_humidity(adc_H);
    return true;
}

/* ── SGP30 ──────────────────────────────────────────────────────────────────── */

#define SGP_ADDR 0x58

static uint8_t sgp30_crc(uint8_t a, uint8_t b)
{
    uint8_t crc = 0xFF;
    crc ^= a; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    crc ^= b; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    return crc;
}

static bool init_sgp30(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SGP_ADDR,
        .scl_speed_hz    = 10000,
    };
    if (i2c_master_bus_add_device(_bus, &cfg, &_sgp) != ESP_OK) return false;

    uint8_t cmd[2] = {0x20, 0x03};  /* Init_air_quality */
    return i2c_cmd(_sgp, cmd, 2) == ESP_OK;
}

static bool read_sgp30(void)
{
    uint8_t cmd[2] = {0x20, 0x08};  /* Measure_air_quality */
    if (i2c_cmd(_sgp, cmd, 2) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(20));   /* SGP30 needs 12ms; 20ms = 2 ticks at 100Hz */

    uint8_t buf[6];
    if (i2c_recv(_sgp, buf, 6) != ESP_OK) return false;

    /* buf: eCO2_H, eCO2_L, CRC, TVOC_H, TVOC_L, CRC */
    if (sgp30_crc(buf[0], buf[1]) != buf[2]) return false;
    if (sgp30_crc(buf[3], buf[4]) != buf[5]) return false;

    g_sensors.eco2_ppm  = ((uint16_t)buf[0] << 8) | buf[1];
    g_sensors.tvoc_ppb  = ((uint16_t)buf[3] << 8) | buf[4];
    return true;
}

/* ── SPS30 ──────────────────────────────────────────────────────────────────── */

#define SPS_ADDR 0x69

static uint8_t sps30_crc(uint8_t a, uint8_t b)
{
    uint8_t crc = 0xFF;
    crc ^= a; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    crc ^= b; for (int i = 0; i < 8; i++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    return crc;
}

static bool init_sps30(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SPS_ADDR,
        .scl_speed_hz    = 100000,
    };
    if (i2c_master_bus_add_device(_bus, &cfg, &_sps) != ESP_OK) return false;

    /* Start measurement in float mode */
    uint8_t cmd[5] = {0x00, 0x10, 0x03, 0x00, sps30_crc(0x03, 0x00)};
    return i2c_cmd(_sps, cmd, 5) == ESP_OK;
}

/* Returns false only for a bus or CRC failure. Data not being ready yet is not
   a failure: the previous values stay and the read still counts as good. */
static bool read_sps30(void)
{
    uint8_t ready_cmd[2] = {0x02, 0x02};
    if (i2c_cmd(_sps, ready_cmd, 2) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(10));   /* 5ms rounds to 0 ticks at 100Hz */

    uint8_t ready[3];
    if (i2c_recv(_sps, ready, 3) != ESP_OK) return false;
    if (sps30_crc(ready[0], ready[1]) != ready[2]) return false;
    if (ready[1] != 1) return true;

    uint8_t cmd[2] = {0x03, 0x00};
    if (i2c_cmd(_sps, cmd, 2) != ESP_OK) return false;
    vTaskDelay(pdMS_TO_TICKS(10));   /* 5ms rounds to 0 ticks at 100Hz */

    uint8_t buf[60];
    if (i2c_recv(_sps, buf, 60) != ESP_OK) return false;

    /* Parse 10 floats (each is 2 words × 3 bytes: data_H, data_L, CRC) */
    float vals[10];
    for (int i = 0; i < 10; i++) {
        uint8_t *w = buf + i * 6;
        if (sps30_crc(w[0], w[1]) != w[2]) return false;
        if (sps30_crc(w[3], w[4]) != w[5]) return false;
        uint32_t raw = ((uint32_t)w[0] << 24) | ((uint32_t)w[1] << 16) |
                       ((uint32_t)w[3] << 8)  |  w[4];
        memcpy(&vals[i], &raw, 4);
    }

    g_sensors.pm2_5  = vals[1];
    g_sensors.pm10   = vals[3];
    return true;
}

/* ── VEML7700 ───────────────────────────────────────────────────────────────── */

#define VEML_ADDR 0x10

static bool init_veml7700(void)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = VEML_ADDR,
        .scl_speed_hz    = 10000,
    };
    if (i2c_master_bus_add_device(_bus, &cfg, &_veml) != ESP_OK) return false;

    /* ALS_CONF: gain=1 (0x00), IT=100ms (0x00), power on (SD=0) */
    uint8_t buf[3] = {0x00, 0x00, 0x00};
    return i2c_master_transmit(_veml, buf, 3, I2C_TIMEOUT) == ESP_OK;
}

static bool read_veml7700(void)
{
    static uint32_t reenable_count = 0;
    uint8_t buf[2];

    /* Shutdown guard: if ALS_SD (bit 0) got set, turn the sensor back on.
       Skip this reading, since the next integration hasn't finished yet. */
    if (i2c_rd(_veml, 0x00, buf, 2) != ESP_OK) return false;
    if (buf[0] & 0x01) {
        uint8_t on[3] = {0x00, 0x00, 0x00};
        if (i2c_master_transmit(_veml, on, 3, I2C_TIMEOUT) != ESP_OK) return false;
        reenable_count++;
        ESP_LOGW(TAG, "VEML re-enabled, count=%lu", (unsigned long)reenable_count);
        return true;
    }

    if (i2c_rd(_veml, 0x04, buf, 2) != ESP_OK) return false;
    uint16_t als = (uint16_t)(buf[0] | (buf[1] << 8));
    g_sensors.lux  = als * 0.0576f;   /* gain=1, IT=100ms resolution */
    return true;
}

/* ── DS18B20 ────────────────────────────────────────────────────────────────── */

#define DS_GPIO 4
#define DS_MAX  3

static ds18b20_device_handle_t _ds[DS_MAX];
static int _ds_count = 0;
static onewire_bus_handle_t _ds_bus = NULL;

/* Finds the sensors on the bus again, so one plugged in or powered after boot
   is picked up without restarting the ESP32. Sensors are listed in ROM-code
   order, which stays the same while the same sensors are fitted. */
static void scan_ds18b20(void)
{
    for (int i = 0; i < _ds_count; i++) ds18b20_del_device(_ds[i]);
    _ds_count = 0;

    onewire_device_iter_handle_t iter;
    if (onewire_new_device_iter(_ds_bus, &iter) != ESP_OK) return;
    onewire_device_t dev;
    while (_ds_count < DS_MAX && onewire_device_iter_get_next(iter, &dev) == ESP_OK) {
        ds18b20_config_t ds_cfg = {};
        if (ds18b20_new_device_from_enumeration(&dev, &ds_cfg, &_ds[_ds_count]) == ESP_OK) {
            ESP_LOGI(TAG, "DS18B20 %d: %016llX", _ds_count, (unsigned long long)dev.address);
            _ds_count++;
        }
    }
    onewire_del_device_iter(iter);
}

static bool init_ds18b20(void)
{
    onewire_bus_config_t bus_cfg     = { .bus_gpio_num = DS_GPIO };
    onewire_bus_rmt_config_t rmt_cfg = { .max_rx_bytes = 10 };
    if (onewire_new_bus_rmt(&bus_cfg, &rmt_cfg, &_ds_bus) != ESP_OK) {
        _ds_bus = NULL;
        return false;
    }
    scan_ds18b20();
    return _ds_count > 0;
}

/* A sensor that fails to read shows -127, the value for "not read", so the
   LCD and telemetry never keep showing an old temperature. */
#define DS_NOT_READ -127.0f

static bool read_ds18b20(void)
{
    static int last_count = -1;
    float t[DS_MAX] = {DS_NOT_READ, DS_NOT_READ, DS_NOT_READ};
    bool all_good = false;

    /* Starts all sensors together; the driver waits 800 ms for the conversion */
    if (_ds_count > 0 && ds18b20_trigger_temperature_conversion_for_all(_ds_bus) == ESP_OK) {
        all_good = true;
        for (int i = 0; i < _ds_count; i++) {
            if (ds18b20_get_temperature(_ds[i], &t[i]) != ESP_OK) {
                t[i] = DS_NOT_READ;
                all_good = false;
            }
        }
    }

    g_sensors.temp_batt_c       = t[0];
    g_sensors.temp_elec_rear_c  = t[1];
    g_sensors.temp_elec_front_c = t[2];

    /* Look for missing or reconnected sensors before the next read */
    if (!all_good || _ds_count < DS_MAX) scan_ds18b20();
    if (_ds_count != last_count) {
        ESP_LOGW(TAG, "DS18B20: %d found", _ds_count);
        last_count = _ds_count;
    }
    return all_good && _ds_count > 0;
}

/* ── MQ-2 ───────────────────────────────────────────────────────────────────── */

#define MQ2_CHANNEL    ADC_CHANNEL_0   /* GPIO 1 = ADC1_CH0 */
#define RAIL_CH_12V    ADC_CHANNEL_1   /* GPIO 2 = ADC1_CH1 */
#define RAIL_CH_5V     ADC_CHANNEL_5   /* GPIO 6 = ADC1_CH5 */
#define RAIL_CH_3V3    ADC_CHANNEL_6   /* GPIO 7 = ADC1_CH6 */
#define ADC_MV_FULL    3100.0f         /* DB_12 full scale mV */
#define MQ2_WARMUP_MS  180000UL

static adc_oneshot_unit_handle_t _adc;

/* per-channel calibration handles — ESP32-S3 curve fitting is per-channel */
static adc_cali_handle_t _cali_ch0 = NULL;   /* MQ2   GPIO1 */
static adc_cali_handle_t _cali_ch1 = NULL;   /* 12V   GPIO2 */
static adc_cali_handle_t _cali_ch5 = NULL;   /* 5V    GPIO6 */
static adc_cali_handle_t _cali_ch6 = NULL;   /* 3V3   GPIO7 */

static float    _mq2_baseline  = 0.0f;
static bool     _mq2_base_set  = false;
static uint32_t _mq2_boot_tick = 0;

static adc_cali_handle_t create_cali(adc_channel_t ch)
{
    adc_cali_handle_t h = NULL;
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = ch,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_cali_create_scheme_curve_fitting(&cfg, &h);
    return h;
}

static void init_mq2(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &_adc);

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(_adc, MQ2_CHANNEL,  &ch_cfg);
    adc_oneshot_config_channel(_adc, RAIL_CH_12V,  &ch_cfg);
    adc_oneshot_config_channel(_adc, RAIL_CH_5V,   &ch_cfg);
    adc_oneshot_config_channel(_adc, RAIL_CH_3V3,  &ch_cfg);

    _cali_ch0 = create_cali(ADC_CHANNEL_0);
    _cali_ch1 = create_cali(ADC_CHANNEL_1);
    _cali_ch5 = create_cali(ADC_CHANNEL_5);
    _cali_ch6 = create_cali(ADC_CHANNEL_6);

    if (!_cali_ch0 && !_cali_ch1 && !_cali_ch5 && !_cali_ch6)
        ESP_LOGW(TAG, "ADC cali unavailable, using raw");

    _mq2_boot_tick = xTaskGetTickCount();
}

static int adc_raw_to_mv(adc_channel_t ch)
{
    int raw;
    adc_oneshot_read(_adc, ch, &raw);

    adc_cali_handle_t h = NULL;
    switch (ch) {
        case ADC_CHANNEL_0: h = _cali_ch0; break;
        case ADC_CHANNEL_1: h = _cali_ch1; break;
        case ADC_CHANNEL_5: h = _cali_ch5; break;
        case ADC_CHANNEL_6: h = _cali_ch6; break;
        default: break;
    }
    if (h) {
        int mv;
        adc_cali_raw_to_voltage(h, raw, &mv);
        return mv;
    }
    return (int)(raw * ADC_MV_FULL / 4095.0f);
}

static float mq2_read_volts(void)
{
    int sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_raw_to_mv(MQ2_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (sum / 16.0f) / 1000.0f;
}

static float adc_to_volts(adc_channel_t ch, float ratio)
{
    /* Average 16 samples, with no delay so the sensor loop stays on time */
    int sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_raw_to_mv(ch);
    }
    return (sum / 16.0f / 1000.0f) * ratio;
}

static void read_rails(void)
{
    g_sensors.v_12v  = adc_to_volts(RAIL_CH_12V, 6.6f);   /* 56k/10k */
    g_sensors.v_5v   = adc_to_volts(RAIL_CH_5V,  3.2f);   /* 22k/10k */
    g_sensors.v_3v3  = adc_to_volts(RAIL_CH_3V3, 2.0f);   /* 10k/10k */
}

static void read_mq2(void)
{
    uint32_t elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - _mq2_boot_tick);

    float v = mq2_read_volts();
    g_sensors.mq2_v = v;

    if (elapsed_ms < MQ2_WARMUP_MS) {
        g_sensors.mq2_status = 0;
        return;
    }

    taskENTER_CRITICAL(&_ok_mux);
    g_sensors.ok |= (1 << OK_BIT_MQ2);   /* mq2 warmed */
    taskEXIT_CRITICAL(&_ok_mux);

    if (!_mq2_base_set) {
        _mq2_baseline = v;
        _mq2_base_set  = true;
    }

    if (_mq2_baseline > 0.01f) {
        float rise = (v - _mq2_baseline) / _mq2_baseline * 100.0f;
        g_sensors.mq2_status = (rise < 20) ? 0 : (rise < 60) ? 1 : 2;
    }
}

/* ── Task ───────────────────────────────────────────────────────────────────── */

static void sensors_task(void *arg)
{
    /* Init I2C master bus */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port            = I2C_PORT,
        .sda_io_num          = PIN_SDA,
        .scl_io_num          = PIN_SCL,
        .clk_source          = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt   = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &_bus));
    esp_log_level_set("i2c.master", ESP_LOG_NONE);   /* suppress driver-level NACK noise */

    /* The first transaction on a fresh bus has been failing: send throwaway
       probes to an empty address so no sensor init is the first one. */
    for (int i = 0; i < 3; i++) {
        i2c_master_probe(_bus, 0x01, I2C_TIMEOUT);
    }

    /* Init sensors — INA228 last so other sensors warm up the bus first */
    bool have_bme  = init_bme280();
    bool have_sgp  = init_sgp30();
    bool have_sps  = init_sps30();
    bool have_veml = init_veml7700();
    bool have_ds   = init_ds18b20();
    bool have_ina  = init_ina228();

    ESP_LOGI(TAG, "ina=%d bme=%d sgp=%d sps=%d veml=%d ds=%d(%d)",
             have_ina, have_bme, have_sgp, have_sps, have_veml, have_ds, _ds_count);

    init_mq2();
    ESP_LOGI(TAG, "MQ-2 init, warming up 3 min");

    uint32_t tick = 0;

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        tick++;

        /* Every 1 s */
        /* Keep SGP30 immediately before INA228: running them on separate timers
           caused repeating INA failures and bad Lux, cause not yet known. */
        if (have_sgp) track_read(OK_BIT_SGP, read_sgp30());
        if (have_ina) track_read(OK_BIT_INA, read_ina228());

        /* Every 2 s */
        if (tick % 2 == 0) {
            if (have_veml) track_read(OK_BIT_VEML, read_veml7700());
            read_mq2();
            read_rails();
        }

        /* Every 5 s */
        if (tick % 5 == 0 && have_sps) track_read(OK_BIT_SPS, read_sps30());

        /* Every 10 s */
        if (tick % 10 == 0) {
            if (have_bme) track_read(OK_BIT_BME, read_bme280());
            if (_ds_bus)  track_read(OK_BIT_DS, read_ds18b20());  /* rescans, so runs even with none found at boot */
        }

        if (tick % 5 == 0) {
            ESP_LOGI(TAG, "Vbat=%.2f I=%.3fA | 12V=%.2f 5V=%.2f 3V3=%.2f | T=%.1fC H=%.0f%% | TVOC=%u eCO2=%u | PM2.5=%.1f | Lux=%.0f | Tbatt=%.1f Trear=%.1f Tfront=%.1f | MQ2=%.3fV st=%u | ok=0x%02X err=%u,%u,%u,%u,%u,%u",
                g_sensors.bus_v, g_sensors.current_a,
                g_sensors.v_12v, g_sensors.v_5v, g_sensors.v_3v3,
                g_sensors.temp_c, g_sensors.humidity_pct,
                g_sensors.tvoc_ppb, g_sensors.eco2_ppm,
                g_sensors.pm2_5, g_sensors.lux,
                g_sensors.temp_batt_c, g_sensors.temp_elec_rear_c, g_sensors.temp_elec_front_c,
                g_sensors.mq2_v, g_sensors.mq2_status, g_sensors.ok,
                g_sensors.err[OK_BIT_INA], g_sensors.err[OK_BIT_BME], g_sensors.err[OK_BIT_SGP],
                g_sensors.err[OK_BIT_SPS], g_sensors.err[OK_BIT_VEML], g_sensors.err[OK_BIT_DS]);
        }
    }
}

/* ── Init ───────────────────────────────────────────────────────────────────── */

void task_sensors_start(void)
{
    xTaskCreate(sensors_task, "sensors", 8192, NULL, 3, NULL);
    ESP_LOGI(TAG, "started");
}

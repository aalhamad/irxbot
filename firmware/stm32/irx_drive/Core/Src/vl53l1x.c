/*
 * vl53l1x.c
 *
 * Minimal VL53L1X driver for IRXBot.
 * - 4 sensors on shared I2C1, XSHUT per sensor for address assignment
 * - Init sequence and default configuration taken from ST's ULD (STSW-IMG009)
 * - Continuous ranging, short distance mode, 50ms timing budget
 *
 * Fault-tolerant: any sensor that fails init or stops ACKing is marked
 * offline; the rest keep working. Nothing here blocks indefinitely.
 */

#include "vl53l1x.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;

/* ── XSHUT pin map (must match CubeMX labels) ──────────────────────────────── */
typedef struct { GPIO_TypeDef *port; uint16_t pin; } xshut_t;

static const xshut_t _xshut[TOF_COUNT] = {
    { TOF_XSHUT_FRONT_GPIO_Port, TOF_XSHUT_FRONT_Pin },
    { TOF_XSHUT_REAR_GPIO_Port,  TOF_XSHUT_REAR_Pin  },
    { TOF_XSHUT_LEFT_GPIO_Port,  TOF_XSHUT_LEFT_Pin  },
    { TOF_XSHUT_RIGHT_GPIO_Port, TOF_XSHUT_RIGHT_Pin },
};

static I2C_HandleTypeDef *_hi2c = NULL;
static ToF_Data_t         _data[TOF_COUNT];

#define VL53L1X_DEFAULT_ADDR   (0x29 << 1)   /* factory address, HAL 8-bit form */
#define ADDR8(idx)             ((uint16_t)((TOF_ADDR_BASE + (idx)) << 1))

/* Key registers */
#define REG_I2C_SLAVE_DEVICE_ADDRESS            0x0001
#define REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND 0x0008
#define REG_SYSTEM_INTERRUPT_CLEAR              0x0086
#define REG_SYSTEM_MODE_START                   0x0087
#define REG_GPIO_TIO_HV_STATUS                  0x0031
#define REG_RESULT_RANGE_STATUS                 0x0089
#define REG_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0  0x0096
#define REG_IDENTIFICATION_MODEL_ID             0x010F
#define REG_FIRMWARE_SYSTEM_STATUS              0x00E5
#define REG_RESULT_OSC_CALIBRATE_VAL            0x00DE
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD      0x006C
#define REG_PHASECAL_CONFIG_TIMEOUT_MACROP      0x004B
#define REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI    0x005E
#define REG_RANGE_CONFIG_VCSEL_PERIOD_A         0x0060
#define REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI    0x0061
#define REG_RANGE_CONFIG_VCSEL_PERIOD_B         0x0063
#define REG_RANGE_CONFIG_VALID_PHASE_HIGH       0x0069
#define REG_SD_CONFIG_WOI_SD0                   0x0078
#define REG_SD_CONFIG_INITIAL_PHASE_SD0         0x007A

/* ── Low-level I2C (16-bit register index) ─────────────────────────────────── */

static HAL_StatusTypeDef wr_bytes(uint16_t addr8, uint16_t reg, const uint8_t *data, uint16_t n)
{
    uint8_t buf[6];
    if (n > 4) return HAL_ERROR;
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    memcpy(&buf[2], data, n);
    return HAL_I2C_Master_Transmit(_hi2c, addr8, buf, (uint16_t)(2 + n), 50);
}

static HAL_StatusTypeDef rd_bytes(uint16_t addr8, uint16_t reg, uint8_t *data, uint16_t n)
{
    uint8_t idx[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    if (HAL_I2C_Master_Transmit(_hi2c, addr8, idx, 2, 50) != HAL_OK) return HAL_ERROR;
    return HAL_I2C_Master_Receive(_hi2c, addr8, data, n, 50);
}

static HAL_StatusTypeDef wr8 (uint16_t a, uint16_t r, uint8_t v)  { return wr_bytes(a, r, &v, 1); }
static HAL_StatusTypeDef wr16(uint16_t a, uint16_t r, uint16_t v) { uint8_t b[2]={(uint8_t)(v>>8),(uint8_t)v}; return wr_bytes(a, r, b, 2); }
static HAL_StatusTypeDef rd8 (uint16_t a, uint16_t r, uint8_t *v) { return rd_bytes(a, r, v, 1); }
static HAL_StatusTypeDef rd16(uint16_t a, uint16_t r, uint16_t *v){ uint8_t b[2]; HAL_StatusTypeDef s=rd_bytes(a,r,b,2); *v=((uint16_t)b[0]<<8)|b[1]; return s; }

/* ── ST ULD default configuration, registers 0x2D..0x87 (91 bytes, verbatim) ── */
static const uint8_t VL51L1X_DEFAULT_CONFIGURATION[] = {
0x00, /* 0x2d */
0x00, /* 0x2e */
0x00, /* 0x2f */
0x01, /* 0x30 : bit4=0 -> active-high interrupt */
0x02, /* 0x31 */
0x00, /* 0x32 */
0x02, /* 0x33 */
0x08, /* 0x34 */
0x00, /* 0x35 */
0x08, /* 0x36 */
0x10, /* 0x37 */
0x01, /* 0x38 */
0x01, /* 0x39 */
0x00, /* 0x3a */
0x00, /* 0x3b */
0x00, /* 0x3c */
0x00, /* 0x3d */
0xff, /* 0x3e */
0x00, /* 0x3f */
0x0F, /* 0x40 */
0x00, /* 0x41 */
0x00, /* 0x42 */
0x00, /* 0x43 */
0x00, /* 0x44 */
0x00, /* 0x45 */
0x20, /* 0x46 : new sample ready interrupt */
0x0b, /* 0x47 */
0x00, /* 0x48 */
0x00, /* 0x49 */
0x02, /* 0x4a */
0x0a, /* 0x4b */
0x21, /* 0x4c */
0x00, /* 0x4d */
0x00, /* 0x4e */
0x05, /* 0x4f */
0x00, /* 0x50 */
0x00, /* 0x51 */
0x00, /* 0x52 */
0x00, /* 0x53 */
0xc8, /* 0x54 */
0x00, /* 0x55 */
0x00, /* 0x56 */
0x38, /* 0x57 */
0xff, /* 0x58 */
0x01, /* 0x59 */
0x00, /* 0x5a */
0x08, /* 0x5b */
0x00, /* 0x5c */
0x00, /* 0x5d */
0x01, /* 0x5e */
0xcc, /* 0x5f */
0x0f, /* 0x60 */
0x01, /* 0x61 */
0xf1, /* 0x62 */
0x0d, /* 0x63 */
0x01, /* 0x64 : sigma threshold MSB */
0x68, /* 0x65 : sigma threshold LSB */
0x00, /* 0x66 : min count rate MSB */
0x80, /* 0x67 : min count rate LSB */
0x08, /* 0x68 */
0xb8, /* 0x69 */
0x00, /* 0x6a */
0x00, /* 0x6b */
0x00, /* 0x6c : intermeasurement period (32-bit) */
0x00, /* 0x6d */
0x0f, /* 0x6e */
0x89, /* 0x6f */
0x00, /* 0x70 */
0x00, /* 0x71 */
0x00, /* 0x72 */
0x00, /* 0x73 */
0x00, /* 0x74 */
0x00, /* 0x75 */
0x00, /* 0x76 */
0x01, /* 0x77 */
0x0f, /* 0x78 */
0x0d, /* 0x79 */
0x0e, /* 0x7a */
0x0e, /* 0x7b */
0x00, /* 0x7c */
0x00, /* 0x7d */
0x02, /* 0x7e */
0xc7, /* 0x7f : ROI center */
0xff, /* 0x80 : ROI XY */
0x9B, /* 0x81 */
0x00, /* 0x82 */
0x00, /* 0x83 */
0x00, /* 0x84 */
0x01, /* 0x85 */
0x00, /* 0x86 : clear interrupt */
0x00  /* 0x87 : start ranging (0x00 = stopped) */
};

/* ── Data-ready check, ULD style: active-high polarity -> ready when bit0==1 ── */
static uint8_t data_ready(uint16_t addr8)
{
    uint8_t v = 0;
    if (rd8(addr8, REG_GPIO_TIO_HV_STATUS, &v) != HAL_OK) return 0xFF; /* comm error */
    return (v & 0x01) ? 1 : 0;
}

/* ── Per-sensor bring-up at its final address (mirrors ULD SensorInit) ─────── */

static HAL_StatusTypeDef sensor_setup(uint16_t addr8)
{
    /* Verify identity */
    uint16_t id = 0;
    if (rd16(addr8, REG_IDENTIFICATION_MODEL_ID, &id) != HAL_OK) return HAL_ERROR;
    if (id != 0xEACC) return HAL_ERROR;

    /* Stop any ongoing ranging (warm-boot case) */
    wr8(addr8, REG_SYSTEM_MODE_START, 0x00);
    HAL_Delay(3);

    /* Wait for firmware boot (bounded) */
    uint8_t st = 0;
    uint32_t t0 = HAL_GetTick();
    do {
        if (rd8(addr8, REG_FIRMWARE_SYSTEM_STATUS, &st) != HAL_OK) return HAL_ERROR;
        if (st & 0x01) break;
        HAL_Delay(2);
    } while ((HAL_GetTick() - t0) < 150);
    if (!(st & 0x01)) return HAL_ERROR;

    /* Load ULD default configuration: regs 0x2D..0x87 */
    for (uint16_t i = 0; i < sizeof(VL51L1X_DEFAULT_CONFIGURATION); i++)
    {
        if (wr8(addr8, (uint16_t)(0x2D + i), VL51L1X_DEFAULT_CONFIGURATION[i]) != HAL_OK)
            return HAL_ERROR;
    }

    /* ULD SensorInit: one throwaway measurement to run VHV calibration */
    wr8(addr8, REG_SYSTEM_MODE_START, 0x40);
    t0 = HAL_GetTick();
    while (data_ready(addr8) != 1)
    {
        if ((HAL_GetTick() - t0) > 250) return HAL_ERROR;  /* bounded */
        HAL_Delay(2);
    }
    wr8(addr8, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(addr8, REG_SYSTEM_MODE_START,      0x00);
    wr8(addr8, REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09); /* two-bound VHV */
    wr8(addr8, 0x000B, 0x00);  /* start VHV from previous temperature */

    /* Distance mode: short (ULD SetDistanceMode) */
    wr8 (addr8, REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14);
    wr8 (addr8, REG_RANGE_CONFIG_VCSEL_PERIOD_A,    0x07);
    wr8 (addr8, REG_RANGE_CONFIG_VCSEL_PERIOD_B,    0x05);
    wr8 (addr8, REG_RANGE_CONFIG_VALID_PHASE_HIGH,  0x38);
    wr16(addr8, REG_SD_CONFIG_WOI_SD0,              0x0705);
    wr16(addr8, REG_SD_CONFIG_INITIAL_PHASE_SD0,    0x0606);

    /* Timing budget 50ms, short mode (ULD SetTimingBudgetInMs) */
    wr16(addr8, REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, 0x01AE);
    wr16(addr8, REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, 0x01E8);

    /* Inter-measurement period 55ms (ULD SetInterMeasurementInMs):
       IMP = ms * (osc_calibrate_val & 0x3FF) * 1.075 */
    {
        uint16_t pll = 0;
        rd16(addr8, REG_RESULT_OSC_CALIBRATE_VAL, &pll);
        pll &= 0x3FF;
        uint32_t imp = (uint32_t)(pll * 55 * 1.075f);
        uint8_t b[4] = { (uint8_t)(imp>>24), (uint8_t)(imp>>16), (uint8_t)(imp>>8), (uint8_t)imp };
        wr_bytes(addr8, REG_SYSTEM_INTERMEASUREMENT_PERIOD, b, 4);
    }

    /* Clear interrupt and start continuous ranging */
    wr8(addr8, REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(addr8, REG_SYSTEM_MODE_START,      0x40);

    return HAL_OK;
}

/* ── Public API ────────────────────────────────────────────────────────────── */

uint8_t ToF_InitAll(I2C_HandleTypeDef *hi2c)
{
    _hi2c = hi2c;
    memset(_data, 0, sizeof(_data));

    /* All sensors into shutdown */
    for (int i = 0; i < TOF_COUNT; i++)
        HAL_GPIO_WritePin(_xshut[i].port, _xshut[i].pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    uint8_t online = 0;

    for (int i = 0; i < TOF_COUNT; i++)
    {
        /* Wake this one only (others still down or already re-addressed) */
        HAL_GPIO_WritePin(_xshut[i].port, _xshut[i].pin, GPIO_PIN_SET);
        HAL_Delay(3);   /* t_boot ~1.2ms */

        /* Warm reset case: sensor kept power through MCU reset and still
           holds its re-assigned address. Re-setup and move on. */
        if (HAL_I2C_IsDeviceReady(_hi2c, ADDR8(i), 2, 20) == HAL_OK)
        {
            if (sensor_setup(ADDR8(i)) == HAL_OK)
            {
                _data[i].online = 1;
                online++;
            }
            continue;
        }

        /* Cold boot case: sensor at factory address? */
        if (HAL_I2C_IsDeviceReady(_hi2c, VL53L1X_DEFAULT_ADDR, 2, 20) != HAL_OK)
        {
            _data[i].online = 0;
            continue;   /* nobody home on this XSHUT line */
        }

        /* Move it to its final address */
        if (wr8(VL53L1X_DEFAULT_ADDR, REG_I2C_SLAVE_DEVICE_ADDRESS,
                (uint8_t)(TOF_ADDR_BASE + i)) != HAL_OK)
        {
            _data[i].online = 0;
            continue;
        }
        HAL_Delay(2);

        /* Full setup at new address */
        if (sensor_setup(ADDR8(i)) == HAL_OK)
        {
            _data[i].online = 1;
            online++;
        }
        else
        {
            _data[i].online = 0;
        }
    }

    return online;
}

void ToF_Process(void)
{
    for (int i = 0; i < TOF_COUNT; i++)
    {
        if (!_data[i].online) continue;

        uint8_t rdy = data_ready(ADDR8(i));
        if (rdy == 0xFF)
        {
            /* Sensor stopped responding — mark offline, keep robot running */
            _data[i].online = 0;
            HAL_UART_Transmit(&huart2, (uint8_t*)"TOF_DROP\r\n", 10, 10);
            continue;
        }
        if (rdy != 1) continue;   /* data ready when bit0 == 1 (active-high) */

        uint8_t  rs   = 0;
        uint16_t dist = 0;
        rd8 (ADDR8(i), REG_RESULT_RANGE_STATUS, &rs);
        rd16(ADDR8(i), REG_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &dist);

        rs &= 0x1F;
        /* ULD maps raw status: 9 = range valid */
        _data[i].status      = (rs == 9) ? 0 : rs;
        _data[i].distance_mm = dist;

        /* Clear interrupt to arm next sample */
        wr8(ADDR8(i), REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    }

    /* TEMP: raw debug for front sensor, 1Hz — remove after bring-up */
    static uint32_t dbg_t = 0;
    if (_data[TOF_FRONT].online && (HAL_GetTick() - dbg_t) >= 1000)
    {
        dbg_t = HAL_GetTick();
        uint8_t rdy = 0xEE, rs = 0xEE; uint16_t d = 0;
        rd8 (ADDR8(TOF_FRONT), REG_GPIO_TIO_HV_STATUS, &rdy);
        rd8 (ADDR8(TOF_FRONT), REG_RESULT_RANGE_STATUS, &rs);
        rd16(ADDR8(TOF_FRONT), REG_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &d);
        char b[48];
        int l = snprintf(b, sizeof(b), "TOF_DBG rdy=%02X rs=%02X d=%u\r\n", rdy, rs, d);
        HAL_UART_Transmit(&huart2, (uint8_t*)b, l, 20);
    }
}

ToF_Data_t* ToF_Get(uint8_t idx)
{
    return &_data[idx];
}

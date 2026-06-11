/**
 * @file    TCS34725.c
 * @brief   Non-blocking TCS34725 color sensor driver using I2C interrupt.
 */

#include "TCS34725.h"
#include "i2c.h"

/* ================= TCS34725 registers ================= */
#define TCS34725_ADDR              (0x29U << 1)
#define TCS34725_CMD_BIT           0x80U
#define TCS34725_CMD_AUTO_INC      0x20U

#define TCS34725_ENABLE            0x00U
#define TCS34725_ATIME             0x01U
#define TCS34725_CONTROL           0x0FU
#define TCS34725_ID                0x12U
#define TCS34725_CDATAL            0x14U

#define TCS34725_ENABLE_PON        0x01U
#define TCS34725_ENABLE_AEN        0x02U

/* ================= User tuning ================= */
#define TCS34725_READ_PERIOD_MS    130U
#define TCS34725_MIN_CLEAR         80U
#define TCS34725_I2C_TIMEOUT_MS    50U

/*
 * Integration time and gain:
 * ATIME = 0xD5  -> about 103 ms integration time
 * CONTROL = 0x01 -> 4x gain
 */
#define TCS34725_ATIME_VALUE       0xD5U
#define TCS34725_CONTROL_VALUE     0x01U

/* ================= Internal state ================= */
typedef enum
{
    TCS_STATE_RESET = 0,

    TCS_INIT_READ_ID_START,
    TCS_INIT_READ_ID_WAIT,

    TCS_INIT_WRITE_ATIME_START,
    TCS_INIT_WRITE_ATIME_WAIT,

    TCS_INIT_WRITE_CONTROL_START,
    TCS_INIT_WRITE_CONTROL_WAIT,

    TCS_INIT_WRITE_PON_START,
    TCS_INIT_WRITE_PON_WAIT,
    TCS_INIT_WAIT_PON,

    TCS_INIT_WRITE_AEN_START,
    TCS_INIT_WRITE_AEN_WAIT,
    TCS_INIT_WAIT_AEN,

    TCS_READY,

    TCS_READ_RAW_START,
    TCS_READ_RAW_WAIT,

    TCS_ERROR
} TCS34725_State_t;

static TCS34725_State_t tcs_state = TCS_STATE_RESET;

static volatile uint8_t i2c_busy  = 0U;
static volatile uint8_t i2c_done  = 0U;
static volatile uint8_t i2c_error = 0U;

static uint8_t i2c_tx_byte = 0U;
static uint8_t i2c_rx_buf[8];

static uint32_t i2c_start_tick = 0U;
static uint32_t state_tick = 0U;
static uint32_t last_read_tick = 0U;

static TCS34725_Raw_t last_raw;
static TCS34725_Color_t current_color = TCS34725_COLOR_NONE;

/* ================= Low-level I2C non-blocking helpers ================= */

static uint8_t TCS34725_StartWrite8_IT(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef status;

    if (i2c_busy != 0U)
    {
        return 0U;
    }

    i2c_tx_byte = value;
    i2c_done = 0U;
    i2c_error = 0U;
    i2c_busy = 1U;
    i2c_start_tick = HAL_GetTick();

    status = HAL_I2C_Mem_Write_IT(&hi2c1,
                                  TCS34725_ADDR,
                                  (uint16_t)(TCS34725_CMD_BIT | reg),
                                  I2C_MEMADD_SIZE_8BIT,
                                  &i2c_tx_byte,
                                  1U);

    if (status != HAL_OK)
    {
        i2c_busy = 0U;
        return 0U;
    }

    return 1U;
}

static uint8_t TCS34725_StartRead_IT(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef status;

    if (i2c_busy != 0U)
    {
        return 0U;
    }

    if ((buf == 0) || (len == 0U))
    {
        return 0U;
    }

    i2c_done = 0U;
    i2c_error = 0U;
    i2c_busy = 1U;
    i2c_start_tick = HAL_GetTick();

    status = HAL_I2C_Mem_Read_IT(&hi2c1,
                                 TCS34725_ADDR,
                                 (uint16_t)(TCS34725_CMD_BIT | reg),
                                 I2C_MEMADD_SIZE_8BIT,
                                 buf,
                                 len);

    if (status != HAL_OK)
    {
        i2c_busy = 0U;
        return 0U;
    }

    return 1U;
}

static uint8_t TCS34725_StartReadRaw_IT(void)
{
    uint8_t reg;

    reg = (uint8_t)(TCS34725_CMD_AUTO_INC | TCS34725_CDATAL);
    return TCS34725_StartRead_IT(reg, i2c_rx_buf, 8U);
}

static uint8_t TCS34725_CheckI2CFinished(void)
{
    uint32_t now;

    now = HAL_GetTick();

    if (i2c_error != 0U)
    {
        i2c_busy = 0U;
        i2c_done = 0U;
        i2c_error = 0U;
        tcs_state = TCS_ERROR;
        return 2U;
    }

    if (i2c_done != 0U)
    {
        i2c_done = 0U;
        return 1U;
    }

    if ((i2c_busy != 0U) && ((now - i2c_start_tick) > TCS34725_I2C_TIMEOUT_MS))
    {
        i2c_busy = 0U;
        i2c_done = 0U;
        i2c_error = 0U;
        tcs_state = TCS_ERROR;
        return 2U;
    }

    return 0U;
}

/* ================= Color classification ================= */

static TCS34725_Color_t TCS34725_ClassifyColor(const TCS34725_Raw_t *raw)
{
    uint32_t sum;
    uint32_t rn;
    uint32_t gn;
    uint32_t bn;

    if (raw == 0)
    {
        return TCS34725_COLOR_NONE;
    }

    if (raw->c < TCS34725_MIN_CLEAR)
    {
        return TCS34725_COLOR_NONE;
    }

    sum = (uint32_t)raw->r + (uint32_t)raw->g + (uint32_t)raw->b;
    if (sum == 0U)
    {
        return TCS34725_COLOR_NONE;
    }

    rn = ((uint32_t)raw->r * 1000U) / sum;
    gn = ((uint32_t)raw->g * 1000U) / sum;
    bn = ((uint32_t)raw->b * 1000U) / sum;

    /*
     * These thresholds are simple practical defaults.
     * You can tune them after watching raw C/R/G/B values in debug/UART.
     */
    if ((raw->c > 600U) &&
        (rn > 250U) && (rn < 400U) &&
        (gn > 250U) && (gn < 400U) &&
        (bn > 250U) && (bn < 400U))
    {
        return TCS34725_COLOR_WHITE;
    }

    if ((rn > 430U) && (rn > (gn + 80U)) && (rn > (bn + 80U)))
    {
        return TCS34725_COLOR_RED;
    }

    if ((gn > 400U) && (gn > (rn + 70U)) && (gn > (bn + 70U)))
    {
        return TCS34725_COLOR_GREEN;
    }

    if ((bn > 390U) && (bn > (rn + 70U)) && (bn > (gn + 70U)))
    {
        return TCS34725_COLOR_BLUE;
    }

    if ((rn > 330U) && (gn > 330U) && (bn < 280U))
    {
        return TCS34725_COLOR_YELLOW;
    }

    return TCS34725_COLOR_UNKNOWN;
}

static void TCS34725_ParseRawBuffer(void)
{
    last_raw.c = (uint16_t)(((uint16_t)i2c_rx_buf[1] << 8) | i2c_rx_buf[0]);
    last_raw.r = (uint16_t)(((uint16_t)i2c_rx_buf[3] << 8) | i2c_rx_buf[2]);
    last_raw.g = (uint16_t)(((uint16_t)i2c_rx_buf[5] << 8) | i2c_rx_buf[4]);
    last_raw.b = (uint16_t)(((uint16_t)i2c_rx_buf[7] << 8) | i2c_rx_buf[6]);
}

/* ================= Public API ================= */

void TCS34725_Init(void)
{
    i2c_busy = 0U;
    i2c_done = 0U;
    i2c_error = 0U;

    last_raw.c = 0U;
    last_raw.r = 0U;
    last_raw.g = 0U;
    last_raw.b = 0U;

    current_color = TCS34725_COLOR_NONE;

    state_tick = HAL_GetTick();
    last_read_tick = HAL_GetTick();

    tcs_state = TCS_INIT_READ_ID_START;
}

void TCS34725_Task(void)
{
    uint32_t now;
    uint8_t check;

    now = HAL_GetTick();

    switch (tcs_state)
    {
        case TCS_STATE_RESET:
        {
            TCS34725_Init();
            break;
        }

        case TCS_INIT_READ_ID_START:
        {
            if (TCS34725_StartRead_IT(TCS34725_ID, i2c_rx_buf, 1U) != 0U)
            {
                tcs_state = TCS_INIT_READ_ID_WAIT;
            }
            break;
        }

        case TCS_INIT_READ_ID_WAIT:
        {
            check = TCS34725_CheckI2CFinished();
            if (check == 1U)
            {
                if ((i2c_rx_buf[0] == 0x44U) || (i2c_rx_buf[0] == 0x4DU))
                {
                    tcs_state = TCS_INIT_WRITE_ATIME_START;
                }
                else
                {
                    tcs_state = TCS_ERROR;
                }
            }
            break;
        }

        case TCS_INIT_WRITE_ATIME_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_ATIME, TCS34725_ATIME_VALUE) != 0U)
            {
                tcs_state = TCS_INIT_WRITE_ATIME_WAIT;
            }
            break;
        }

        case TCS_INIT_WRITE_ATIME_WAIT:
        {
            check = TCS34725_CheckI2CFinished();
            if (check == 1U)
            {
                tcs_state = TCS_INIT_WRITE_CONTROL_START;
            }
            break;
        }

        case TCS_INIT_WRITE_CONTROL_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_CONTROL, TCS34725_CONTROL_VALUE) != 0U)
            {
                tcs_state = TCS_INIT_WRITE_CONTROL_WAIT;
            }
            break;
        }

        case TCS_INIT_WRITE_CONTROL_WAIT:
        {
            check = TCS34725_CheckI2CFinished();
            if (check == 1U)
            {
                tcs_state = TCS_INIT_WRITE_PON_START;
            }
            break;
        }

        case TCS_INIT_WRITE_PON_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_ENABLE, TCS34725_ENABLE_PON) != 0U)
            {
                tcs_state = TCS_INIT_WRITE_PON_WAIT;
            }
            break;
        }

        case TCS_INIT_WRITE_PON_WAIT:
        {
            check = TCS34725_CheckI2CFinished();
            if (check == 1U)
            {
                state_tick = now;
                tcs_state = TCS_INIT_WAIT_PON;
            }
            break;
        }

        case TCS_INIT_WAIT_PON:
        {
            if ((now - state_tick) >= 5U)
            {
                tcs_state = TCS_INIT_WRITE_AEN_START;
            }
            break;
        }

        case TCS_INIT_WRITE_AEN_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_ENABLE,
                                        (uint8_t)(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) != 0U)
            {
                tcs_state = TCS_INIT_WRITE_AEN_WAIT;
            }
            break;
        }

        case TCS_INIT_WRITE_AEN_WAIT:
        {
            check = TCS34725_CheckI2CFinished();
            if (check == 1U)
            {
                state_tick = now;
                tcs_state = TCS_INIT_WAIT_AEN;
            }
            break;
        }

        case TCS_INIT_WAIT_AEN:
        {
            if ((now - state_tick) >= 120U)
            {
                last_read_tick = now;
                tcs_state = TCS_READY;
            }
            break;
        }

        case TCS_READY:
        {
            if ((now - last_read_tick) >= TCS34725_READ_PERIOD_MS)
            {
                tcs_state = TCS_READ_RAW_START;
            }
            break;
        }

        case TCS_READ_RAW_START:
        {
            if (TCS34725_StartReadRaw_IT() != 0U)
            {
                tcs_state = TCS_READ_RAW_WAIT;
            }
            break;
        }

        case TCS_READ_RAW_WAIT:
        {
            check = TCS34725_CheckI2CFinished();
            if (check == 1U)
            {
                TCS34725_ParseRawBuffer();
                current_color = TCS34725_ClassifyColor(&last_raw);

                last_read_tick = now;
                tcs_state = TCS_READY;
            }
            break;
        }

        case TCS_ERROR:
        default:
        {
            /*
             * Stay here.
             * Call TCS34725_Init() again if you want to retry.
             */
            break;
        }
    }
}

TCS34725_Status_t TCS34725_GetStatus(void)
{
    if (tcs_state == TCS_ERROR)
    {
        return TCS34725_STATUS_ERROR;
    }

    if ((tcs_state == TCS_READY) ||
        (tcs_state == TCS_READ_RAW_START) ||
        (tcs_state == TCS_READ_RAW_WAIT))
    {
        return TCS34725_STATUS_READY;
    }

    return TCS34725_STATUS_INIT;
}

TCS34725_Color_t TCS34725_GetCurrentColor(void)
{
    return current_color;
}

TCS34725_Raw_t TCS34725_GetLastRaw(void)
{
    return last_raw;
}

uint8_t TCS34725_IsI2CBusy(void)
{
    return i2c_busy;
}

const char *TCS34725_ColorToString(TCS34725_Color_t color)
{
    switch (color)
    {
        case TCS34725_COLOR_RED:
            return "RED";

        case TCS34725_COLOR_GREEN:
            return "GREEN";

        case TCS34725_COLOR_BLUE:
            return "BLUE";

        case TCS34725_COLOR_YELLOW:
            return "YELLOW";

        case TCS34725_COLOR_WHITE:
            return "WHITE";

        case TCS34725_COLOR_UNKNOWN:
            return "UNKNOWN";

        case TCS34725_COLOR_NONE:
        default:
            return "NONE";
    }
}

/* ================= HAL callback routers ================= */

void TCS34725_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == 0)
    {
        return;
    }

    if (hi2c->Instance != hi2c1.Instance)
    {
        return;
    }

    if (i2c_busy == 0U)
    {
        return;
    }

    i2c_busy = 0U;
    i2c_done = 1U;
}

void TCS34725_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == 0)
    {
        return;
    }

    if (hi2c->Instance != hi2c1.Instance)
    {
        return;
    }

    if (i2c_busy == 0U)
    {
        return;
    }

    i2c_busy = 0U;
    i2c_done = 1U;
}

void TCS34725_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == 0)
    {
        return;
    }

    if (hi2c->Instance != hi2c1.Instance)
    {
        return;
    }

    i2c_busy = 0U;
    i2c_error = 1U;
}

void TCS34725_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == 0)
    {
        return;
    }

    if (hi2c->Instance != hi2c1.Instance)
    {
        return;
    }

    i2c_busy = 0U;
    i2c_error = 1U;
}

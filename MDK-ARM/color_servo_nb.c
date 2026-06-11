/**
 * @file    color_servo_nb.c
 * @brief   Non-blocking TCS34725 color sensor + servo control using I2C interrupt
 */

#include "color_servo_nb.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

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
#define COLOR_READ_PERIOD_MS       50U
#define COLOR_STABLE_TIME_MS       2000U
#define COLOR_RELEASE_TIME_MS      500U
#define TCS_MIN_CLEAR              80U

#define TCS_I2C_TIMEOUT_MS         50U

/* Servo setup: keep same logic as your old library */
#define SERVO_CENTER_US            1500U
#define SERVO_60_OFFSET_US         500U
#define SERVO_TIMER_TICK_US        10U
#define SERVO_TIMER_CHANNEL        TIM_CHANNEL_1

/* ================= Internal state ================= */
typedef enum
{
    CS_STATE_RESET = 0,

    CS_INIT_READ_ID_START,
    CS_INIT_READ_ID_WAIT,

    CS_INIT_WRITE_ATIME_START,
    CS_INIT_WRITE_ATIME_WAIT,

    CS_INIT_WRITE_CONTROL_START,
    CS_INIT_WRITE_CONTROL_WAIT,

    CS_INIT_WRITE_PON_START,
    CS_INIT_WRITE_PON_WAIT,
    CS_INIT_WAIT_PON,

    CS_INIT_WRITE_AEN_START,
    CS_INIT_WRITE_AEN_WAIT,
    CS_INIT_WAIT_AEN,

    CS_READY,

    CS_READ_RAW_START,
    CS_READ_RAW_WAIT,

    CS_ERROR
} ColorServo_State_t;

static ColorServo_State_t cs_state = CS_STATE_RESET;

static volatile uint8_t i2c_busy  = 0U;
static volatile uint8_t i2c_done  = 0U;
static volatile uint8_t i2c_error = 0U;

static uint8_t i2c_tx_byte = 0U;
static uint8_t i2c_rx_buf[8];
static uint32_t i2c_start_tick = 0U;

static uint32_t state_tick = 0U;
static uint32_t last_read_tick = 0U;

static TCS34725_Raw_t last_raw;
static Color_t current_color = COLOR_NONE;
static Color_t last_confirmed_color = COLOR_NONE;
static Color_t candidate_color = COLOR_NONE;

static uint32_t candidate_start_ms = 0U;
static uint32_t release_start_ms = 0U;
static uint8_t wait_release = 0U;
static uint8_t servo_is_at_60 = 0U;

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

static uint8_t ColorServo_CheckI2CFinished(void)
{
    uint32_t now;

    now = HAL_GetTick();

    if (i2c_error != 0U)
    {
        i2c_busy = 0U;
        i2c_done = 0U;
        i2c_error = 0U;
        cs_state = CS_ERROR;
        return 2U;
    }

    if (i2c_done != 0U)
    {
        i2c_done = 0U;
        return 1U;
    }

    if ((i2c_busy != 0U) && ((now - i2c_start_tick) > TCS_I2C_TIMEOUT_MS))
    {
        i2c_busy = 0U;
        i2c_done = 0U;
        i2c_error = 0U;
        cs_state = CS_ERROR;
        return 2U;
    }

    return 0U;
}

/* ================= Color classification ================= */
static Color_t TCS34725_ClassifyColor(TCS34725_Raw_t *raw)
{
    uint32_t sum;
    uint32_t rn;
    uint32_t gn;
    uint32_t bn;

    if (raw == 0)
    {
        return COLOR_NONE;
    }

    if (raw->c < TCS_MIN_CLEAR)
    {
        return COLOR_NONE;
    }

    sum = (uint32_t)raw->r + (uint32_t)raw->g + (uint32_t)raw->b;
    if (sum == 0U)
    {
        return COLOR_NONE;
    }

    rn = ((uint32_t)raw->r * 1000U) / sum;
    gn = ((uint32_t)raw->g * 1000U) / sum;
    bn = ((uint32_t)raw->b * 1000U) / sum;

    if ((raw->c > 600U) &&
        (rn > 250U) && (rn < 400U) &&
        (gn > 250U) && (gn < 400U) &&
        (bn > 250U) && (bn < 400U))
    {
        return COLOR_WHITE;
    }

    if ((rn > 430U) && (rn > (gn + 80U)) && (rn > (bn + 80U)))
    {
        return COLOR_RED;
    }

    if ((gn > 400U) && (gn > (rn + 70U)) && (gn > (bn + 70U)))
    {
        return COLOR_GREEN;
    }

    if ((bn > 390U) && (bn > (rn + 70U)) && (bn > (gn + 70U)))
    {
        return COLOR_BLUE;
    }

    if ((rn > 330U) && (gn > 330U) && (bn < 280U))
    {
        return COLOR_YELLOW;
    }

    return COLOR_UNKNOWN;
}

/* ================= Servo ================= */
static void Servo_WriteUs(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim1, SERVO_TIMER_CHANNEL, pulse_us / SERVO_TIMER_TICK_US);
}

void Servo_SetRelativeDegree(int16_t degree)
{
    uint16_t pulse_us;

    if (degree > 60)
    {
        degree = 60;
    }

    if (degree < -60)
    {
        degree = -60;
    }

    pulse_us = (uint16_t)(SERVO_CENTER_US + (((int32_t)degree * SERVO_60_OFFSET_US) / 60));
    Servo_WriteUs(pulse_us);
}

/* ================= Stable color logic ================= */
static void Process_StableColor(Color_t color)
{
    if ((color == COLOR_NONE) || (color == COLOR_UNKNOWN))
    {
        return;
    }

    if (last_confirmed_color == COLOR_NONE)
    {
        last_confirmed_color = color;
        servo_is_at_60 = 1U;
        Servo_SetRelativeDegree(60);
    }
    else
    {
        if (color == last_confirmed_color)
        {
            servo_is_at_60 ^= 1U;
            Servo_SetRelativeDegree((servo_is_at_60 != 0U) ? 60 : 0);
        }
        else
        {
            last_confirmed_color = color;
            servo_is_at_60 = 1U;
            Servo_SetRelativeDegree(60);
        }
    }

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

static void ColorServo_ProcessColor(Color_t color)
{
    uint32_t now;

    now = HAL_GetTick();

    if (wait_release != 0U)
    {
        if ((color == COLOR_NONE) || (color == COLOR_UNKNOWN))
        {
            if (release_start_ms == 0U)
            {
                release_start_ms = now;
            }

            if ((now - release_start_ms) >= COLOR_RELEASE_TIME_MS)
            {
                wait_release = 0U;
                release_start_ms = 0U;
                candidate_color = COLOR_NONE;
                candidate_start_ms = 0U;
            }
        }
        else
        {
            release_start_ms = 0U;
        }

        return;
    }

    if ((color == COLOR_NONE) || (color == COLOR_UNKNOWN))
    {
        candidate_color = COLOR_NONE;
        candidate_start_ms = 0U;
        return;
    }

    if (color != candidate_color)
    {
        candidate_color = color;
        candidate_start_ms = now;
        return;
    }

    if ((now - candidate_start_ms) >= COLOR_STABLE_TIME_MS)
    {
        Process_StableColor(color);
        wait_release = 1U;
        release_start_ms = 0U;
    }
}

static void TCS34725_ParseRawBuffer(void)
{
    last_raw.c = (uint16_t)(((uint16_t)i2c_rx_buf[1] << 8) | i2c_rx_buf[0]);
    last_raw.r = (uint16_t)(((uint16_t)i2c_rx_buf[3] << 8) | i2c_rx_buf[2]);
    last_raw.g = (uint16_t)(((uint16_t)i2c_rx_buf[5] << 8) | i2c_rx_buf[4]);
    last_raw.b = (uint16_t)(((uint16_t)i2c_rx_buf[7] << 8) | i2c_rx_buf[6]);
}

/* ================= Public API ================= */
void ColorServo_Init(void)
{
    i2c_busy = 0U;
    i2c_done = 0U;
    i2c_error = 0U;

    last_raw.c = 0U;
    last_raw.r = 0U;
    last_raw.g = 0U;
    last_raw.b = 0U;

    current_color = COLOR_NONE;
    last_confirmed_color = COLOR_NONE;
    candidate_color = COLOR_NONE;

    candidate_start_ms = 0U;
    release_start_ms = 0U;
    wait_release = 0U;
    servo_is_at_60 = 0U;

    state_tick = HAL_GetTick();
    last_read_tick = HAL_GetTick();

    HAL_TIM_PWM_Start(&htim1, SERVO_TIMER_CHANNEL);
    Servo_SetRelativeDegree(0);

    cs_state = CS_INIT_READ_ID_START;
}

void ColorServo_Task(void)
{
    uint32_t now;
    uint8_t check;

    now = HAL_GetTick();

    switch (cs_state)
    {
        case CS_STATE_RESET:
        {
            ColorServo_Init();
            break;
        }

        case CS_INIT_READ_ID_START:
        {
            if (TCS34725_StartRead_IT(TCS34725_ID, i2c_rx_buf, 1U) != 0U)
            {
                cs_state = CS_INIT_READ_ID_WAIT;
            }
            break;
        }

        case CS_INIT_READ_ID_WAIT:
        {
            check = ColorServo_CheckI2CFinished();
            if (check == 1U)
            {
                if ((i2c_rx_buf[0] == 0x44U) || (i2c_rx_buf[0] == 0x4DU))
                {
                    cs_state = CS_INIT_WRITE_ATIME_START;
                }
                else
                {
                    cs_state = CS_ERROR;
                }
            }
            break;
        }

        case CS_INIT_WRITE_ATIME_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_ATIME, 0xEBU) != 0U)
            {
                cs_state = CS_INIT_WRITE_ATIME_WAIT;
            }
            break;
        }

        case CS_INIT_WRITE_ATIME_WAIT:
        {
            check = ColorServo_CheckI2CFinished();
            if (check == 1U)
            {
                cs_state = CS_INIT_WRITE_CONTROL_START;
            }
            break;
        }

        case CS_INIT_WRITE_CONTROL_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_CONTROL, 0x01U) != 0U)
            {
                cs_state = CS_INIT_WRITE_CONTROL_WAIT;
            }
            break;
        }

        case CS_INIT_WRITE_CONTROL_WAIT:
        {
            check = ColorServo_CheckI2CFinished();
            if (check == 1U)
            {
                cs_state = CS_INIT_WRITE_PON_START;
            }
            break;
        }

        case CS_INIT_WRITE_PON_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_ENABLE, TCS34725_ENABLE_PON) != 0U)
            {
                cs_state = CS_INIT_WRITE_PON_WAIT;
            }
            break;
        }

        case CS_INIT_WRITE_PON_WAIT:
        {
            check = ColorServo_CheckI2CFinished();
            if (check == 1U)
            {
                state_tick = now;
                cs_state = CS_INIT_WAIT_PON;
            }
            break;
        }

        case CS_INIT_WAIT_PON:
        {
            if ((now - state_tick) >= 5U)
            {
                cs_state = CS_INIT_WRITE_AEN_START;
            }
            break;
        }

        case CS_INIT_WRITE_AEN_START:
        {
            if (TCS34725_StartWrite8_IT(TCS34725_ENABLE,
                                        (uint8_t)(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) != 0U)
            {
                cs_state = CS_INIT_WRITE_AEN_WAIT;
            }
            break;
        }

        case CS_INIT_WRITE_AEN_WAIT:
        {
            check = ColorServo_CheckI2CFinished();
            if (check == 1U)
            {
                state_tick = now;
                cs_state = CS_INIT_WAIT_AEN;
            }
            break;
        }

        case CS_INIT_WAIT_AEN:
        {
            if ((now - state_tick) >= 60U)
            {
                last_read_tick = now;
                cs_state = CS_READY;
            }
            break;
        }

        case CS_READY:
        {
            if ((now - last_read_tick) >= COLOR_READ_PERIOD_MS)
            {
                cs_state = CS_READ_RAW_START;
            }
            break;
        }

        case CS_READ_RAW_START:
        {
            if (TCS34725_StartReadRaw_IT() != 0U)
            {
                cs_state = CS_READ_RAW_WAIT;
            }
            break;
        }

        case CS_READ_RAW_WAIT:
        {
            check = ColorServo_CheckI2CFinished();
            if (check == 1U)
            {
                TCS34725_ParseRawBuffer();
                current_color = TCS34725_ClassifyColor(&last_raw);
                ColorServo_ProcessColor(current_color);

                last_read_tick = now;
                cs_state = CS_READY;
            }
            break;
        }

        case CS_ERROR:
        default:
        {
            /* Stay here. Call ColorServo_Init() again if you want to retry. */
            break;
        }
    }
}

void App_ColorServoLoop(void)
{
    ColorServo_Task();
}

ColorServo_Status_t ColorServo_GetStatus(void)
{
    if (cs_state == CS_ERROR)
    {
        return COLOR_SERVO_STATUS_ERROR;
    }

    if ((cs_state == CS_READY) ||
        (cs_state == CS_READ_RAW_START) ||
        (cs_state == CS_READ_RAW_WAIT))
    {
        return COLOR_SERVO_STATUS_READY;
    }

    return COLOR_SERVO_STATUS_INIT;
}

Color_t ColorServo_GetCurrentColor(void)
{
    return current_color;
}

Color_t ColorServo_GetLastConfirmedColor(void)
{
    return last_confirmed_color;
}

uint8_t ColorServo_IsI2CBusy(void)
{
    return i2c_busy;
}

TCS34725_Raw_t ColorServo_GetLastRaw(void)
{
    return last_raw;
}

/* ================= HAL callback routers ================= */
void ColorServo_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
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

void ColorServo_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
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

void ColorServo_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
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

void ColorServo_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
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

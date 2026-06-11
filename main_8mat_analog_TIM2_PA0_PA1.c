/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 + L298N + 8 analog line sensors
  ******************************************************************************
  * MOTOR - giu PWM nhu code cu:
  *   Right motor ENA -> PA0 / TIM2_CH1
  *   Left  motor ENB -> PA1 / TIM2_CH2
  *
  * MOTOR DIRECTION - doi sang PB10..PB13 de PA2..PA7 lam ADC:
  *   IN1 right -> PB10 / IN_1_R
  *   IN2 right -> PB11 / IN_2_R
  *   IN3 left  -> PB12 / IN_1_L
  *   IN4 left  -> PB13 / IN_2_L
  *
  * LINE SENSOR ANALOG 8 MAT:
  *   S1 -> PA2 / ADC1_IN2
  *   S2 -> PA3 / ADC1_IN3
  *   S3 -> PA4 / ADC1_IN4
  *   S4 -> PA5 / ADC1_IN5
  *   S5 -> PA6 / ADC1_IN6
  *   S6 -> PA7 / ADC1_IN7
  *   S7 -> PB0 / ADC1_IN8
  *   S8 -> PB1 / ADC1_IN9
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN PV */

/* ===================== MOTOR CONFIG ===================== */
#define PWM_MAX_PERCENT              100U

#define BASE_SPEED                   ((int16_t)55)
#define MIN_RUN_SPEED                ((int16_t)30)
#define MAX_RUN_SPEED                ((int16_t)85)

/*
 * Analog position tinh tu -3500 den +3500,
 * nen KP nho hon code digital 5 mat cu.
 */
#define LINE_KP                      ((int16_t)10)

/* Neu robot danh lai nguoc, doi 1 thanh -1. */
#define STEERING_SIGN                ((int16_t)1)

/* Neu motor tien/lui bi nguoc, doi 0 thanh 1. */
#define RIGHT_MOTOR_INVERT           0
#define LEFT_MOTOR_INVERT            0

/* ===================== ANALOG LINE SENSOR CONFIG ===================== */
#define LINE_SENSOR_COUNT            8U

/*
 * Neu dat line den ma ADC giam thap hon nen trang -> de 1.
 * Neu dat line den ma ADC tang cao hon nen trang -> doi thanh 0.
 */
#define LINE_BLACK_LOW_ADC           1

/*
 * Tong tin hieu nho hon nguong nay thi xem nhu mat line.
 * Hay dung nham: giam 220 xuong 120..180.
 * Chay khi khong co line: tang 220 len 300..500.
 */
#define LINE_SIGNAL_MIN_SUM          220U

/* Thoi gian tu canh chinh luc moi bat nguon. */
#define LINE_CALIBRATION_TIME_MS     2500U

/* USER CODE END PV */

void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static uint32_t PWM_PercentToCCR(TIM_HandleTypeDef *htim, uint8_t percent);
static void PWM_Start_All(void);

static void Motor_Right_SetPercent(int16_t percent);
static void Motor_Left_SetPercent(int16_t percent);
static void Motor_Drive(int16_t left_percent, int16_t right_percent);
static void Motor_AllStop(void);

static uint16_t ADC_Read_Channel(uint32_t channel);
static void Line_Calibrate(void);
static uint8_t Line_GetPosition(int16_t *position_out);
static void Line_Follow_Task(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static uint32_t PWM_PercentToCCR(TIM_HandleTypeDef *htim, uint8_t percent)
{
    if (percent > PWM_MAX_PERCENT)
    {
        percent = PWM_MAX_PERCENT;
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    return ((uint32_t)percent * (arr + 1U)) / 100U;
}

static void PWM_Start_All(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); /* PA0 -> ENA / right motor */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); /* PA1 -> ENB / left motor  */

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
}

static void Motor_Right_SetPercent(int16_t percent)
{
#if RIGHT_MOTOR_INVERT
    percent = -percent;
#endif

    if (percent > 100)  percent = 100;
    if (percent < -100) percent = -100;

    if (percent == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
        HAL_GPIO_WritePin(IN_1_R_GPIO_Port, IN_1_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN_2_R_GPIO_Port, IN_2_R_Pin, GPIO_PIN_RESET);
        return;
    }

    if (percent > 0)
    {
        HAL_GPIO_WritePin(IN_1_R_GPIO_Port, IN_1_R_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN_2_R_GPIO_Port, IN_2_R_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                              PWM_PercentToCCR(&htim2, (uint8_t)percent));
    }
    else
    {
        HAL_GPIO_WritePin(IN_1_R_GPIO_Port, IN_1_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN_2_R_GPIO_Port, IN_2_R_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,
                              PWM_PercentToCCR(&htim2, (uint8_t)(-percent)));
    }
}

static void Motor_Left_SetPercent(int16_t percent)
{
#if LEFT_MOTOR_INVERT
    percent = -percent;
#endif

    if (percent > 100)  percent = 100;
    if (percent < -100) percent = -100;

    if (percent == 0)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
        HAL_GPIO_WritePin(IN_1_L_GPIO_Port, IN_1_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN_2_L_GPIO_Port, IN_2_L_Pin, GPIO_PIN_RESET);
        return;
    }

    if (percent > 0)
    {
        HAL_GPIO_WritePin(IN_1_L_GPIO_Port, IN_1_L_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN_2_L_GPIO_Port, IN_2_L_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                              PWM_PercentToCCR(&htim2, (uint8_t)percent));
    }
    else
    {
        HAL_GPIO_WritePin(IN_1_L_GPIO_Port, IN_1_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN_2_L_GPIO_Port, IN_2_L_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,
                              PWM_PercentToCCR(&htim2, (uint8_t)(-percent)));
    }
}

static void Motor_Drive(int16_t left_percent, int16_t right_percent)
{
    if (left_percent > MAX_RUN_SPEED) left_percent = MAX_RUN_SPEED;
    if (left_percent < -MAX_RUN_SPEED) left_percent = -MAX_RUN_SPEED;

    if (right_percent > MAX_RUN_SPEED) right_percent = MAX_RUN_SPEED;
    if (right_percent < -MAX_RUN_SPEED) right_percent = -MAX_RUN_SPEED;

    if ((left_percent > 0) && (left_percent < MIN_RUN_SPEED)) left_percent = MIN_RUN_SPEED;
    if ((right_percent > 0) && (right_percent < MIN_RUN_SPEED)) right_percent = MIN_RUN_SPEED;

    Motor_Left_SetPercent(left_percent);
    Motor_Right_SetPercent(right_percent);
}

static void Motor_AllStop(void)
{
    Motor_Right_SetPercent(0);
    Motor_Left_SetPercent(0);
}

/* ===================== 8 ANALOG LINE SENSOR CODE ===================== */

static const uint32_t line_adc_channel[LINE_SENSOR_COUNT] =
{
    ADC_CHANNEL_2,  /* S1 -> PA2 */
    ADC_CHANNEL_3,  /* S2 -> PA3 */
    ADC_CHANNEL_4,  /* S3 -> PA4 */
    ADC_CHANNEL_5,  /* S4 -> PA5 */
    ADC_CHANNEL_6,  /* S5 -> PA6 */
    ADC_CHANNEL_7,  /* S6 -> PA7 */
    ADC_CHANNEL_8,  /* S7 -> PB0 */
    ADC_CHANNEL_9   /* S8 -> PB1 */
};

static uint16_t line_min[LINE_SENSOR_COUNT] =
{
    4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095
};

static uint16_t line_max[LINE_SENSOR_COUNT] =
{
    0, 0, 0, 0, 0, 0, 0, 0
};

static uint16_t ADC_Read_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_ADC_PollForConversion(&hadc1, 10U) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0U;
    }

    uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return value;
}

static void Line_Calibrate(void)
{
    uint32_t start_tick = HAL_GetTick();

    while ((HAL_GetTick() - start_tick) < LINE_CALIBRATION_TIME_MS)
    {
        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
        {
            uint16_t raw = ADC_Read_Channel(line_adc_channel[i]);

            if (raw < line_min[i]) line_min[i] = raw;
            if (raw > line_max[i]) line_max[i] = raw;
        }

        HAL_Delay(3U);
    }

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        if ((line_max[i] - line_min[i]) < 80U)
        {
            line_max[i] = (uint16_t)(line_min[i] + 80U);

            if (line_max[i] > 4095U)
            {
                line_max[i] = 4095U;
            }
        }
    }
}

/*
 * return 1: co line, return 0: mat line.
 * position_out: -3500 trai ngoai cung, 0 giua, +3500 phai ngoai cung.
 */
static uint8_t Line_GetPosition(int16_t *position_out)
{
    static const int16_t weight[LINE_SENSOR_COUNT] =
    {
        -3500, -2500, -1500, -500, 500, 1500, 2500, 3500
    };

    int32_t weighted_sum = 0;
    uint32_t signal_sum = 0;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        uint16_t raw = ADC_Read_Channel(line_adc_channel[i]);

        if (raw < line_min[i]) raw = line_min[i];
        if (raw > line_max[i]) raw = line_max[i];

        uint16_t range = (uint16_t)(line_max[i] - line_min[i]);
        if (range < 1U) range = 1U;

        uint32_t signal;

#if LINE_BLACK_LOW_ADC
        signal = ((uint32_t)(line_max[i] - raw) * 1000U) / range;
#else
        signal = ((uint32_t)(raw - line_min[i]) * 1000U) / range;
#endif

        weighted_sum += ((int32_t)weight[i] * (int32_t)signal);
        signal_sum += signal;
    }

    if (signal_sum < LINE_SIGNAL_MIN_SUM)
    {
        *position_out = 0;
        return 0U;
    }

    *position_out = (int16_t)(weighted_sum / (int32_t)signal_sum);
    return 1U;
}

static void Line_Follow_Task(void)
{
    int16_t position = 0;
    uint8_t found = Line_GetPosition(&position);

    if (!found)
    {
        Motor_AllStop();
        return;
    }

    int16_t correction = (int16_t)((STEERING_SIGN * LINE_KP * position) / 1000);

    int16_t left_speed  = (int16_t)(BASE_SPEED + correction);
    int16_t right_speed = (int16_t)(BASE_SPEED - correction);

    Motor_Drive(left_speed, right_speed);
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_TIM2_Init();

    PWM_Start_All();
    Motor_AllStop();

    HAL_ADCEx_Calibration_Start(&hadc1);

    HAL_Delay(500U);

    /* Quet cam bien qua nen trang va line den trong 2.5 giay. */
    Line_Calibrate();

    while (1)
    {
        Line_Follow_Task();
        HAL_Delay(5U);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

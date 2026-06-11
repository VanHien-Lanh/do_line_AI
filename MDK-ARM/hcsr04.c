#include "hcsr04.h"
#include "stm32f1xx_hal.h"

/**
 * @brief Initialize HC-SR04 sensor
 * @param hcsr Pointer to sensor handle
 */
void HCSR04_Init(HCSR04_HandleTypeDef *hcsr)
{
    HAL_GPIO_WritePin(hcsr->TRIG_Port, hcsr->TRIG_Pin, GPIO_PIN_RESET);
    HAL_Delay(2); // Wait sensor to settle
    hcsr->pulse = 0;
    hcsr->ready = 0;
    hcsr->start_tick = 0;
}

/**
 * @brief Send 10us trigger pulse (blocking)
 * @param hcsr Pointer to sensor handle
 */
static void HCSR04_DelayUs(TIM_HandleTypeDef *htim, uint16_t us)
{
    uint16_t start;

    if ((htim == 0) || (htim->Instance == 0))
    {
        return;
    }

    start = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
    while ((uint16_t)((uint16_t)__HAL_TIM_GET_COUNTER(htim) - start) < us)
    {
        /* wait */
    }
}

void HCSR04_Trigger(HCSR04_HandleTypeDef *hcsr)
{
    if ((hcsr == 0) || (hcsr->htim_us == 0))
    {
        return;
    }

    /* Reset ve suon len truoc moi lan do. */
    __HAL_TIM_SET_CAPTUREPOLARITY(hcsr->htim_us, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    hcsr->ready = 0U;

    HAL_GPIO_WritePin(hcsr->TRIG_Port, hcsr->TRIG_Pin, GPIO_PIN_RESET);
    HCSR04_DelayUs(hcsr->htim_us, 2U);

    HAL_GPIO_WritePin(hcsr->TRIG_Port, hcsr->TRIG_Pin, GPIO_PIN_SET);
    HCSR04_DelayUs(hcsr->htim_us, 12U); /* HC-SR04 can xung trigger toi thieu 10us. */
    HAL_GPIO_WritePin(hcsr->TRIG_Port, hcsr->TRIG_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Input capture callback, called from HAL_TIM_IC_CaptureCallback
 * @param hcsr Pointer to sensor handle
 * @param htim Timer handle from HAL callback
 */
void HCSR04_TIM_IC_Callback(HCSR04_HandleTypeDef *hcsr, TIM_HandleTypeDef *htim)
{
    if(htim->Instance != hcsr->htim_us->Instance) return;

    if(HAL_GPIO_ReadPin(hcsr->ECHO_Port, hcsr->ECHO_Pin) == GPIO_PIN_SET)
    {
        // 1. Ph?t hi?n su?n l?n (B?t d?u xung ECHO)
        hcsr->start_tick = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        
        // QUAN TR?NG: ??i c?c Input Capture sang su?n xu?ng (Falling Edge)
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
        // 2. Ph?t hi?n su?n xu?ng (K?t th?c xung ECHO)
        uint32_t end_tick = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if(end_tick >= hcsr->start_tick)
            hcsr->pulse = end_tick - hcsr->start_tick;
        else
            hcsr->pulse = (0xFFFF - hcsr->start_tick) + end_tick;

        hcsr->ready = 1; // B?t c? b?o hi?u d? do xong
        
        // QUAN TR?NG: ??i c?c ng?t tr? l?i su?n l?n (Rising Edge) d? chu?n b? cho l?n ph?t sau
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

/**
 * @brief Read distance in cm (non-blocking)
 * @param hcsr Pointer to sensor handle
 * @return Distance in cm, 0 if no new measurement
 */
uint32_t HCSR04_ReadDistanceCm(HCSR04_HandleTypeDef *hcsr)
{
    if(hcsr->ready)
    {
        hcsr->ready = 0;
        return hcsr->pulse / 58;
    }
    return 0; // No new measurement
}

/**
 * @brief Read distance in mm (non-blocking)
 * @param hcsr Pointer to sensor handle
 * @return Distance in mm, 0 if no new measurement
 */
uint32_t HCSR04_ReadDistanceMm(HCSR04_HandleTypeDef *hcsr)
{
    if(hcsr->ready)
    {
        hcsr->ready = 0;
        return (uint32_t)(hcsr->pulse / 5.8);
    }
    return 0; // No new measurement
}

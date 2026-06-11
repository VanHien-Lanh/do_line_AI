#ifndef HCSR04_H
#define HCSR04_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief HC-SR04 sensor handle structure
 */
typedef struct
{
    GPIO_TypeDef *TRIG_Port;       // TRIG pin port
    uint16_t      TRIG_Pin;        // TRIG pin
    GPIO_TypeDef *ECHO_Port;       // ECHO pin port
    uint16_t      ECHO_Pin;        // ECHO pin
    TIM_HandleTypeDef *htim_us;    // Timer handle for microsecond capture
    volatile uint32_t pulse;       // Last measured pulse width (us)
    volatile uint8_t  ready;       // Flag: 1 = new measurement ready
    volatile uint32_t start_tick;  // Rising edge tick
} HCSR04_HandleTypeDef;

/* Initialize HC-SR04 sensor */
void HCSR04_Init(HCSR04_HandleTypeDef *hcsr);

/* Send 10us trigger pulse (blocking) */
void HCSR04_Trigger(HCSR04_HandleTypeDef *hcsr);

/* Get latest distance in cm (non-blocking) */
uint32_t HCSR04_ReadDistanceCm(HCSR04_HandleTypeDef *hcsr);

/* Get latest distance in mm (non-blocking) */
uint32_t HCSR04_ReadDistanceMm(HCSR04_HandleTypeDef *hcsr);

/* TIM input capture callback, must be called from HAL_TIM_IC_CaptureCallback */
void HCSR04_TIM_IC_Callback(HCSR04_HandleTypeDef *hcsr, TIM_HandleTypeDef *htim);

#endif /* HCSR04_H */



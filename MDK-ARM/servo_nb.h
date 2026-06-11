#ifndef SERVO_NB_H
#define SERVO_NB_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    TIM_HandleTypeDef *htim;
    uint32_t channel;

    uint16_t min_us;        // Usually 500us
    uint16_t max_us;        // Usually 2500us
    uint16_t center_us;     // Usually 1500us

    uint16_t timer_tick_us; // Timer tick unit, usually 1us or 10us

    uint16_t current_us;
    uint16_t target_us;

    uint16_t step_us;       // Pulse change per update
    uint32_t update_ms;     // Update period

    uint32_t last_update_tick;

    uint8_t is_started;
    uint8_t is_moving;

} ServoNB_HandleTypeDef;

void ServoNB_Init(ServoNB_HandleTypeDef *hservo,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel,
                  uint16_t min_us,
                  uint16_t max_us,
                  uint16_t center_us,
                  uint16_t timer_tick_us);

void ServoNB_Start(ServoNB_HandleTypeDef *hservo);
void ServoNB_Stop(ServoNB_HandleTypeDef *hservo);

/* Set immediately */
void ServoNB_WriteUs(ServoNB_HandleTypeDef *hservo, uint16_t pulse_us);
void ServoNB_WriteAngle(ServoNB_HandleTypeDef *hservo, uint16_t angle);

/* Move gradually, non-blocking */
void ServoNB_MoveToUs(ServoNB_HandleTypeDef *hservo,
                      uint16_t target_us,
                      uint16_t step_us,
                      uint32_t update_ms);

void ServoNB_MoveToAngle(ServoNB_HandleTypeDef *hservo,
                         uint16_t angle,
                         uint16_t step_us,
                         uint32_t update_ms);

/* Relative angle around center: -90 to +90 */
void ServoNB_WriteRelativeDegree(ServoNB_HandleTypeDef *hservo, int16_t degree);

void ServoNB_MoveToRelativeDegree(ServoNB_HandleTypeDef *hservo,
                                  int16_t degree,
                                  uint16_t step_us,
                                  uint32_t update_ms);

/* Must be called repeatedly inside while(1) */
void ServoNB_Task(ServoNB_HandleTypeDef *hservo);

uint8_t ServoNB_IsMoving(ServoNB_HandleTypeDef *hservo);
uint16_t ServoNB_GetCurrentUs(ServoNB_HandleTypeDef *hservo);

#endif /* SERVO_NB_H */


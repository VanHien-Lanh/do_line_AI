#include "servo_nb.h"

static uint16_t ServoNB_ClampUs(ServoNB_HandleTypeDef *hservo, uint16_t pulse_us)
{
    if (pulse_us < hservo->min_us)
    {
        pulse_us = hservo->min_us;
    }

    if (pulse_us > hservo->max_us)
    {
        pulse_us = hservo->max_us;
    }

    return pulse_us;
}

static uint16_t ServoNB_AngleToUs(ServoNB_HandleTypeDef *hservo, uint16_t angle)
{
    uint32_t pulse_range;
    uint32_t pulse_us;

    if (angle > 180U)
    {
        angle = 180U;
    }

    pulse_range = (uint32_t)(hservo->max_us - hservo->min_us);

    pulse_us = (uint32_t)hservo->min_us +
               ((pulse_range * angle) / 180U);

    return (uint16_t)pulse_us;
}

static uint16_t ServoNB_RelativeDegreeToUs(ServoNB_HandleTypeDef *hservo, int16_t degree)
{
    int32_t pulse_us;
    int32_t offset_range;

    if (degree > 90)
    {
        degree = 90;
    }

    if (degree < -90)
    {
        degree = -90;
    }

    if (degree >= 0)
    {
        offset_range = (int32_t)hservo->max_us - (int32_t)hservo->center_us;
    }
    else
    {
        offset_range = (int32_t)hservo->center_us - (int32_t)hservo->min_us;
    }

    pulse_us = (int32_t)hservo->center_us +
               ((offset_range * degree) / 90);

    return ServoNB_ClampUs(hservo, (uint16_t)pulse_us);
}

static void ServoNB_SetCompareUs(ServoNB_HandleTypeDef *hservo, uint16_t pulse_us)
{
    uint32_t compare;

    if (hservo == 0) return;
    if (hservo->htim == 0) return;
    if (hservo->timer_tick_us == 0U) return;

    pulse_us = ServoNB_ClampUs(hservo, pulse_us);

    compare = (uint32_t)pulse_us / (uint32_t)hservo->timer_tick_us;

    __HAL_TIM_SET_COMPARE(hservo->htim, hservo->channel, compare);
}

void ServoNB_Init(ServoNB_HandleTypeDef *hservo,
                  TIM_HandleTypeDef *htim,
                  uint32_t channel,
                  uint16_t min_us,
                  uint16_t max_us,
                  uint16_t center_us,
                  uint16_t timer_tick_us)
{
    if (hservo == 0) return;

    hservo->htim = htim;
    hservo->channel = channel;

    hservo->min_us = min_us;
    hservo->max_us = max_us;
    hservo->center_us = center_us;

    if (timer_tick_us == 0U)
    {
        timer_tick_us = 1U;
    }

    hservo->timer_tick_us = timer_tick_us;

    hservo->current_us = center_us;
    hservo->target_us = center_us;

    hservo->step_us = 10U;
    hservo->update_ms = 10U;

    hservo->last_update_tick = HAL_GetTick();

    hservo->is_started = 0U;
    hservo->is_moving = 0U;
}

void ServoNB_Start(ServoNB_HandleTypeDef *hservo)
{
    if (hservo == 0) return;
    if (hservo->htim == 0) return;

    HAL_TIM_PWM_Start(hservo->htim, hservo->channel);

    hservo->is_started = 1U;

    ServoNB_SetCompareUs(hservo, hservo->current_us);
}

void ServoNB_Stop(ServoNB_HandleTypeDef *hservo)
{
    if (hservo == 0) return;
    if (hservo->htim == 0) return;

    HAL_TIM_PWM_Stop(hservo->htim, hservo->channel);

    hservo->is_started = 0U;
    hservo->is_moving = 0U;
}

void ServoNB_WriteUs(ServoNB_HandleTypeDef *hservo, uint16_t pulse_us)
{
    if (hservo == 0) return;

    pulse_us = ServoNB_ClampUs(hservo, pulse_us);

    hservo->current_us = pulse_us;
    hservo->target_us = pulse_us;
    hservo->is_moving = 0U;

    if (hservo->is_started)
    {
        ServoNB_SetCompareUs(hservo, pulse_us);
    }
}

void ServoNB_WriteAngle(ServoNB_HandleTypeDef *hservo, uint16_t angle)
{
    uint16_t pulse_us;

    if (hservo == 0) return;

    pulse_us = ServoNB_AngleToUs(hservo, angle);

    ServoNB_WriteUs(hservo, pulse_us);
}

void ServoNB_MoveToUs(ServoNB_HandleTypeDef *hservo,
                      uint16_t target_us,
                      uint16_t step_us,
                      uint32_t update_ms)
{
    if (hservo == 0) return;

    target_us = ServoNB_ClampUs(hservo, target_us);

    if (step_us == 0U)
    {
        step_us = 1U;
    }

    if (update_ms == 0U)
    {
        update_ms = 1U;
    }

    hservo->target_us = target_us;
    hservo->step_us = step_us;
    hservo->update_ms = update_ms;
    hservo->last_update_tick = HAL_GetTick();

    if (hservo->current_us != hservo->target_us)
    {
        hservo->is_moving = 1U;
    }
    else
    {
        hservo->is_moving = 0U;
    }
}

void ServoNB_MoveToAngle(ServoNB_HandleTypeDef *hservo,
                         uint16_t angle,
                         uint16_t step_us,
                         uint32_t update_ms)
{
    uint16_t target_us;

    if (hservo == 0) return;

    target_us = ServoNB_AngleToUs(hservo, angle);

    ServoNB_MoveToUs(hservo, target_us, step_us, update_ms);
}

void ServoNB_WriteRelativeDegree(ServoNB_HandleTypeDef *hservo, int16_t degree)
{
    uint16_t pulse_us;

    if (hservo == 0) return;

    pulse_us = ServoNB_RelativeDegreeToUs(hservo, degree);

    ServoNB_WriteUs(hservo, pulse_us);
}

void ServoNB_MoveToRelativeDegree(ServoNB_HandleTypeDef *hservo,
                                  int16_t degree,
                                  uint16_t step_us,
                                  uint32_t update_ms)
{
    uint16_t target_us;

    if (hservo == 0) return;

    target_us = ServoNB_RelativeDegreeToUs(hservo, degree);

    ServoNB_MoveToUs(hservo, target_us, step_us, update_ms);
}

void ServoNB_Task(ServoNB_HandleTypeDef *hservo)
{
    uint32_t now;

    if (hservo == 0) return;
    if (hservo->is_started == 0U) return;
    if (hservo->is_moving == 0U) return;

    now = HAL_GetTick();

    if ((now - hservo->last_update_tick) < hservo->update_ms)
    {
        return;
    }

    hservo->last_update_tick = now;

    if (hservo->current_us < hservo->target_us)
    {
        if ((hservo->target_us - hservo->current_us) <= hservo->step_us)
        {
            hservo->current_us = hservo->target_us;
            hservo->is_moving = 0U;
        }
        else
        {
            hservo->current_us += hservo->step_us;
        }
    }
    else if (hservo->current_us > hservo->target_us)
    {
        if ((hservo->current_us - hservo->target_us) <= hservo->step_us)
        {
            hservo->current_us = hservo->target_us;
            hservo->is_moving = 0U;
        }
        else
        {
            hservo->current_us -= hservo->step_us;
        }
    }
    else
    {
        hservo->is_moving = 0U;
    }

    ServoNB_SetCompareUs(hservo, hservo->current_us);
}

uint8_t ServoNB_IsMoving(ServoNB_HandleTypeDef *hservo)
{
    if (hservo == 0) return 0U;

    return hservo->is_moving;
}

uint16_t ServoNB_GetCurrentUs(ServoNB_HandleTypeDef *hservo)
{
    if (hservo == 0) return 0U;

    return hservo->current_us;
}


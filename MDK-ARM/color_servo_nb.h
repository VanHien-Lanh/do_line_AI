#ifndef COLOR_SERVO_NB_H
#define COLOR_SERVO_NB_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= Color type ================= */
typedef enum
{
    COLOR_NONE = 0,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW,
    COLOR_WHITE,
    COLOR_UNKNOWN
} Color_t;

/* ================= Raw data type ================= */
typedef struct
{
    uint16_t c;
    uint16_t r;
    uint16_t g;
    uint16_t b;
} TCS34725_Raw_t;

/* ================= Driver status ================= */
typedef enum
{
    COLOR_SERVO_STATUS_INIT = 0,
    COLOR_SERVO_STATUS_READY,
    COLOR_SERVO_STATUS_ERROR
} ColorServo_Status_t;

/*
 * Start/reset non-blocking color-servo driver.
 * Call once after MX_I2C1_Init(), MX_TIM1_Init(), MX_GPIO_Init().
 */
void ColorServo_Init(void);

/*
 * Non-blocking task.
 * Call continuously in while(1).
 */
void ColorServo_Task(void);

/* Backward-compatible name if your old main calls App_ColorServoLoop(). */
void App_ColorServoLoop(void);

/* Servo control */
void Servo_SetRelativeDegree(int16_t degree);

/* Status/readback helpers */
ColorServo_Status_t ColorServo_GetStatus(void);
Color_t ColorServo_GetCurrentColor(void);
Color_t ColorServo_GetLastConfirmedColor(void);
uint8_t ColorServo_IsI2CBusy(void);
TCS34725_Raw_t ColorServo_GetLastRaw(void);

/*
 * Call these from HAL I2C callbacks in main.c.
 */
void ColorServo_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void ColorServo_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void ColorServo_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void ColorServo_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* COLOR_SERVO_NB_H */

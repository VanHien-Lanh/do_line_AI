#ifndef TCS34725_H
#define TCS34725_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================= Color type ================= */
typedef enum
{
    TCS34725_COLOR_NONE = 0,
    TCS34725_COLOR_RED,
    TCS34725_COLOR_GREEN,
    TCS34725_COLOR_BLUE,
    TCS34725_COLOR_YELLOW,
    TCS34725_COLOR_WHITE,
    TCS34725_COLOR_UNKNOWN
} TCS34725_Color_t;

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
    TCS34725_STATUS_INIT = 0,
    TCS34725_STATUS_READY,
    TCS34725_STATUS_ERROR
} TCS34725_Status_t;

/*
 * Non-blocking TCS34725 color sensor driver using I2C interrupt.
 *
 * CubeMX requirement:
 * - Enable I2C1
 * - Enable I2C1 EV interrupt
 * - Enable I2C1 ER interrupt
 *
 * Usage:
 * 1. Call TCS34725_Init() once after MX_I2C1_Init().
 * 2. Call TCS34725_Task() continuously in while(1).
 * 3. Route HAL I2C callbacks to the callback functions below.
 */

/* Start/reset driver */
void TCS34725_Init(void);

/* Non-blocking task. Call continuously in while(1). */
void TCS34725_Task(void);

/* Status/readback helpers */
TCS34725_Status_t TCS34725_GetStatus(void);
TCS34725_Color_t  TCS34725_GetCurrentColor(void);
TCS34725_Raw_t    TCS34725_GetLastRaw(void);
uint8_t           TCS34725_IsI2CBusy(void);

/* Optional helper for UART/debug */
const char *TCS34725_ColorToString(TCS34725_Color_t color);

/*
 * Call these from HAL I2C callbacks in main.c.
 */
void TCS34725_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void TCS34725_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void TCS34725_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void TCS34725_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif /* TCS34725_H */

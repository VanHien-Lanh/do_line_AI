/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103C8T6 + L298N + 5 TCRT5000 digital line sensors
  *                   + HC-SR04 + TCS34725 + 2 servos.
  *
  * FINAL INTEGRATED FLOW:
  *   1) Follow line.
  *   2) If obstacle <= 10 cm: slow down smoothly.
  *   3) If obstacle <= 3 cm: stop immediately.
  *   4) Read object color slowly/stably, remember it.
  *   5) Servo1 down 90 deg, Servo2 close 40 deg, Servo1 up 0 deg.
  *   6) Carry object and follow line.
  *   7) Next obstacle: slow at 10 cm, stop at 3 cm, read color.
  *   8) If color matches: drop object, then follow line to finish marker.
  *   9) If color is wrong: avoid to the right, recover line, continue searching.
  *
  * MOTOR:
  *   Right motor ENA -> PA0 / TIM2_CH1
  *   Left  motor ENB -> PA1 / TIM2_CH2
  *   IN1 right -> PB10
  *   IN2 right -> PB11
  *   IN3 left  -> PB12
  *   IN4 left  -> PB13
  *
  * 5 DIGITAL TCRT5000 LINE SENSOR BOARD:
  *   S1/OUT1 left-most  -> PA2
  *   S2/OUT2            -> PA3
  *   S3/OUT3 center     -> PA4
  *   S4/OUT4            -> PA5
  *   S5/OUT5 right-most -> PA6
  *
  * OTHER PERIPHERALS:
  *   HC-SR04 TRIG -> PB5, ECHO -> PB4 / TIM3_CH1
  *   TCS34725 I2C1 remap: SCL -> PB8, SDA -> PB9
  *   Servo1 -> PA9 / TIM1_CH2, Servo2 -> PA8 / TIM1_CH1
  *   USART1 remap: TX -> PB6, RX -> PB7, 115200 baud
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "../../MDK-ARM/servo_nb.h"
#include "../../MDK-ARM/hcsr04.h"
#include "../../MDK-ARM/TCS34725.h"
#include <stdio.h>

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void Line5_GPIO_Init(void);
static uint32_t PWM_PercentToCCR(TIM_HandleTypeDef *htim, uint8_t percent);
static int16_t Clamp16(int16_t value, int16_t min_value, int16_t max_value);
static int32_t MapLong(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);

static void PWM_Start_All(void);
static void Motor_Right_SetPercent(int16_t percent);
static void Motor_Left_SetPercent(int16_t percent);
static void Motor_RunPercent(int16_t left_percent, int16_t right_percent);
static void Motor_Run255(int16_t left_255, int16_t right_255);
static void Motor_AllStop(void);

static uint8_t Line5_ReadOne(GPIO_TypeDef *port, uint16_t pin);
static uint8_t Line5_ReadMask(void);
static int16_t Line5_MapSpeedByError(int16_t base_speed, int16_t error_abs);
static void Line5_SetSpeed(int16_t speed_255);
static void Line5_ResetMemory(void);
static void Line5_SearchLostLine(void);
static void Line5_Task(void);
static uint8_t Line5_FinishMarkerTask(void);

static void Servo_App_Init(void);
static void Servo_CommandAngle(ServoNB_HandleTypeDef *servo, uint16_t target_angle);
static uint8_t Servo_WaitDone(uint32_t timeout_ms);

static void HCSR04_App_Init(void);
static void HCSR04_App_Task(void);
static uint8_t Distance_IsFresh(uint32_t now);
static uint8_t Distance_UpdateSpeedAndCheckStop(uint32_t now, int16_t base_speed_255);

static void TCS34725_App_Init(void);
static void TCS34725_App_Task(void);
static uint8_t Color_IsValid(TCS34725_Color_t color);
static TCS34725_Color_t Color_ClassifyRawStable(const TCS34725_Raw_t *raw);
static void ColorStable_Reset(void);
static void ColorVote_Add(TCS34725_Color_t color);
static uint8_t ColorVote_GetWinner(TCS34725_Color_t *color_out);
static uint8_t ColorStable_Task(TCS34725_Color_t *stable_color_out);

static uint16_t UART1_AddString(uint16_t index, const char *s);
static uint16_t UART1_AddNumber(uint16_t index, uint32_t num);
static uint16_t UART1_MakeDebugString(void);
static void UART1_Debug_Task(void);

static void Robot_Main_Task(void);

/* ===================== MOTOR CONFIG ===================== */
#define PWM_MAX_PERCENT              100U
#define MOTOR_MIN_PERCENT            18U
#define RIGHT_MOTOR_INVERT           0
#define LEFT_MOTOR_INVERT            0

/* 0 = no UART spam, 1 = print debug every UART_DEBUG_PERIOD_MS. */
#define ROBOT_UART_DEBUG             0U
#define UART_DEBUG_PERIOD_MS         500U

/* ===================== 5CH DIGITAL LINE CONFIG ===================== */
#define LINE5_ACTIVE_LOW             1U
#define LINE5_TASK_PERIOD_MS         2U

/* Full robot speed is intentionally lower than the line-only test.
 * Priority: stable line following + enough time for HC-SR04 to slow at 10cm.
 */
#define LINE5_BASE_SPEED_255         ((int16_t)75)   /* Giam toc de bam line on dinh hon */
#define LINE5_CARRY_SPEED_255        ((int16_t)65)   /* Dang mang vat thi chay cham hon */
#define LINE5_MIN_CURVE_SPEED_255    ((int16_t)40)
#define LINE5_MAX_SPEED_255          ((int16_t)170)
#define LINE5_MAX_STEER_255          ((int16_t)150)
#define LINE5_KP_NUM                 60L
#define LINE5_KD_NUM                 35L
#define LINE5_PID_DIV                1000L

/* Lost-line recovery V6:
 * - Khong dung im khi mat line.
 * - Nho huong/vi tri line cuoi cung.
 * - Quay nhanh ve line vua mat truoc, sau do quet trai/phai voi toc do
 *   xap xi toc do bo cua binh thuong de dong co du luc keo.
 */
#define LINE5_LOST_BACKTRACK_SPEED_255   ((int16_t)-62)
#define LINE5_LOST_RETURN_TURN_255       ((int16_t)110)
#define LINE5_LOST_RETURN_REVERSE_255    ((int16_t)-70)
#define LINE5_LOST_SWEEP_TURN_255        ((int16_t)105)
#define LINE5_LOST_SWEEP_REVERSE_255     ((int16_t)-65)
#define LINE5_LOST_FORWARD_255           ((int16_t)58)
#define LINE5_LOST_BACKUP_255            ((int16_t)-55)
#define LINE5_LOST_BACKTRACK_COUNT       45U    /* 90ms: lui nhanh ve line vua mat */
#define LINE5_LOST_RETURN_COUNT          160U   /* 320ms: quay nhanh ve huong line vua mat */
#define LINE5_SEARCH_SWEEP_COUNT         230U   /* 460ms moi ben: quet manh trai/phai */
#define LINE5_SEARCH_FORWARD_COUNT       70U    /* 140ms tien cham de mo rong vung tim */
#define LINE5_SEARCH_BACKUP_COUNT        90U    /* 180ms lui nhe roi quet lai */
#define LINE5_SEARCH_CYCLE_COUNT         ((2U * LINE5_SEARCH_SWEEP_COUNT) + LINE5_SEARCH_FORWARD_COUNT + LINE5_SEARCH_BACKUP_COUNT)

/* Center-line lost recovery V7:
 * Neu xe mat line khi truoc do line o gan giua, xe se quay 360 do truoc,
 * sau do tien thang ngan, roi moi vao chu ky quet trai/phai nhu cu.
 * Luu y: LINE5_CENTER_SPIN_COUNT phu thuoc toc do motor, pin va mat san.
 */
#define LINE5_CENTER_LOST_ENABLE         1U
#define LINE5_CENTER_ERROR_WINDOW        350
#define LINE5_CENTER_SPIN_TURN_255       ((int16_t)115)
#define LINE5_CENTER_SPIN_REVERSE_255    ((int16_t)-75)
#define LINE5_CENTER_SPIN_COUNT          720U   /* 1.44s @ 2ms/task: tune de gan 360 do */
#define LINE5_CENTER_FORWARD_255         ((int16_t)75)
#define LINE5_CENTER_FORWARD_COUNT       150U   /* 300ms di thang sau khi quay 360 */

#define LINE5_S1_GPIO_Port           GPIOA
#define LINE5_S1_Pin                 GPIO_PIN_2
#define LINE5_S2_GPIO_Port           GPIOA
#define LINE5_S2_Pin                 GPIO_PIN_3
#define LINE5_S3_GPIO_Port           GPIOA
#define LINE5_S3_Pin                 GPIO_PIN_4
#define LINE5_S4_GPIO_Port           GPIOA
#define LINE5_S4_Pin                 GPIO_PIN_5
#define LINE5_S5_GPIO_Port           GPIOA
#define LINE5_S5_Pin                 GPIO_PIN_6

/* Finish marker: a wide black marker detected by all 5 sensors. */
#define FINISH_MARKER_ENABLE         1U
#define FINISH_MARKER_MASK           0x1FU
#define FINISH_MARKER_HOLD_MS        350U
#define FINISH_MARKER_HOLD_COUNT     (FINISH_MARKER_HOLD_MS / LINE5_TASK_PERIOD_MS)

/* ===================== OBSTACLE CONFIG ===================== */
#define OBSTACLE_SLOW_DIST_CM        30U
#define OBSTACLE_STOP_DIST_CM        3U
#define OBSTACLE_MIN_SPEED_255       ((int16_t)25)   /* Gan 3cm thi bo rat cham truoc khi dung */
#define HCSR04_TRIGGER_PERIOD_MS     35U    /* Tang tan so do de phan ung nhanh hon o 10cm/3cm */
#define HCSR04_DISTANCE_TIMEOUT_MS   150U   /* Mat echo nhanh thi khong dung gia tri cu qua lau */
#define HCSR04_VALID_MIN_CM          2U
#define HCSR04_VALID_MAX_CM          300U

#define OBSTACLE_IGNORE_AFTER_PICK_MS 2500U
#define WRONG_COLOR_IGNORE_MS        3000U
#define UNKNOWN_OBJECT_IGNORE_MS     1800U  /* Vat ao/mat mau: tam bo qua de tiep tuc do line */

/* ===================== COLOR CONFIG ===================== */
#define COLOR_SAMPLE_INTERVAL_MS     150U
#define COLOR_SAMPLE_COUNT           12U
#define COLOR_SAMPLE_MIN_VOTE        8U
#define COLOR_SETTLE_BEFORE_READ_MS  800U
#define COLOR_READ_TIMEOUT_MS        10000U   /* 10s: vat ao/khong doc duoc mau -> ne va di tiep */
#define COLOR_INVALID_TOLERANCE_MS   350U
#define COLOR_CLEAR_MIN              50U
#define COLOR_CLEAR_MAX              60000U

/* ===================== SERVO CONFIG ===================== */
#define SERVO1_DEFAULT_ANGLE         0U
#define SERVO1_GRAB_ANGLE            90U
#define SERVO2_DEFAULT_ANGLE         100U
#define SERVO2_GRAB_ANGLE            40U

/* Smaller step + longer update = slower servo movement, more stable gripping. */
#define SERVO_MOVE_STEP_US           8U
#define SERVO_MOVE_UPDATE_MS         12U
#define SERVO_STEP_TIMEOUT_MS        4500U
#define SERVO_HOLD_AFTER_MOVE_MS     600U
#define SERVO_GRIP_HOLD_MS           800U

/* ===================== WRONG COLOR AVOID CONFIG ===================== */
#define AVOID_BACKUP_MS              300U
#define AVOID_TURN_RIGHT_MS          520U
#define AVOID_FORWARD_MS             650U
#define AVOID_TURN_LEFT_MS           420U
#define AVOID_SEARCH_TIMEOUT_MS      3500U

#define AVOID_BACKUP_SPEED_255       ((int16_t)-55)
#define AVOID_TURN_FAST_255          ((int16_t)105)
#define AVOID_TURN_REVERSE_255       ((int16_t)-70)
#define AVOID_FORWARD_SPEED_255      ((int16_t)70)

/* ===================== GLOBAL OBJECTS ===================== */
ServoNB_HandleTypeDef servo1;   /* PA9  -> TIM1_CH2 */
ServoNB_HandleTypeDef servo2;   /* PA8  -> TIM1_CH1 */
HCSR04_HandleTypeDef hcsr04;

static uint8_t uart1_tx_buf[160];
static volatile uint8_t uart1_tx_busy = 0U;
static uint32_t uart_debug_last_tick = 0U;

volatile uint32_t distance_cm = 0U;
static uint32_t distance_last_valid_tick = 0U;
static uint32_t last_hcsr04_tick = 0U;

/* Debug watch variables */
volatile uint8_t  dbg_line5_mask = 0U;
volatile int16_t  dbg_line5_error = 0;
volatile int16_t  dbg_line5_correction = 0;
volatile int8_t   dbg_line5_remember = 0;
volatile uint16_t dbg_line5_lost_count = 0U;
volatile uint8_t  dbg_line5_search_phase = 0U;
volatile int16_t  dbg_line5_speed_cmd = LINE5_BASE_SPEED_255;
volatile uint8_t  dbg_line5_last_mask = 0U;
volatile int8_t   dbg_line5_recovery_dir = 0;
volatile uint8_t  dbg_line5_center_lost = 0U;

volatile uint32_t dbg_distance_cm = 0U;
volatile uint32_t dbg_distance_age_ms = 0U;
volatile uint8_t  dbg_distance_fresh = 0U;

volatile uint8_t  dbg_tcs_status = 0U;
volatile uint8_t  dbg_tcs_color = 0U;
volatile uint16_t dbg_tcs_c = 0U;
volatile uint16_t dbg_tcs_r = 0U;
volatile uint16_t dbg_tcs_g = 0U;
volatile uint16_t dbg_tcs_b = 0U;

volatile uint8_t  dbg_robot_state = 0U;
volatile uint8_t  dbg_color_sample_count = 0U;
volatile uint8_t  dbg_color_vote_red = 0U;
volatile uint8_t  dbg_color_vote_green = 0U;
volatile uint8_t  dbg_color_vote_blue = 0U;
volatile uint8_t  dbg_color_vote_yellow = 0U;
volatile uint8_t  dbg_has_target_color = 0U;
volatile uint8_t  dbg_target_color = 0U;
volatile uint8_t  dbg_avoid_return_state = 0U;
volatile uint16_t dbg_finish_marker_count = 0U;

static int16_t line5_last_error = 0;
static int8_t line5_remember = 0;
static uint8_t line5_last_mask = 0U;
static int8_t line5_recovery_dir = 0;
static uint8_t line5_center_lost_recovery = 0U;
static uint16_t line5_lost_count = 0U;
static int16_t line5_speed_cmd = LINE5_BASE_SPEED_255;
static uint16_t finish_marker_count = 0U;

static const int16_t line5_weight[5] =
{
    -2000, -1000, 0, 1000, 2000
};

static TCS34725_Color_t target_color = TCS34725_COLOR_NONE;
static uint8_t has_target_color = 0U;
static uint32_t obstacle_ignore_until = 0U;

static uint32_t action_start_tick = 0U;
static uint32_t color_last_valid_tick = 0U;
static uint32_t color_last_sample_tick = 0U;
static uint8_t color_sample_count = 0U;
static uint8_t color_vote_red = 0U;
static uint8_t color_vote_green = 0U;
static uint8_t color_vote_blue = 0U;
static uint8_t color_vote_yellow = 0U;

typedef enum
{
    STATE_LINE_TO_PICK = 0,
    STATE_READ_PICK_COLOR,
    STATE_GRAB_SERVO1_DOWN,
    STATE_GRAB_SERVO1_DOWN_HOLD,
    STATE_GRAB_SERVO2_CLOSE,
    STATE_GRAB_SERVO2_CLOSE_HOLD,
    STATE_GRAB_SERVO1_UP,
    STATE_LINE_CARRYING,
    STATE_READ_TARGET_COLOR,
    STATE_DROP_SERVO1_DOWN,
    STATE_DROP_SERVO1_DOWN_HOLD,
    STATE_DROP_SERVO2_OPEN,
    STATE_DROP_SERVO2_OPEN_HOLD,
    STATE_DROP_SERVO1_UP,
    STATE_FINISH_RUN,
    STATE_STOP_END,
    STATE_WRONG_COLOR_BACKUP,
    STATE_WRONG_COLOR_TURN_RIGHT,
    STATE_WRONG_COLOR_FORWARD,
    STATE_WRONG_COLOR_TURN_LEFT,
    STATE_WRONG_COLOR_SEARCH_LINE
} RobotState_t;

static RobotState_t robotState = STATE_LINE_TO_PICK;
static RobotState_t avoid_return_state = STATE_LINE_CARRYING;

static void Line5_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = LINE5_S1_Pin | LINE5_S2_Pin | LINE5_S3_Pin |
                          LINE5_S4_Pin | LINE5_S5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static int16_t Clamp16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static int32_t MapLong(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    if (in_max == in_min)
    {
        return out_min;
    }

    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;

    return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}

static uint32_t PWM_PercentToCCR(TIM_HandleTypeDef *htim, uint8_t percent)
{
    uint32_t arr;

    if (percent > PWM_MAX_PERCENT)
    {
        percent = PWM_MAX_PERCENT;
    }

    arr = __HAL_TIM_GET_AUTORELOAD(htim);
    return ((uint32_t)percent * (arr + 1U)) / 100U;
}

static void PWM_Start_All(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0U);
}

static void Motor_Right_SetPercent(int16_t percent)
{
#if RIGHT_MOTOR_INVERT
    percent = -percent;
#endif

    percent = Clamp16(percent, -100, 100);

    if ((percent > 0) && (percent < (int16_t)MOTOR_MIN_PERCENT))
    {
        percent = (int16_t)MOTOR_MIN_PERCENT;
    }
    else if ((percent < 0) && (percent > -(int16_t)MOTOR_MIN_PERCENT))
    {
        percent = -(int16_t)MOTOR_MIN_PERCENT;
    }

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
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_PercentToCCR(&htim2, (uint8_t)percent));
    }
    else
    {
        HAL_GPIO_WritePin(IN_1_R_GPIO_Port, IN_1_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN_2_R_GPIO_Port, IN_2_R_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, PWM_PercentToCCR(&htim2, (uint8_t)(-percent)));
    }
}

static void Motor_Left_SetPercent(int16_t percent)
{
#if LEFT_MOTOR_INVERT
    percent = -percent;
#endif

    percent = Clamp16(percent, -100, 100);

    if ((percent > 0) && (percent < (int16_t)MOTOR_MIN_PERCENT))
    {
        percent = (int16_t)MOTOR_MIN_PERCENT;
    }
    else if ((percent < 0) && (percent > -(int16_t)MOTOR_MIN_PERCENT))
    {
        percent = -(int16_t)MOTOR_MIN_PERCENT;
    }

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
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_PercentToCCR(&htim2, (uint8_t)percent));
    }
    else
    {
        HAL_GPIO_WritePin(IN_1_L_GPIO_Port, IN_1_L_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN_2_L_GPIO_Port, IN_2_L_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PWM_PercentToCCR(&htim2, (uint8_t)(-percent)));
    }
}

static void Motor_RunPercent(int16_t left_percent, int16_t right_percent)
{
    Motor_Left_SetPercent(left_percent);
    Motor_Right_SetPercent(right_percent);
}

static void Motor_Run255(int16_t left_255, int16_t right_255)
{
    int16_t left_percent;
    int16_t right_percent;

    left_255 = Clamp16(left_255, -255, 255);
    right_255 = Clamp16(right_255, -255, 255);

    left_percent = (int16_t)(((int32_t)left_255 * 100L) / 255L);
    right_percent = (int16_t)(((int32_t)right_255 * 100L) / 255L);

    Motor_RunPercent(left_percent, right_percent);
}

static void Motor_AllStop(void)
{
    Motor_Right_SetPercent(0);
    Motor_Left_SetPercent(0);
}

static uint8_t Line5_ReadOne(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(port, pin);

#if LINE5_ACTIVE_LOW
    return (state == GPIO_PIN_RESET) ? 1U : 0U;
#else
    return (state == GPIO_PIN_SET) ? 1U : 0U;
#endif
}

static uint8_t Line5_ReadMask(void)
{
    uint8_t mask = 0U;

    if (Line5_ReadOne(LINE5_S1_GPIO_Port, LINE5_S1_Pin) != 0U) mask |= 0x10U;
    if (Line5_ReadOne(LINE5_S2_GPIO_Port, LINE5_S2_Pin) != 0U) mask |= 0x08U;
    if (Line5_ReadOne(LINE5_S3_GPIO_Port, LINE5_S3_Pin) != 0U) mask |= 0x04U;
    if (Line5_ReadOne(LINE5_S4_GPIO_Port, LINE5_S4_Pin) != 0U) mask |= 0x02U;
    if (Line5_ReadOne(LINE5_S5_GPIO_Port, LINE5_S5_Pin) != 0U) mask |= 0x01U;

    return mask;
}

static int16_t Line5_MapSpeedByError(int16_t base_speed, int16_t error_abs)
{
    int32_t speed;

    if (error_abs < 0) error_abs = (int16_t)(-error_abs);
    if (error_abs > 2000) error_abs = 2000;

    speed = (int32_t)base_speed - (((int32_t)(base_speed - LINE5_MIN_CURVE_SPEED_255) * error_abs) / 2000L);
    return Clamp16((int16_t)speed, LINE5_MIN_CURVE_SPEED_255, LINE5_MAX_SPEED_255);
}

static void Line5_SetSpeed(int16_t speed_255)
{
    line5_speed_cmd = Clamp16(speed_255, 0, LINE5_MAX_SPEED_255);
    dbg_line5_speed_cmd = line5_speed_cmd;
}

static void Line5_ResetMemory(void)
{
    line5_last_error = 0;
    line5_remember = 0;
    line5_last_mask = 0U;
    line5_recovery_dir = 0;
    line5_center_lost_recovery = 0U;
    line5_lost_count = 0U;
    dbg_line5_error = 0;
    dbg_line5_correction = 0;
    dbg_line5_remember = 0;
    dbg_line5_lost_count = 0U;
    dbg_line5_search_phase = 0U;
    dbg_line5_last_mask = 0U;
    dbg_line5_recovery_dir = 0;
    dbg_line5_center_lost = 0U;
}

static void Line5_SearchLostLine(void)
{
    uint16_t phase_count;
    int8_t primary_dir;

    /*
     * Mat line V6:
     * line5_recovery_dir duoc chot NGAY tai thoi diem vua mat line,
     * nen xe luon quay ve dung phia line vua roi khoi cam bien truoc.
     */
    primary_dir = line5_recovery_dir;

    if (primary_dir == 0)
    {
        primary_dir = line5_remember;
    }

    if (primary_dir == 0)
    {
        if (line5_last_error > 80)
        {
            primary_dir = 1;
        }
        else if (line5_last_error < -80)
        {
            primary_dir = -1;
        }
        else
        {
            primary_dir = 1;
        }
    }

    dbg_line5_recovery_dir = primary_dir;
    dbg_line5_center_lost = line5_center_lost_recovery;

#if LINE5_CENTER_LOST_ENABLE
    if (line5_center_lost_recovery != 0U)
    {
        if (line5_lost_count <= LINE5_CENTER_SPIN_COUNT)
        {
            dbg_line5_search_phase = 7U; /* mat line o giua: quay 360 do */
            if (primary_dir >= 0)
            {
                Motor_Run255(LINE5_CENTER_SPIN_TURN_255, LINE5_CENTER_SPIN_REVERSE_255);
            }
            else
            {
                Motor_Run255(LINE5_CENTER_SPIN_REVERSE_255, LINE5_CENTER_SPIN_TURN_255);
            }
            return;
        }

        if (line5_lost_count <= (LINE5_CENTER_SPIN_COUNT + LINE5_CENTER_FORWARD_COUNT))
        {
            dbg_line5_search_phase = 8U; /* sau 360 do: di thang ngan */
            Motor_Run255(LINE5_CENTER_FORWARD_255, LINE5_CENTER_FORWARD_255);
            return;
        }

        /* Sau quay 360 + di thang, tiep tuc chu ky quet trai/phai nhu cu. */
        phase_count = (uint16_t)((line5_lost_count - LINE5_CENTER_SPIN_COUNT - LINE5_CENTER_FORWARD_COUNT) % LINE5_SEARCH_CYCLE_COUNT);

        if (phase_count < LINE5_SEARCH_SWEEP_COUNT)
        {
            dbg_line5_search_phase = 3U;
            if (primary_dir > 0)
            {
                Motor_Run255(LINE5_LOST_SWEEP_TURN_255, LINE5_LOST_SWEEP_REVERSE_255);
            }
            else
            {
                Motor_Run255(LINE5_LOST_SWEEP_REVERSE_255, LINE5_LOST_SWEEP_TURN_255);
            }
        }
        else if (phase_count < (2U * LINE5_SEARCH_SWEEP_COUNT))
        {
            dbg_line5_search_phase = 4U;
            if (primary_dir > 0)
            {
                Motor_Run255(LINE5_LOST_SWEEP_REVERSE_255, LINE5_LOST_SWEEP_TURN_255);
            }
            else
            {
                Motor_Run255(LINE5_LOST_SWEEP_TURN_255, LINE5_LOST_SWEEP_REVERSE_255);
            }
        }
        else if (phase_count < ((2U * LINE5_SEARCH_SWEEP_COUNT) + LINE5_SEARCH_FORWARD_COUNT))
        {
            dbg_line5_search_phase = 5U;
            Motor_Run255(LINE5_LOST_FORWARD_255, LINE5_LOST_FORWARD_255);
        }
        else
        {
            dbg_line5_search_phase = 6U;
            Motor_Run255(LINE5_LOST_BACKUP_255, LINE5_LOST_BACKUP_255);
        }
        return;
    }
#endif

    if (line5_lost_count <= LINE5_LOST_BACKTRACK_COUNT)
    {
        dbg_line5_search_phase = 1U; /* lui nhanh ve vi tri line vua mat */
        Motor_Run255(LINE5_LOST_BACKTRACK_SPEED_255, LINE5_LOST_BACKTRACK_SPEED_255);
        return;
    }

    if (line5_lost_count <= (LINE5_LOST_BACKTRACK_COUNT + LINE5_LOST_RETURN_COUNT))
    {
        dbg_line5_search_phase = 2U; /* quay nhanh ve huong line vua mat */
        if (primary_dir > 0)
        {
            Motor_Run255(LINE5_LOST_RETURN_TURN_255, LINE5_LOST_RETURN_REVERSE_255);
        }
        else
        {
            Motor_Run255(LINE5_LOST_RETURN_REVERSE_255, LINE5_LOST_RETURN_TURN_255);
        }
        return;
    }

    phase_count = (uint16_t)((line5_lost_count - LINE5_LOST_BACKTRACK_COUNT - LINE5_LOST_RETURN_COUNT) % LINE5_SEARCH_CYCLE_COUNT);

    if (phase_count < LINE5_SEARCH_SWEEP_COUNT)
    {
        dbg_line5_search_phase = 3U; /* quet manh theo huong uu tien */
        if (primary_dir > 0)
        {
            Motor_Run255(LINE5_LOST_SWEEP_TURN_255, LINE5_LOST_SWEEP_REVERSE_255);
        }
        else
        {
            Motor_Run255(LINE5_LOST_SWEEP_REVERSE_255, LINE5_LOST_SWEEP_TURN_255);
        }
    }
    else if (phase_count < (2U * LINE5_SEARCH_SWEEP_COUNT))
    {
        dbg_line5_search_phase = 4U; /* quet manh nguoc lai */
        if (primary_dir > 0)
        {
            Motor_Run255(LINE5_LOST_SWEEP_REVERSE_255, LINE5_LOST_SWEEP_TURN_255);
        }
        else
        {
            Motor_Run255(LINE5_LOST_SWEEP_TURN_255, LINE5_LOST_SWEEP_REVERSE_255);
        }
    }
    else if (phase_count < ((2U * LINE5_SEARCH_SWEEP_COUNT) + LINE5_SEARCH_FORWARD_COUNT))
    {
        dbg_line5_search_phase = 5U; /* tien cham de bat lai line neu xe dang sat vach */
        Motor_Run255(LINE5_LOST_FORWARD_255, LINE5_LOST_FORWARD_255);
    }
    else
    {
        dbg_line5_search_phase = 6U; /* lui nhe roi lap lai chu ky quet */
        Motor_Run255(LINE5_LOST_BACKUP_255, LINE5_LOST_BACKUP_255);
    }
}

static void Line5_Task(void)
{
    uint8_t mask;
    uint8_t count = 0U;
    int32_t weighted_sum = 0;
    int16_t error;
    int16_t error_delta;
    int32_t correction32;
    int16_t correction;
    int16_t speed;
    int16_t left_speed;
    int16_t right_speed;

    mask = Line5_ReadMask();
    dbg_line5_mask = mask;

    if (mask & 0x10U) { weighted_sum += line5_weight[0]; count++; }
    if (mask & 0x08U) { weighted_sum += line5_weight[1]; count++; }
    if (mask & 0x04U) { weighted_sum += line5_weight[2]; count++; }
    if (mask & 0x02U) { weighted_sum += line5_weight[3]; count++; }
    if (mask & 0x01U) { weighted_sum += line5_weight[4]; count++; }

    if (count == 0U)
    {
        if (line5_lost_count == 0U)
        {
            /* Chot huong tim line tai khoanh khac vua mat line. */
            if (line5_last_mask & 0x01U)
            {
                line5_recovery_dir = 1;
            }
            else if (line5_last_mask & 0x10U)
            {
                line5_recovery_dir = -1;
            }
            else if (line5_remember != 0)
            {
                line5_recovery_dir = line5_remember;
            }
            else if (line5_last_error > 80)
            {
                line5_recovery_dir = 1;
            }
            else if (line5_last_error < -80)
            {
                line5_recovery_dir = -1;
            }
            else
            {
                line5_recovery_dir = 1;
            }

            /* Neu mat line khi line vua nam gan giua, uu tien quay 360 do truoc.
             * Cac mask gan giua thuong co S3 hoac sai so nho quanh 0.
             */
            if ((line5_last_mask != 0U) &&
                (((line5_last_mask & 0x04U) != 0U) ||
                 ((line5_last_error >= -LINE5_CENTER_ERROR_WINDOW) &&
                  (line5_last_error <=  LINE5_CENTER_ERROR_WINDOW))))
            {
                line5_center_lost_recovery = 1U;
            }
            else
            {
                line5_center_lost_recovery = 0U;
            }
            dbg_line5_center_lost = line5_center_lost_recovery;
        }

        if (line5_lost_count < 0xFFFFU)
        {
            line5_lost_count++;
        }
        dbg_line5_lost_count = line5_lost_count;

        Line5_SearchLostLine();
        return;
    }

    line5_lost_count = 0U;
    line5_recovery_dir = 0;
    line5_center_lost_recovery = 0U;
    line5_last_mask = mask;
    dbg_line5_lost_count = 0U;
    dbg_line5_search_phase = 0U;
    dbg_line5_last_mask = line5_last_mask;
    dbg_line5_recovery_dir = 0;
    dbg_line5_center_lost = 0U;

    error = (int16_t)(weighted_sum / (int32_t)count);
    error_delta = (int16_t)(error - line5_last_error);

    correction32 = ((int32_t)LINE5_KP_NUM * (int32_t)error) +
                   ((int32_t)LINE5_KD_NUM * (int32_t)error_delta);
    correction32 /= LINE5_PID_DIV;

    correction = Clamp16((int16_t)correction32, -LINE5_MAX_STEER_255, LINE5_MAX_STEER_255);
    speed = Line5_MapSpeedByError(line5_speed_cmd, error);

    left_speed = Clamp16((int16_t)(speed + correction), -255, 255);
    right_speed = Clamp16((int16_t)(speed - correction), -255, 255);

    Motor_Run255(left_speed, right_speed);

    if ((mask & 0x01U) != 0U)
    {
        line5_remember = 1;
    }
    else if ((mask & 0x10U) != 0U)
    {
        line5_remember = -1;
    }
    else if (error > 180)
    {
        line5_remember = 1;
    }
    else if (error < -180)
    {
        line5_remember = -1;
    }

    line5_last_error = error;

    dbg_line5_error = error;
    dbg_line5_correction = correction;
    dbg_line5_remember = line5_remember;
}

static uint8_t Line5_FinishMarkerTask(void)
{
#if FINISH_MARKER_ENABLE
    uint8_t mask = Line5_ReadMask();
    dbg_line5_mask = mask;

    if ((mask & FINISH_MARKER_MASK) == FINISH_MARKER_MASK)
    {
        if (finish_marker_count < 0xFFFFU)
        {
            finish_marker_count++;
        }
    }
    else
    {
        finish_marker_count = 0U;
    }

    dbg_finish_marker_count = finish_marker_count;

    if (finish_marker_count >= FINISH_MARKER_HOLD_COUNT)
    {
        Motor_AllStop();
        return 1U;
    }
#else
    (void)finish_marker_count;
#endif

    return 0U;
}

static void Servo_App_Init(void)
{
    ServoNB_Init(&servo1, &htim1, TIM_CHANNEL_2, 500, 2500, 1500, 1);
    ServoNB_Init(&servo2, &htim1, TIM_CHANNEL_1, 500, 2500, 1500, 1);

    ServoNB_Start(&servo1);
    ServoNB_Start(&servo2);

    ServoNB_WriteAngle(&servo1, SERVO1_DEFAULT_ANGLE);
    ServoNB_WriteAngle(&servo2, SERVO2_DEFAULT_ANGLE);
}

static void Servo_CommandAngle(ServoNB_HandleTypeDef *servo, uint16_t target_angle)
{
    ServoNB_MoveToAngle(servo, target_angle, SERVO_MOVE_STEP_US, SERVO_MOVE_UPDATE_MS);
    action_start_tick = HAL_GetTick();
}

static uint8_t Servo_WaitDone(uint32_t timeout_ms)
{
    if ((ServoNB_IsMoving(&servo1) == 0U) && (ServoNB_IsMoving(&servo2) == 0U))
    {
        return 1U;
    }

    if ((HAL_GetTick() - action_start_tick) >= timeout_ms)
    {
        return 1U;
    }

    return 0U;
}

static void HCSR04_App_Init(void)
{
    hcsr04.TRIG_Port = GPIOB;
    hcsr04.TRIG_Pin  = GPIO_PIN_5;
    hcsr04.ECHO_Port = GPIOB;
    hcsr04.ECHO_Pin  = GPIO_PIN_4;
    hcsr04.htim_us   = &htim3;

    HCSR04_Init(&hcsr04);
    HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);

    last_hcsr04_tick = HAL_GetTick() - HCSR04_TRIGGER_PERIOD_MS;
    distance_cm = 0U;
    distance_last_valid_tick = 0U;
}

static void HCSR04_App_Task(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t d;

    if ((now - last_hcsr04_tick) >= HCSR04_TRIGGER_PERIOD_MS)
    {
        last_hcsr04_tick = now;
        HCSR04_Trigger(&hcsr04);
    }

    d = HCSR04_ReadDistanceCm(&hcsr04);
    if ((d >= HCSR04_VALID_MIN_CM) && (d <= HCSR04_VALID_MAX_CM))
    {
        distance_cm = d;
        distance_last_valid_tick = now;
    }

    if ((distance_cm > 0U) && ((now - distance_last_valid_tick) > HCSR04_DISTANCE_TIMEOUT_MS))
    {
        distance_cm = 0U;
    }

    dbg_distance_cm = distance_cm;
    dbg_distance_age_ms = (distance_last_valid_tick == 0U) ? 0xFFFFFFFFUL : (now - distance_last_valid_tick);
    dbg_distance_fresh = Distance_IsFresh(now);
}

static uint8_t Distance_IsFresh(uint32_t now)
{
    if (distance_cm == 0U) return 0U;
    if (distance_last_valid_tick == 0U) return 0U;
    if ((now - distance_last_valid_tick) > HCSR04_DISTANCE_TIMEOUT_MS) return 0U;
    return 1U;
}

static uint8_t Distance_UpdateSpeedAndCheckStop(uint32_t now, int16_t base_speed_255)
{
    int32_t mapped_speed;

    if (Distance_IsFresh(now) == 0U)
    {
        Line5_SetSpeed(base_speed_255);
        return 0U;
    }

    if (distance_cm <= OBSTACLE_STOP_DIST_CM)
    {
        Line5_SetSpeed(0);
        return 1U;
    }

    if (distance_cm <= OBSTACLE_SLOW_DIST_CM)
    {
        mapped_speed = MapLong((int32_t)distance_cm,
                               (int32_t)OBSTACLE_STOP_DIST_CM,
                               (int32_t)OBSTACLE_SLOW_DIST_CM,
                               (int32_t)OBSTACLE_MIN_SPEED_255,
                               (int32_t)base_speed_255);
        Line5_SetSpeed(Clamp16((int16_t)mapped_speed, OBSTACLE_MIN_SPEED_255, base_speed_255));
    }
    else
    {
        Line5_SetSpeed(base_speed_255);
    }

    return 0U;
}

static void TCS34725_App_Init(void)
{
    TCS34725_Init();
}

static void TCS34725_App_Task(void)
{
    TCS34725_Raw_t raw;

    TCS34725_Task();

    raw = TCS34725_GetLastRaw();
    dbg_tcs_status = (uint8_t)TCS34725_GetStatus();
    dbg_tcs_color = (uint8_t)TCS34725_GetCurrentColor();
    dbg_tcs_c = raw.c;
    dbg_tcs_r = raw.r;
    dbg_tcs_g = raw.g;
    dbg_tcs_b = raw.b;
}

static uint8_t Color_IsValid(TCS34725_Color_t color)
{
    if (color == TCS34725_COLOR_RED) return 1U;
    if (color == TCS34725_COLOR_GREEN) return 1U;
    if (color == TCS34725_COLOR_BLUE) return 1U;
    if (color == TCS34725_COLOR_YELLOW) return 1U;
    return 0U;
}

static TCS34725_Color_t Color_ClassifyRawStable(const TCS34725_Raw_t *raw)
{
    uint32_t sum;
    uint32_t rn;
    uint32_t gn;
    uint32_t bn;

    if (raw == 0) return TCS34725_COLOR_NONE;
    if (raw->c < COLOR_CLEAR_MIN) return TCS34725_COLOR_NONE;
    if (raw->c > COLOR_CLEAR_MAX) return TCS34725_COLOR_UNKNOWN;

    sum = (uint32_t)raw->r + (uint32_t)raw->g + (uint32_t)raw->b;
    if (sum == 0U) return TCS34725_COLOR_NONE;

    rn = ((uint32_t)raw->r * 1000U) / sum;
    gn = ((uint32_t)raw->g * 1000U) / sum;
    bn = ((uint32_t)raw->b * 1000U) / sum;

    /* Default normalized thresholds. Tune these after recording C/R/G/B logs. */
    if ((rn > 430U) && (rn > (gn + 70U)) && (rn > (bn + 90U)))
    {
        return TCS34725_COLOR_RED;
    }

    if ((gn > 390U) && (gn > (rn + 45U)) && (gn > (bn + 50U)))
    {
        return TCS34725_COLOR_GREEN;
    }

    if ((bn > 380U) && (bn > (rn + 60U)) && (bn > (gn + 45U)))
    {
        return TCS34725_COLOR_BLUE;
    }

    if ((rn > 330U) && (gn > 330U) && (bn < 270U))
    {
        return TCS34725_COLOR_YELLOW;
    }

    return TCS34725_COLOR_UNKNOWN;
}

static void ColorStable_Reset(void)
{
    color_last_valid_tick = HAL_GetTick();
    color_last_sample_tick = 0U;
    color_sample_count = 0U;
    color_vote_red = 0U;
    color_vote_green = 0U;
    color_vote_blue = 0U;
    color_vote_yellow = 0U;

    dbg_color_sample_count = 0U;
    dbg_color_vote_red = 0U;
    dbg_color_vote_green = 0U;
    dbg_color_vote_blue = 0U;
    dbg_color_vote_yellow = 0U;
}

static void ColorVote_Add(TCS34725_Color_t color)
{
    if (color == TCS34725_COLOR_RED) color_vote_red++;
    else if (color == TCS34725_COLOR_GREEN) color_vote_green++;
    else if (color == TCS34725_COLOR_BLUE) color_vote_blue++;
    else if (color == TCS34725_COLOR_YELLOW) color_vote_yellow++;

    dbg_color_vote_red = color_vote_red;
    dbg_color_vote_green = color_vote_green;
    dbg_color_vote_blue = color_vote_blue;
    dbg_color_vote_yellow = color_vote_yellow;
}

static uint8_t ColorVote_GetWinner(TCS34725_Color_t *color_out)
{
    uint8_t best = color_vote_red;
    TCS34725_Color_t best_color = TCS34725_COLOR_RED;

    if (color_vote_green > best)
    {
        best = color_vote_green;
        best_color = TCS34725_COLOR_GREEN;
    }

    if (color_vote_blue > best)
    {
        best = color_vote_blue;
        best_color = TCS34725_COLOR_BLUE;
    }

    if (color_vote_yellow > best)
    {
        best = color_vote_yellow;
        best_color = TCS34725_COLOR_YELLOW;
    }

    if ((best >= COLOR_SAMPLE_MIN_VOTE) && (color_out != 0))
    {
        *color_out = best_color;
        return 1U;
    }

    return 0U;
}

static uint8_t ColorStable_Task(TCS34725_Color_t *stable_color_out)
{
    TCS34725_Raw_t raw;
    TCS34725_Color_t now_color;
    uint32_t now = HAL_GetTick();

    if (stable_color_out == 0) return 0U;

    /* Wait after the robot stops so the sensor reads a stable surface/object. */
    if ((now - action_start_tick) < COLOR_SETTLE_BEFORE_READ_MS)
    {
        return 0U;
    }

    if ((now - color_last_sample_tick) < COLOR_SAMPLE_INTERVAL_MS)
    {
        return 0U;
    }
    color_last_sample_tick = now;

    if (TCS34725_GetStatus() != TCS34725_STATUS_READY)
    {
        if ((color_sample_count > 0U) && ((now - color_last_valid_tick) <= COLOR_INVALID_TOLERANCE_MS))
        {
            return 0U;
        }
        ColorStable_Reset();
        return 0U;
    }

    raw = TCS34725_GetLastRaw();
    now_color = Color_ClassifyRawStable(&raw);

    if (Color_IsValid(now_color) == 0U)
    {
        if ((color_sample_count > 0U) && ((now - color_last_valid_tick) <= COLOR_INVALID_TOLERANCE_MS))
        {
            return 0U;
        }
        ColorStable_Reset();
        return 0U;
    }

    color_last_valid_tick = now;
    ColorVote_Add(now_color);
    color_sample_count++;
    dbg_color_sample_count = color_sample_count;

    if (color_sample_count >= COLOR_SAMPLE_COUNT)
    {
        if (ColorVote_GetWinner(stable_color_out) != 0U)
        {
            return 1U;
        }
        ColorStable_Reset();
    }

    return 0U;
}

static uint16_t UART1_AddString(uint16_t index, const char *s)
{
    while ((*s != '\0') && (index < (sizeof(uart1_tx_buf) - 1U)))
    {
        uart1_tx_buf[index++] = (uint8_t)(*s++);
    }
    return index;
}

static uint16_t UART1_AddNumber(uint16_t index, uint32_t num)
{
    char temp[10];
    uint8_t i = 0U;

    if (num == 0U)
    {
        if (index < sizeof(uart1_tx_buf)) uart1_tx_buf[index++] = '0';
        return index;
    }

    while ((num > 0U) && (i < sizeof(temp)))
    {
        temp[i++] = (char)('0' + (num % 10U));
        num /= 10U;
    }

    while ((i > 0U) && (index < sizeof(uart1_tx_buf)))
    {
        uart1_tx_buf[index++] = (uint8_t)temp[--i];
    }

    return index;
}

static uint16_t UART1_MakeDebugString(void)
{
    uint16_t index = 0U;
    TCS34725_Raw_t raw = TCS34725_GetLastRaw();

    index = UART1_AddString(index, "S=");
    index = UART1_AddNumber(index, (uint32_t)robotState);
    index = UART1_AddString(index, " D=");
    index = UART1_AddNumber(index, distance_cm);
    index = UART1_AddString(index, " TCS=");
    index = UART1_AddNumber(index, (uint32_t)TCS34725_GetStatus());
    index = UART1_AddString(index, " Col=");
    index = UART1_AddString(index, TCS34725_ColorToString(TCS34725_GetCurrentColor()));
    index = UART1_AddString(index, " C=");
    index = UART1_AddNumber(index, raw.c);
    index = UART1_AddString(index, " R=");
    index = UART1_AddNumber(index, raw.r);
    index = UART1_AddString(index, " G=");
    index = UART1_AddNumber(index, raw.g);
    index = UART1_AddString(index, " B=");
    index = UART1_AddNumber(index, raw.b);
    index = UART1_AddString(index, "\r\n");

    return index;
}

static void UART1_Debug_Task(void)
{
#if ROBOT_UART_DEBUG
    uint32_t now = HAL_GetTick();

    if ((now - uart_debug_last_tick) < UART_DEBUG_PERIOD_MS)
    {
        return;
    }
    uart_debug_last_tick = now;

    if (uart1_tx_busy == 0U)
    {
        uint16_t len = UART1_MakeDebugString();
        uart1_tx_busy = 1U;
        if (HAL_UART_Transmit_IT(&huart1, uart1_tx_buf, len) != HAL_OK)
        {
            uart1_tx_busy = 0U;
        }
    }
#endif
}

static void Robot_StartAvoid(RobotState_t return_state, uint32_t now)
{
    /*
     * Goi khi gap vat sai mau hoac vat can van con o truoc xe sau khi doc mau timeout.
     * Xe se ne phai, tim lai line, roi quay ve return_state.
     */
    avoid_return_state = return_state;
    dbg_avoid_return_state = (uint8_t)return_state;
    ColorStable_Reset();
    Line5_ResetMemory();
    action_start_tick = now;
    robotState = STATE_WRONG_COLOR_BACKUP;
}

static void Robot_HandleUnknownObject(RobotState_t return_state, uint32_t now)
{
    /*
     * Vat ao/khong doc duoc mau trong 10s:
     * - Neu sieu am khong con thay vat gan -> tiep tuc do line, bo qua sieu am trong khoang ngan.
     * - Neu vat van con trong vung <= 10cm -> ne phai de tranh dam vao vat that.
     */
    ColorStable_Reset();

    if ((Distance_IsFresh(now) != 0U) && (distance_cm <= OBSTACLE_SLOW_DIST_CM))
    {
        Robot_StartAvoid(return_state, now);
        return;
    }

    Line5_ResetMemory();
    obstacle_ignore_until = now + UNKNOWN_OBJECT_IGNORE_MS;
    robotState = return_state;
}

static void Robot_Main_Task(void)
{
    TCS34725_Color_t stable_color;
    uint32_t now = HAL_GetTick();

    /* Uu tien cao nhat: cap nhat sieu am truoc moi quyet dinh dieu khien. */
    HCSR04_App_Task();

    /* TCS la ngoai vi phu thuoc: cap nhat non-blocking, khong thay the uu tien line/sieu am. */
    TCS34725_App_Task();
    ServoNB_Task(&servo1);
    ServoNB_Task(&servo2);
    UART1_Debug_Task();

    dbg_robot_state = (uint8_t)robotState;
    dbg_has_target_color = has_target_color;
    dbg_target_color = (uint8_t)target_color;
    dbg_avoid_return_state = (uint8_t)avoid_return_state;

    switch (robotState)
    {
        case STATE_LINE_TO_PICK:
            if (now >= obstacle_ignore_until)
            {
                if (Distance_UpdateSpeedAndCheckStop(now, LINE5_BASE_SPEED_255) != 0U)
                {
                    Motor_AllStop();
                    ColorStable_Reset();
                    action_start_tick = now;
                    robotState = STATE_READ_PICK_COLOR;
                    break;
                }
            }
            else
            {
                Line5_SetSpeed(LINE5_BASE_SPEED_255);
            }
            Line5_Task();
            break;

        case STATE_READ_PICK_COLOR:
            Motor_AllStop();
            if (ColorStable_Task(&stable_color) != 0U)
            {
                target_color = stable_color;
                has_target_color = 1U;
                Servo_CommandAngle(&servo1, SERVO1_GRAB_ANGLE);
                robotState = STATE_GRAB_SERVO1_DOWN;
            }
            else if ((now - action_start_tick) >= COLOR_READ_TIMEOUT_MS)
            {
                /* Vat ao/khong doc duoc mau: tiep tuc neu het vat, hoac ne neu vat van chan truoc xe. */
                Robot_HandleUnknownObject(STATE_LINE_TO_PICK, now);
            }
            break;

        case STATE_GRAB_SERVO1_DOWN:
            Motor_AllStop();
            if (Servo_WaitDone(SERVO_STEP_TIMEOUT_MS) != 0U)
            {
                action_start_tick = now;
                robotState = STATE_GRAB_SERVO1_DOWN_HOLD;
            }
            break;

        case STATE_GRAB_SERVO1_DOWN_HOLD:
            Motor_AllStop();
            if ((now - action_start_tick) >= SERVO_HOLD_AFTER_MOVE_MS)
            {
                Servo_CommandAngle(&servo2, SERVO2_GRAB_ANGLE);
                robotState = STATE_GRAB_SERVO2_CLOSE;
            }
            break;

        case STATE_GRAB_SERVO2_CLOSE:
            Motor_AllStop();
            if (Servo_WaitDone(SERVO_STEP_TIMEOUT_MS) != 0U)
            {
                action_start_tick = now;
                robotState = STATE_GRAB_SERVO2_CLOSE_HOLD;
            }
            break;

        case STATE_GRAB_SERVO2_CLOSE_HOLD:
            Motor_AllStop();
            if ((now - action_start_tick) >= SERVO_GRIP_HOLD_MS)
            {
                Servo_CommandAngle(&servo1, SERVO1_DEFAULT_ANGLE);
                robotState = STATE_GRAB_SERVO1_UP;
            }
            break;

        case STATE_GRAB_SERVO1_UP:
            Motor_AllStop();
            if (Servo_WaitDone(SERVO_STEP_TIMEOUT_MS) != 0U)
            {
                ServoNB_WriteAngle(&servo1, SERVO1_DEFAULT_ANGLE);
                ServoNB_WriteAngle(&servo2, SERVO2_GRAB_ANGLE);
                Line5_ResetMemory();
                obstacle_ignore_until = now + OBSTACLE_IGNORE_AFTER_PICK_MS;
                robotState = STATE_LINE_CARRYING;
            }
            break;

        case STATE_LINE_CARRYING:
            if (now >= obstacle_ignore_until)
            {
                if (Distance_UpdateSpeedAndCheckStop(now, LINE5_CARRY_SPEED_255) != 0U)
                {
                    Motor_AllStop();
                    ColorStable_Reset();
                    action_start_tick = now;
                    robotState = STATE_READ_TARGET_COLOR;
                    break;
                }
            }
            else
            {
                Line5_SetSpeed(LINE5_CARRY_SPEED_255);
            }
            Line5_Task();
            break;

        case STATE_READ_TARGET_COLOR:
            Motor_AllStop();
            if (ColorStable_Task(&stable_color) != 0U)
            {
                if ((has_target_color != 0U) && (stable_color == target_color))
                {
                    Servo_CommandAngle(&servo1, SERVO1_GRAB_ANGLE);
                    robotState = STATE_DROP_SERVO1_DOWN;
                }
                else
                {
                    /* Wrong color: avoid this object and continue carrying/searching. */
                    Robot_StartAvoid(STATE_LINE_CARRYING, now);
                }
            }
            else if ((now - action_start_tick) >= COLOR_READ_TIMEOUT_MS)
            {
                /* Khong doc duoc mau dich: tiep tuc neu vat ao, hoac ne neu vat van chan truoc xe. */
                Robot_HandleUnknownObject(STATE_LINE_CARRYING, now);
            }
            break;

        case STATE_WRONG_COLOR_BACKUP:
            Motor_Run255(AVOID_BACKUP_SPEED_255, AVOID_BACKUP_SPEED_255);
            if ((now - action_start_tick) >= AVOID_BACKUP_MS)
            {
                action_start_tick = now;
                robotState = STATE_WRONG_COLOR_TURN_RIGHT;
            }
            break;

        case STATE_WRONG_COLOR_TURN_RIGHT:
            Motor_Run255(AVOID_TURN_FAST_255, AVOID_TURN_REVERSE_255);
            if ((now - action_start_tick) >= AVOID_TURN_RIGHT_MS)
            {
                action_start_tick = now;
                robotState = STATE_WRONG_COLOR_FORWARD;
            }
            break;

        case STATE_WRONG_COLOR_FORWARD:
            Motor_Run255(AVOID_FORWARD_SPEED_255, AVOID_FORWARD_SPEED_255);
            if ((now - action_start_tick) >= AVOID_FORWARD_MS)
            {
                action_start_tick = now;
                robotState = STATE_WRONG_COLOR_TURN_LEFT;
            }
            break;

        case STATE_WRONG_COLOR_TURN_LEFT:
            Motor_Run255(AVOID_TURN_REVERSE_255, AVOID_TURN_FAST_255);
            if ((now - action_start_tick) >= AVOID_TURN_LEFT_MS)
            {
                Line5_ResetMemory();
                action_start_tick = now;
                obstacle_ignore_until = now + WRONG_COLOR_IGNORE_MS;
                robotState = STATE_WRONG_COLOR_SEARCH_LINE;
            }
            break;

        case STATE_WRONG_COLOR_SEARCH_LINE:
            /* Keep searching instead of stopping. Line5_Task() now performs
             * multi-direction lost-line recovery when all sensors are off line.
             */
            Line5_SetSpeed(LINE5_CARRY_SPEED_255);
            Line5_Task();
            if (Line5_ReadMask() != 0U)
            {
                obstacle_ignore_until = now + WRONG_COLOR_IGNORE_MS;
                robotState = avoid_return_state;
            }
            break;

        case STATE_DROP_SERVO1_DOWN:
            Motor_AllStop();
            if (Servo_WaitDone(SERVO_STEP_TIMEOUT_MS) != 0U)
            {
                action_start_tick = now;
                robotState = STATE_DROP_SERVO1_DOWN_HOLD;
            }
            break;

        case STATE_DROP_SERVO1_DOWN_HOLD:
            Motor_AllStop();
            if ((now - action_start_tick) >= SERVO_HOLD_AFTER_MOVE_MS)
            {
                Servo_CommandAngle(&servo2, SERVO2_DEFAULT_ANGLE);
                robotState = STATE_DROP_SERVO2_OPEN;
            }
            break;

        case STATE_DROP_SERVO2_OPEN:
            Motor_AllStop();
            if (Servo_WaitDone(SERVO_STEP_TIMEOUT_MS) != 0U)
            {
                action_start_tick = now;
                robotState = STATE_DROP_SERVO2_OPEN_HOLD;
            }
            break;

        case STATE_DROP_SERVO2_OPEN_HOLD:
            Motor_AllStop();
            if ((now - action_start_tick) >= SERVO_GRIP_HOLD_MS)
            {
                Servo_CommandAngle(&servo1, SERVO1_DEFAULT_ANGLE);
                robotState = STATE_DROP_SERVO1_UP;
            }
            break;

        case STATE_DROP_SERVO1_UP:
            Motor_AllStop();
            if (Servo_WaitDone(SERVO_STEP_TIMEOUT_MS) != 0U)
            {
                ServoNB_WriteAngle(&servo1, SERVO1_DEFAULT_ANGLE);
                ServoNB_WriteAngle(&servo2, SERVO2_DEFAULT_ANGLE);
                finish_marker_count = 0U;
                Line5_ResetMemory();
                obstacle_ignore_until = now + OBSTACLE_IGNORE_AFTER_PICK_MS;
                robotState = STATE_FINISH_RUN;
            }
            break;

        case STATE_FINISH_RUN:
            /* Van uu tien sieu am khi chay ve cuoi hanh trinh. */
            if (Distance_UpdateSpeedAndCheckStop(now, LINE5_BASE_SPEED_255) != 0U)
            {
                Motor_AllStop();
                robotState = STATE_STOP_END;
                break;
            }

            if (Line5_FinishMarkerTask() != 0U)
            {
                robotState = STATE_STOP_END;
                break;
            }
            Line5_Task();
            break;

        case STATE_STOP_END:
        default:
            Motor_AllStop();
            break;
    }
}

int main(void)
{
    uint32_t last_task_tick = 0U;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();

    /* PA2..PA6 are digital line inputs. Do not call MX_ADC1_Init(). */
    Line5_GPIO_Init();

    PWM_Start_All();
    Motor_AllStop();

    HCSR04_App_Init();
    TCS34725_App_Init();
    Servo_App_Init();

    HAL_Delay(1000U);

    Line5_ResetMemory();
    Line5_SetSpeed(LINE5_BASE_SPEED_255);
    target_color = TCS34725_COLOR_NONE;
    has_target_color = 0U;
    obstacle_ignore_until = 0U;
    robotState = STATE_LINE_TO_PICK;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if ((now - last_task_tick) >= LINE5_TASK_PERIOD_MS)
        {
            last_task_tick = now;
            Robot_Main_Task();
        }
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    HCSR04_TIM_IC_Callback(&hcsr04, htim);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_tx_busy = 0U;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uart1_tx_busy = 0U;
    }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    TCS34725_I2C_MemTxCpltCallback(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    TCS34725_I2C_MemRxCpltCallback(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    TCS34725_I2C_ErrorCallback(hi2c);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
    TCS34725_I2C_AbortCpltCallback(hi2c);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */

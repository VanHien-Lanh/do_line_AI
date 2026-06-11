# do_line_AI - FINAL FULL ROBOT FIRMWARE

This package is based on the Stage 2 firmware where the 5-channel TCRT5000 line following was reported stable. The remaining peripherals have been integrated back in:

- HC-SR04 ultrasonic distance sensor
- TCS34725 color sensor
- Servo1 and Servo2 gripper
- L298N motor control
- End-of-route finish marker stop logic
- Wrong-color right-side avoidance + line recovery

## Pin map kept from the project

No functional pin conflict was found in the firmware pin plan.

### Motor L298N

- Right ENA: PA0 / TIM2_CH1
- Left ENB: PA1 / TIM2_CH2
- Right IN1: PB10
- Right IN2: PB11
- Left IN3: PB12
- Left IN4: PB13

### 5-channel TCRT5000 line sensor

- S1 / OUT1 left-most: PA2
- S2 / OUT2: PA3
- S3 / OUT3 center: PA4
- S4 / OUT4: PA5
- S5 / OUT5 right-most: PA6

`LINE5_ACTIVE_LOW = 1` by default. If your module outputs HIGH on black line, change it to 0 in `Core/Src/main.c`.

### HC-SR04

- TRIG: PB5
- ECHO: PB4 / TIM3_CH1

Important hardware note: most HC-SR04 modules output 5V on ECHO. STM32F103 input is 3.3V logic, so use a voltage divider or level shifter on ECHO.

### TCS34725

- SCL: PB8 / I2C1 remap
- SDA: PB9 / I2C1 remap

Use 3.3V pull-ups on I2C. Most TCS34725 breakout boards already include pull-ups.

### Servos

- Servo1: PA9 / TIM1_CH2
- Servo2: PA8 / TIM1_CH1

Servo logic:

- Servo1 default: 0 deg
- Servo1 grab/drop down: 90 deg
- Servo2 open: 100 deg
- Servo2 close/grip: 40 deg

## Main behavior

1. Follow line normally.
2. When HC-SR04 distance is <= 10 cm, the robot slows down smoothly.
3. When distance is <= 3 cm, the robot stops immediately.
4. The TCS34725 waits 800 ms after stopping, then reads 12 samples at 150 ms spacing. It accepts the color only if at least 8 of 12 votes agree.
5. The robot saves the first object color, then grabs it slowly:
   - Servo1 -> 90 deg
   - Servo2 -> 40 deg
   - Servo1 -> 0 deg
6. It follows line while carrying.
7. At the next obstacle, it repeats the slow/stop/color read logic.
8. If target color matches: it drops the object:
   - Servo1 -> 90 deg
   - Servo2 -> 100 deg
   - Servo1 -> 0 deg
9. After dropping, it follows line until it sees a finish marker.
10. If target color is wrong: it performs a right-side avoidance maneuver and then searches/rejoins the line.

## Finish marker

The firmware stops after dropping the object only when all 5 line sensors see black for about 350 ms.

- Finish marker mask: `0x1F`
- Config: `FINISH_MARKER_ENABLE`, `FINISH_MARKER_MASK`, `FINISH_MARKER_HOLD_MS`

If you do not have a final wide black marker, the robot will keep following the line after dropping. You can either add the marker or set `FINISH_MARKER_ENABLE` to 0 and handle stop manually.

## Speed tuning

The base speed is now:

```c
#define LINE5_BASE_SPEED_255  90
#define LINE5_CARRY_SPEED_255 80
```

If the line following becomes unstable after adding the payload, reduce:

```c
LINE5_BASE_SPEED_255  -> 85 or 80
LINE5_CARRY_SPEED_255 -> 75 or 70
```

Increase speed only after color/ultrasonic/servo behavior is stable.

## Important debug variables for Keil Watch

Line:

- `dbg_line5_mask`
- `dbg_line5_error`
- `dbg_line5_correction`
- `dbg_line5_remember`
- `dbg_line5_lost_count`
- `dbg_line5_speed_cmd`

Ultrasonic:

- `dbg_distance_cm`
- `dbg_distance_age_ms`
- `dbg_distance_fresh`

Color:

- `dbg_tcs_status`
- `dbg_tcs_color`
- `dbg_tcs_c`
- `dbg_tcs_r`
- `dbg_tcs_g`
- `dbg_tcs_b`
- `dbg_color_sample_count`
- `dbg_color_vote_red`
- `dbg_color_vote_green`
- `dbg_color_vote_blue`
- `dbg_color_vote_yellow`

Robot:

- `dbg_robot_state`
- `dbg_has_target_color`
- `dbg_target_color`
- `dbg_finish_marker_count`

## UART debug

`ROBOT_UART_DEBUG = 1` by default. USART1 is remapped to PB6/PB7 at 115200 baud.

The UART prints a compact status every 500 ms:

```text
S=<state> D=<distance> TCS=<status> Col=<driver color> C=<clear> R=<red> G=<green> B=<blue>
```

When the robot is fully stable, you may set:

```c
#define ROBOT_UART_DEBUG 0U
```

to reduce background interrupt traffic.

## Notes about do_line_AI.ioc

The Keil project and firmware are ready to rebuild. The `.ioc` file was originally generated for the analog 8-sensor project, so if you open CubeMX and regenerate code, you should manually set PA2..PA6 as GPIO input for the 5-channel TCRT5000 version and avoid re-enabling `MX_ADC1_Init()` in `main.c`.

Do not call `MX_ADC1_Init()` in this 5-channel digital line version.

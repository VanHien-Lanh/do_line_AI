Ban nay sua dung theo huong ban yeu cau:
- Giu TIM2_CH1 = PA0 cho PWM motor phai ENA.
- Giu TIM2_CH2 = PA1 cho PWM motor trai ENB.
- Cam bien line analog 8 mat dung ADC1:
  S1 -> PA2 / ADC1_IN2
  S2 -> PA3 / ADC1_IN3
  S3 -> PA4 / ADC1_IN4
  S4 -> PA5 / ADC1_IN5
  S5 -> PA6 / ADC1_IN6
  S6 -> PA7 / ADC1_IN7
  S7 -> PB0 / ADC1_IN8
  S8 -> PB1 / ADC1_IN9
- Doi chan dieu khien chieu L298N sang:
  IN1 right -> PB10 / IN_1_R
  IN2 right -> PB11 / IN_2_R
  IN3 left  -> PB12 / IN_1_L
  IN4 left  -> PB13 / IN_2_L

Cach dung:
1. Mo file do_line_AI_3_8mat_analog_TIM2_PA0_PA1.ioc trong CubeMX/CubeIDE.
2. Generate Code. Sau generate phai co Core/Src/adc.c va Core/Inc/adc.h.
3. Thay Core/Src/main.c bang main_8mat_analog_TIM2_PA0_PA1.c va doi ten thanh main.c.
4. Dau day dung bang tren.
5. Khi bat nguon, trong 2.5 giay dau dua thanh cam bien qua nen trang va line den de tu calib.

Neu robot re nguoc: doi STEERING_SIGN tu 1 thanh -1.
Neu robot hieu nguoc trang/den: doi LINE_BLACK_LOW_ADC tu 1 thanh 0.
Neu robot lac manh: giam LINE_KP.
Neu robot cua yeu: tang LINE_KP.

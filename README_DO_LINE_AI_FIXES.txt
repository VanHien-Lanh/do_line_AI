DO_LINE_AI - GIAI DOAN 1 VA 2

Trong thu muc dinh kem co 2 ban project rieng:

1) do_line_AI_STAGE1_8ANALOG_FIXED
   - Giu cam bien line analog 8 mat nhu project goc.
   - Da sua PWM motor TIM2 len 1kHz, bat lai task sieu am/mau, sua state machine, toi uu bam line.
   - Mo file Keil: MDK-ARM/do_line_AI.uvprojx
   - Build lai de tao HEX moi. Khong dung HEX cu trong project goc.

2) do_line_AI_STAGE2_5TCRT5000_LINE_ONLY
   - Test huong moi voi module TCRT5000 5 kenh digital.
   - Ngoai vi khac van giu trong project, nhung firmware nay chi test line + motor.
   - Ket noi OUT1..OUT5 vao PA2..PA6.
   - Mo file Keil: MDK-ARM/do_line_AI.uvprojx
   - Build lai de tao HEX moi.

Thu tu test khuyen nghi:
1. Nap Stage2 hoac Stage1 line-only truoc de kiem tra huong motor va line.
2. De xe tren vach, mo Keil Watch xem bien debug:
   - Stage1: dbg_sensor_mask, dbg_last_pos, dbg_servo_pwm, dbg_sensor_value[8]
   - Stage2: dbg_line5_mask, dbg_line5_error, dbg_line5_correction
3. Neu xe quay nguoc huong: doi RIGHT_MOTOR_INVERT hoac LEFT_MOTOR_INVERT.
4. Neu sensor nhan nguoc trang/den:
   - Stage1: doi LINE_ADC_INVERT.
   - Stage2: doi LINE5_ACTIVE_LOW.
5. Khi line on dinh moi ghep tiep sieu am, mau, servo.

DO_LINE_AI_STAGE2_5TCRT5000_LINE_ONLY

Ban nay dung de test huong cam bien TCRT5000 5 kenh digital nhu hinh.
Ngoai vi khac van giu nguyen trong project, nhung firmware nay chi chay motor + line sensor de test bam line.

Ket noi TCRT5000 5 kenh:
- OUT1/S1 trai nhat  -> PA2
- OUT2/S2            -> PA3
- OUT3/S3 giua       -> PA4
- OUT4/S4            -> PA5
- OUT5/S5 phai nhat  -> PA6
- VCC: uu tien 3.3V neu module ho tro; neu cap 5V hay do chan OUT khong vuot 3.3V truoc khi noi STM32.
- GND: chung GND voi STM32/L298N.

Luu y quan trong:
- Code khong goi MX_ADC1_Init vi PA2..PA6 duoc dung lam GPIO input.
- Line5_GPIO_Init() cau hinh PA2..PA6 thanh input pull-up.
- Mac dinh LINE5_ACTIVE_LOW=1 vi nhieu board TCRT5000/LM393 keo LOW khi gap vach den.
  Neu Keil Watch dbg_line5_mask khong doi khi dua vao vach den, doi LINE5_ACTIVE_LOW=0.
- Can chinh bien tro tren module: tren nen trang output phai khac khi gap vach den.

Thong so tune nhanh:
- Xe roi line: giam LINE5_BASE_SPEED_255 85 -> 70 hoac tang LINE5_KP_NUM 60 -> 70.
- Xe lac/zigzag: giam LINE5_KP_NUM 60 -> 45 hoac giam LINE5_KD_NUM 35 -> 20.
- Xe quay tim line sai huong: dao thu tu OUT1..OUT5 hoac doi logic line5_remember trong Line5_Task.
- Motor quay nguoc: doi RIGHT_MOTOR_INVERT/LEFT_MOTOR_INVERT.

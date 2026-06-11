# do_line_AI - Stage 2 Full Robot Priority Recovery V5

Ban nay uu tien theo yeu cau moi:

1. Do line va HC-SR04 duoc cap nhat lien tuc trong vong lap chinh.
2. HC-SR04 duoc kiem tra truoc khi Line5_Task ra lenh motor trong cac trang thai chay line.
3. Gap vat <= 10 cm: xe giam toc tu tu.
4. Gap vat <= 3 cm: xe dung ngay de doc mau/ra quyet dinh.
5. Neu doc mau 10 giay khong hop le:
   - Neu sieu am khong con thay vat gan: xe bo qua trong khoang ngan va tiep tuc do line.
   - Neu vat van nam trong vung <= 10 cm: xe ne phai, tim lai line, roi tiep tuc.
6. Mat line: xe khong dung im. Xe lui ngan ve vung vua mat line, quay ve huong line vua thay, sau do quet trai/phai cham.
7. Toc do da giam de xe bam line on dinh hon:
   - LINE5_BASE_SPEED_255 = 75
   - LINE5_CARRY_SPEED_255 = 65
   - LINE5_MIN_CURVE_SPEED_255 = 40
8. UART debug mac dinh tat de xe chay muot hon. Muon xem debug thi doi ROBOT_UART_DEBUG = 1U.

Bien debug nen xem trong Keil Watch:
- dbg_line5_mask
- dbg_line5_error
- dbg_line5_correction
- dbg_line5_remember
- dbg_line5_lost_count
- dbg_line5_search_phase
- dbg_line5_speed_cmd
- dbg_distance_cm
- dbg_distance_age_ms
- dbg_distance_fresh
- dbg_robot_state
- dbg_color_sample_count
- dbg_tcs_status
- dbg_tcs_c, dbg_tcs_r, dbg_tcs_g, dbg_tcs_b

Giai thich dbg_line5_search_phase:
0 = dang thay line / PID binh thuong
1 = lui ve vung line vua mat
2 = quay ve huong line vua mat
3 = quet theo huong uu tien
4 = quet nguoc lai
5 = tien rat cham
6 = lui nhe de mo rong vung tim

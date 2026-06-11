# V7 - Center 360 Lost-Line Recovery

Ban nay duoc phat trien tu V6 Fast Line Recovery.

## Thay doi chinh

Khi xe mat line trong luc line vua nam gan giua/giua cam bien, xe se:

1. Quay 360 do de tim lai line.
2. Neu chua bat lai line, di thang ngan.
3. Sau do quay ve chu ky tim line cu: quet huong uu tien, quet nguoc lai, tien cham, lui nhe.

Neu xe mat line o mep trai/phai, xe van dung thuat toan V6: lui nhanh ve line vua mat, quay ve huong vua mat, sau do quet trai/phai.

## Thong so can tune neu thuc te chua dung 360 do

Trong `Core/Src/main.c`:

```c
#define LINE5_CENTER_SPIN_TURN_255       ((int16_t)115)
#define LINE5_CENTER_SPIN_REVERSE_255    ((int16_t)-75)
#define LINE5_CENTER_SPIN_COUNT          720U
#define LINE5_CENTER_FORWARD_255         ((int16_t)75)
#define LINE5_CENTER_FORWARD_COUNT       150U
```

- Neu xe quay chua du 360 do: tang `LINE5_CENTER_SPIN_COUNT`.
- Neu xe quay qua 360 do: giam `LINE5_CENTER_SPIN_COUNT`.
- Neu mat san/pin yeu lam xe quay cham: tang `LINE5_CENTER_SPIN_TURN_255` len 120 hoac 125.

## Debug trong Keil Watch

```c
dbg_line5_search_phase
```

Gia tri:

- 0: dang thay line, PID binh thuong.
- 1: lui nhanh ve line vua mat.
- 2: quay nhanh ve huong line vua mat.
- 3: quet manh theo huong uu tien.
- 4: quet manh nguoc lai.
- 5: tien cham.
- 6: lui nhe.
- 7: mat line gan giua, dang quay 360 do.
- 8: sau quay 360 do, dang di thang ngan.

Cac bien nen xem them:

```c
dbg_line5_mask
dbg_line5_last_mask
dbg_line5_recovery_dir
dbg_line5_center_lost
dbg_line5_lost_count
```

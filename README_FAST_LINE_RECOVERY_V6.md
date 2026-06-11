# FAST LINE RECOVERY V6

Ban nay cai thien rieng phan mat line cua ban V5.

## Muc tieu

- Khi mat line, xe khong dung im.
- Xe lui ngan de quay ve vung line vua mat.
- Xe quay nhanh ve huong line duoc nho gan nhat.
- Neu chua thay line, xe quet trai/phai manh hon voi toc do gan toc do bo cua binh thuong.
- Van giu toc do bam line chinh o muc on dinh: BASE=75, CARRY=65.

## Thay doi chinh trong Core/Src/main.c

### 1. Toc do tim line tang len

- LINE5_LOST_BACKTRACK_SPEED_255 = -62
- LINE5_LOST_RETURN_TURN_255 = 110
- LINE5_LOST_RETURN_REVERSE_255 = -70
- LINE5_LOST_SWEEP_TURN_255 = 105
- LINE5_LOST_SWEEP_REVERSE_255 = -65
- LINE5_LOST_FORWARD_255 = 58
- LINE5_LOST_BACKUP_255 = -55

### 2. Them nho line manh hon

- line5_last_mask: luu mask gan nhat con thay line.
- line5_recovery_dir: chot huong tim line ngay luc vua mat line.
- dbg_line5_last_mask: bien debug.
- dbg_line5_recovery_dir: bien debug.

### 3. Logic mat line moi

- Phase 1: lui nhanh ve vung line vua mat.
- Phase 2: quay nhanh ve huong line vua mat.
- Phase 3: quet manh theo huong uu tien.
- Phase 4: quet manh nguoc lai.
- Phase 5: tien cham de bat lai line.
- Phase 6: lui nhe roi lap chu ky.

## Bien debug nen xem trong Keil Watch

- dbg_line5_mask
- dbg_line5_last_mask
- dbg_line5_recovery_dir
- dbg_line5_search_phase
- dbg_line5_lost_count
- dbg_line5_remember
- dbg_line5_error
- dbg_line5_correction


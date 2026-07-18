# Astra — Báo cáo tầm depth & FOV (T0 spike)

**Đo lúc:** 2026-06-25 17:04:48
**Chế độ:** probe · 30 frame

## KẾT LUẬN: `[INCONCLUSIVE]` CHƯA KẾT LUẬN — cảnh quá gần

> 95% pixel < 105mm (median 48mm) → camera đang nhìn vật RẤT GẦN / bị che / úp xuống. Không thể đánh giá tầm xa của cảm biến. HÃY hướng camera ra cảnh XA & THOÁNG nhất (hành lang, phòng rộng) rồi CHẠY LẠI.

## Số liệu

| Chỉ số | Giá trị |
|--------|---------|
| Frame / FPS | 30 / 21.9 |
| Kích thước frame | 640x480 (WxH) |
| %pixel hợp lệ | 100.0% |
| FOV ngang / dọc (hình học) | 58.6° / 45.6° |
| Độ phủ ngang hữu dụng | 100% số cột |
| min depth | 20 mm |
| p50 (median) | 48 mm |
| p95 | 105 mm |
| **p99 (tầm max robust)** | **213 mm** |
| p99.9 | 486 mm |
| max | 1022 mm |

## Histogram (mm)

```
      0-  500mm | ██████████████████████████████████████████████ 9207384
    500- 1000mm |  7637
   1000- 1500mm |  979
```

## Ý nghĩa cho PLAN_SLAM_NAV2.md

- `[A]` tầm ≥3m → tiếp tục T1+ (slam_toolbox + NAV2 trên Astra depth).
- `[MARGINAL]` 1.5–3m → chỉ khu vực nhỏ, rủi ro map trôi; cân nhắc LiDAR.
- `[B]` <1.5m hoặc cap ~1022mm → DỪNG, đề xuất LiDAR 2D trước khi đầu tư stack.

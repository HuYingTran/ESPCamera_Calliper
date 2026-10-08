# ESP32-CAM Calliper

Firmware cho **ESP32-CAM (AI-Thinker)** biến board thành một thiết bị đo lường
có camera: vừa **stream video** lên web, vừa **điều khiển servo**, vừa **đọc
giá trị thước kẹp điện tử (digital caliper)** — tất cả qua một giao diện web tối,
responsive, không cần cài app.

Board tự phát WiFi (AP) nên dùng được ngay tại chỗ, đồng thời có thể kết nối
vào mạng WiFi có sẵn (STA) để truy cập từ xa — cấu hình hoàn toàn trên web.

## Tính năng

| Thành phần | Mô tả |
|---|---|
| **WiFi** | Chạy đồng thời **AP + STA**: tự phát sóng để điều khiển tại chỗ, song song kết nối mạng ngoài. SSID/mật khẩu mạng ngoài lưu vào NVS. |
| **Webserver** | `esp_http_server` — trang điều khiển + API ở cổng **80**, luồng video ở cổng **81**. |
| **Video** | Stream **MJPEG** (`/stream`), VGA 640×480 dùng PSRAM, **xoay 180°** (`vflip` + `hmirror`). Chụp 1 ảnh JPEG qua `/capture`. |
| **Servo** | PWM 50 Hz (LEDC), điều khiển bằng slider trên web → `/servo?angle=0..180`. |
| **Thước kẹp** | Đọc gói dữ liệu 24-bit qua ngắt cạnh clock, trả về giá trị mm/inch. |
| **Đèn flash** | Bật/tắt LED flash on-board (GPIO 4) qua `/light`. |
| **Giao diện** | Web UI tối, thẻ kính mờ, slider gradient, chỉ báo LIVE; responsive cho điện thoại. |

## Sơ đồ chân

Ngoài các chân camera cố định của AI-Thinker ESP32-CAM:

| Tín hiệu | GPIO |
|---|---|
| Servo signal | 12 |
| Caliper CLK (ngắt) | 13 |
| Caliper DATA | 15 |
| LED flash on-board | 4 |

## HTTP endpoints

| Endpoint | Cổng | Chức năng |
|---|---|---|
| `/` | 80 | Trang web điều khiển |
| `/servo?angle=0..180` | 80 | Đặt góc servo, trả JSON |
| `/caliper` | 80 | Giá trị thước kẹp: `{valid, value, unit}` |
| `/capture` | 80 | Chụp & tải về 1 ảnh JPEG |
| `/light?on=0\|1` | 80 | Bật/tắt đèn LED flash |
| `/scan` | 80 | Quét các mạng WiFi xung quanh (mảng JSON) |
| `/connect?ssid=&pass=` | 80 | Kết nối mạng ngoài (lưu NVS), trả IP |
| `/wifi` | 80 | Trạng thái STA: `{connected, ssid, ip}` |
| `/switch` | 80 | Tắt AP, chỉ giữ STA để chuyển sang IP mạng ngoài |
| `/stream` | 81 | Luồng video MJPEG |

## Cấu trúc dự án

```
ESPCamera_Calliper/
├── platformio.ini      # cấu hình board, PSRAM, partition, tốc độ nạp
├── src/main.cpp        # toàn bộ firmware (camera, web, servo, caliper, WiFi)
├── include/ lib/ test/ # chuẩn PlatformIO (hiện để trống)
└── README.md
```

## Build & nạp

Yêu cầu: [PlatformIO](https://platformio.org/) và một module **FTDI/USB-UART**
để nạp (ESP32-CAM không có cổng USB).

```bash
pio run              # biên dịch
pio run -t upload    # nạp firmware
pio device monitor   # xem log (115200 baud)
```

Sau khi nạp xong:

1. Kết nối WiFi **`ESP32CAM-Calliper`** — mật khẩu **`12345678`**.
2. Mở trình duyệt vào **http://192.168.4.1/**
3. (Tùy chọn) Dùng mục cấu hình WiFi trên web để kết nối board vào mạng có sẵn
   rồi bấm **Switch** để truy cập qua IP mạng ngoài.

## Cấu hình

Sửa các hằng số đầu `src/main.cpp`:

```cpp
static const char *AP_SSID = "ESP32CAM-Calliper";
static const char *AP_PASS = "12345678";        // >= 8 ký tự
#define SERVO_PIN             12
#define CALIPER_CLK_PIN       13
#define CALIPER_DATA_PIN      15
#define CALIPER_DATA_INVERTED false              // true nếu đọc ra bit bị đảo
#define LIGHT_PIN             4
```

**Xoay hình camera**: trong `cameraInit()`, `set_vflip` + `set_hmirror` cùng đặt
`1` để xoay 180°; đặt cả hai về `0` để tắt. OV2640 không xoay 90°/270° bằng phần
cứng — nếu cần, xoay phía client bằng CSS.

## ⚠️ Lưu ý phần cứng

1. **Thước kẹp giao tiếp ở mức ~1.5 V** — **KHÔNG** nối trực tiếp vào GPIO 3.3 V.
   Cần **mạch dịch mức** (level shifter / transistor) để tránh làm hỏng chân IO.
2. **Giao thức thước kẹp khác nhau tùy model.** Code dùng chuẩn phổ biến của
   thước giá rẻ (gói 24-bit, LSB trước, bit 20 = dấu, bit 23 = inch, đơn vị
   0.01 mm). Nếu đọc sai/đảo bit: bật `CALIPER_DATA_INVERTED` hoặc chỉnh layout
   bit trong `caliperPoll()`.
3. **Nguồn**: camera + WiFi tiêu thụ dòng lớn. Dùng nguồn **5 V** đủ khỏe và cáp
   tốt để tránh reset giữa chừng.

## Khắc phục lỗi nạp firmware

ESP32-CAM nạp qua FTDI nên dễ gặp `The chip stopped responding`:

- Cấp **nguồn 5 V** khỏe, dùng cáp tốt.
- Nối **GPIO 0 → GND** khi nạp (vào chế độ bootloader), tháo ra khi chạy.
- Hạ `upload_speed` trong `platformio.ini` (mặc định `460800`, có thể hạ xuống
  `115200`).
- Khi màn hình báo `Connecting...` thì nhấn nút **RESET** trên board.

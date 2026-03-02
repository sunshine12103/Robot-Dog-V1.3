# SpotMicro Robot Dog — Hướng dẫn cài đặt

## 1. Yêu cầu hệ thống
- Windows 10/11
- Python 3.10+
- ESP32-S3 đã flash MicroPython v1.27+

---

## 2. Cài Python

Tải tại: https://www.python.org/downloads/

> ⚠️ Tick **"Add Python to PATH"** khi cài đặt!

---

## 3. Cài thư viện Python

Mở PowerShell, chạy:

```powershell
pip install pyserial mpremote
```

---

## 4. Cài driver USB (nếu ESP32 không nhận)

Tải CH340 driver: https://www.wch-ic.com/downloads/CH341SER_EXE.html

Sau khi cài, cắm ESP32 → kiểm tra **Device Manager** để biết COM port (ví dụ: `COM9`).

---

## 5. Upload firmware lên ESP32

> Chỉ cần làm **1 lần** hoặc khi có cập nhật firmware.

Mở PowerShell tại thư mục dự án:

```powershell
cd C:\...\Robot-Dog-V1.3
```

Upload từng file:

```powershell
python -m mpremote connect COM9 cp micropython/pca9685.py :pca9685.py
python -m mpremote connect COM9 cp micropython/profiler.py :profiler.py
python -m mpremote connect COM9 cp micropython/state.py :state.py
python -m mpremote connect COM9 cp micropython/crawl.py :crawl.py
python -m mpremote connect COM9 cp micropython/esp32_robot_receiver.py :main.py
python -m mpremote connect COM9 reset
```

> Thay `COM9` bằng COM port thực của mày.

Kiểm tra xem board đã boot thành công chưa:

```powershell
python -m mpremote connect COM9 repl
```

Nếu thấy `READY` và `WIFI IP: ...` là OK. Nhấn `Ctrl+]` để thoát.

---

## 6. Chạy Dashboard

```powershell
python robot_dashboard.py
```

### Kết nối Serial (cần cắm dây):
- Chọn **Serial**
- Chọn COM port → nhấn **Connect**

### Kết nối WiFi (không cần dây):
- Bật hotspot: SSID `Tuan Kiet` / Pass `Kiet0708`
- Chọn **WiFi**
- Nhập IP (xem trong repl sau khi boot) và Port `8888`
- Nhấn **Connect**

---

## 7. Các file firmware cần thiết trên ESP32

| File trên máy tính | Upload thành | Mô tả |
|---|---|---|
| `micropython/pca9685.py` | `pca9685.py` | Driver servo PCA9685 |
| `micropython/profiler.py` | `profiler.py` | Motion profiler / IK |
| `micropython/state.py` | `state.py` | State machine |
| `micropython/crawl.py` | `crawl.py` | Crawl gait algorithm |
| `micropython/esp32_robot_receiver.py` | `main.py` | Main firmware (chạy khi boot) |

---

## 8. Cấu hình WiFi

Nếu muốn đổi WiFi, sửa trong `micropython/esp32_robot_receiver.py`:

```python
WIFI_SSID = "Tuan Kiet"
WIFI_PASS = "Kiet0708"
TCP_PORT  = 8888
```

Sau đó upload lại `main.py`.

---

## 9. Ghi chú

- Mỗi khi thay đổi code `esp32_robot_receiver.py` hoặc `crawl.py`, phải upload lại lên ESP32
- Nếu mpremote báo `Up to date`, xóa file trên ESP32 trước:
  ```powershell
  python -m mpremote connect COM9 exec "import os; os.remove('main.py')" + cp micropython/esp32_robot_receiver.py :main.py + reset
  ```

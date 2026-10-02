# Host test — scale_parse

Test parser cân chạy trên PC, không cần ESP-IDF.

```bash
gcc -Wall -I../../components/scale_serial test_scale_parse.c \
    ../../components/scale_serial/scale_parse.c -o test_scale_parse
./test_scale_parse
```

Máy này chưa có gcc — cài qua một trong các cách: `winget install MSYS2.MSYS2`
(rồi `pacman -S mingw-w64-ucrt-x86_64-gcc`), hoặc WSL, hoặc dùng compiler đi kèm
bất kỳ. Logic thuật toán đã được kiểm chứng bằng bản port Python 1:1 (12/12 PASS,
xem lịch sử dự án).

**Quy tắc:** mỗi khi bắt được dòng dữ liệu THẬT từ cân của khách bằng
`tools/capture_scale.py`, thêm dòng đó vào `test_scale_parse.c` làm test case
mới trước khi tin parser.

# Host test — cmd_auth (lệnh MQTT ký HMAC + dedup)

Phần thuần C của đường lệnh downlink (`components/mqtt_link/cmd_auth.c`): cắt
`sig` khỏi gói canonical, escape JSON kiểu Python, dựng ack, cửa sổ dedup, kiểm
`ts`. Chạy từ thư mục gốc `esp32/`:

```bash
gcc -Wall -Wextra -I components/mqtt_link tests/host/test_cmd_auth.c \
    components/mqtt_link/cmd_auth.c -o /tmp/test_cmd_auth
/tmp/test_cmd_auth; echo "exit=$?"
```

Vector canonical/HMAC trong test sinh THẬT bằng Python (`json.dumps(sort_keys=True,
separators=(",",":"))` + `hmac.sha256` key `"k3y"`), y như
`edge_collector/manager.py::queue_command`. Đổi format phía Python → sinh lại vector.
HMAC thật (mbedtls) và `mqtt_link.c` (cJSON/esp-mqtt) không test được ở đây.

**Quy tắc:** bắt được gói lệnh THẬT từ edge (`mosquitto_sub` trên topic lệnh của node, mặc định dạng `fms/<serial>/cmd`)
→ thêm vào `SIG_VECTORS` trong `test_cmd_auth.c` trước khi tin parser.

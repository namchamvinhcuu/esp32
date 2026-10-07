# Host test — scale_parse

Test parser cân chạy trên PC, không cần ESP-IDF.

Chạy từ thư mục gốc `esp32/` (máy Linux dev có sẵn `gcc`):

```bash
gcc -Wall -I components/scale_serial tests/host/test_scale_parse.c \
    components/scale_serial/scale_parse.c -o /tmp/test_scale_parse
/tmp/test_scale_parse; echo "exit=$?"
```

Máy Windows chưa có gcc thì cài MSYS2 (`winget install MSYS2.MSYS2`, rồi
`pacman -S mingw-w64-ucrt-x86_64-gcc`) hoặc dùng WSL.

**Quy tắc:** mỗi khi bắt được dòng dữ liệu THẬT từ cân của khách (đọc thô cổng
serial bằng bất kỳ công cụ nào, vd `idf.py monitor`, `minicom`, `pyserial`),
thêm dòng đó vào `test_scale_parse.c` làm test case mới trước khi tin parser.

# Host test — cmd_auth (lệnh MQTT ký HMAC + dedup)

Phần thuần C của đường lệnh downlink (`components/mqtt_link/cmd_auth.c`): cắt
`sig` khỏi gói canonical, escape JSON kiểu Python, dựng ack, cửa sổ dedup, kiểm
`ts`, chuyển `ms`/`period_ms` thành int32 (`cmd_duration_ms`), kiểm `id` lệnh (`cmd_id_from_double`, dùng chung cho MQTT lẫn HTTP poll). Chạy từ thư mục gốc `esp32/`:

```bash
gcc -Wall -Wextra -I components/mqtt_link tests/host/test_cmd_auth.c \
    components/mqtt_link/cmd_auth.c -o /tmp/test_cmd_auth
/tmp/test_cmd_auth; echo "exit=$?"
```

Chạy thêm bản UBSan để bắt UB khi ép double → số nguyên (vd NaN lọt qua guard —
bản thường vẫn PASS vì trên x86 kết quả ép NaN tình cờ vẫn bị từ chối).
`-fno-sanitize-recover=all` là bắt buộc: thiếu nó UBSan chỉ in lỗi ra stderr mà exit vẫn 0.

```bash
gcc -Wall -Wextra -I components/mqtt_link tests/host/test_cmd_auth.c \
    components/mqtt_link/cmd_auth.c \
    -fsanitize=undefined,float-cast-overflow -fno-sanitize-recover=all \
    -o /tmp/test_cmd_auth_ubsan && /tmp/test_cmd_auth_ubsan | tail -1
```

Vector canonical/HMAC trong test sinh THẬT bằng Python (`json.dumps(sort_keys=True,
separators=(",",":"))` + `hmac.sha256` key `"k3y"`), y như
`edge_collector/manager.py::queue_command`. Đổi format phía Python → sinh lại vector.
HMAC thật (mbedtls) và `mqtt_link.c` (cJSON/esp-mqtt) không test được ở đây.

**Quy tắc:** bắt được gói lệnh THẬT từ edge (`mosquitto_sub` trên topic lệnh của node, mặc định dạng `fms/<serial>/cmd`)
→ thêm vào `SIG_VECTORS` trong `test_cmd_auth.c` trước khi tin parser.

# FMS Node — ESP32-S3 Firmware

Firmware C/ESP-IDF (FreeRTOS) cho node giám sát nhà máy "FMS Node" — đọc cảm
biến (Modbus TCP/RTU, I2C, cân điện tử serial ASCII, GPIO input/output, BLE
central + peripheral) rồi đẩy dữ liệu lên **edge_collector** qua hợp đồng HTTP
`/node/v1/*`, cùng contract mà `node_agent` (Python, chạy trên Pi/PC) dùng.

Kiến trúc 1 chiều: **node → edge → Odoo**. Node không bao giờ nói thẳng Odoo.

- Target chip: `esp32s3`
- ESP-IDF: v5.2+/v6 (đã build-verify trên **v5.5.4**)
- Repo GitHub: `namchamvinhcuu/esp32`

## Build

### Máy Windows (build + flash board thật)

```
idf.py set-target esp32s3
idf.py build
idf.py -p COM3 flash monitor
```

### Máy Linux/macOS (build-only, không flash)

```bash
git clone -b v5.5.4 --recursive --shallow-submodules \
    https://github.com/espressif/esp-idf.git ~/esp/v5.5.4/esp-idf
cd ~/esp/v5.5.4/esp-idf && ./install.sh esp32s3
. ~/esp/v5.5.4/esp-idf/export.sh

cd <thư mục project này>
idf.py build
```

Không cắm board vào máy Linux/macOS thì bỏ qua `idf.py flash` — build thành
công là đủ để bắt lỗi compile/link sớm.

## Cấu trúc project

```
esp32/
  main/main.c                      # app_main() — khởi 18 component tuần tự + main loop 100ms
  components/
    cfg/                           # NVS config: creds, channels, api_key, boot counter — khởi ĐẦU TIÊN
    prov_portal/                   # SoftAP provisioning (node mới) + HTTP portal runtime + BOOT-button watch
    meas_core/                     # hàng đợi đo lường trung tâm — mọi nguồn cảm biến push vào đây
    net_mgr/                       # WiFi STA, RSSI, IP, đồng bộ giờ từ server
    spooler/                       # RAM spool — sống sót mất WiFi, KHÔNG sống sót reboot
    mb_tcp/                        # Modbus TCP polling
    mb_rtu/                        # Modbus RTU (RS485 onboard) — sensor + relay tower light
    i2c_sens/                      # cảm biến I2C
    io_scan/                       # quét GPIO input
    gpio_out/                      # relay/lamp output, điều khiển từ server
    scale_serial/ + scale_parse.c  # cân điện tử serial ASCII — scale_parse.c tách riêng (logic thuần, host-test được)
    uplink/                        # HTTP client lên edge_collector /node/v1/* — có Kconfig stub (tắt được, fallback MQTT)
    mqtt_link/                     # MQTT broker link — chạy song song HTTP, có Kconfig stub
    wdt_util/                      # helper dùng chung: sleep dài mà vẫn nuôi task watchdog
    ble_uart/                      # BLE peripheral — live JSON feed cho app đồng hành
    ble_central/                   # BLE central — kết nối thiết bị đo (caliper...)
    ble_print/                     # in qua BLE (máy in di động)
    tower_light/                   # đèn tháp qua relay — có Kconfig stub
    diag/                          # theo dõi task runtime/heap, log định kỳ
  tests/host/                      # host-test qua gcc thường, KHÔNG cần ESP-IDF (chỉ file logic thuần)
```

## Cấu hình (`idf.py menuconfig` → "FMS Node")

Các mục chính: WiFi SSID/pass (seed NVS lần boot đầu), `FMS_SERVER_URL` (base
URL của `edge_collector`), `FMS_API_KEY` (thường để trống — học tự động qua
`/node/v1/hello`), cấu hình UART/GPIO cho cân điện tử. Xem `main/Kconfig.projbuild`
và `Kconfig.projbuild` của từng component (`uplink`, `mqtt_link`) để biết đầy
đủ toggle.

## Test

Phần logic thuần (không phụ thuộc FreeRTOS/ESP-IDF) test được trực tiếp bằng
`gcc`, không cần toolchain ESP-IDF:

```bash
gcc -Wall -Icomponents/scale_serial tests/host/test_scale_parse.c \
    components/scale_serial/scale_parse.c -o /tmp/test_scale_parse
/tmp/test_scale_parse
```

17/18 component còn lại (task/GPIO/BLE/WiFi/Modbus thật) không unit-test có ý
nghĩa ngoài phần cứng — verify bằng review code + build thật + flash thiết bị.

## Ràng buộc quan trọng cần biết trước khi sửa code

- **Task watchdog toàn cục 30s** — sleep dài ở bất kỳ task nào phải dùng
  `wdt_util::wdt_safe_sleep_ms()` (chia bước ≤1s + reset định kỳ), không
  `vTaskDelay` một phát dài.
- **Heap ~48KB free, PSRAM tắt có chủ đích** — ưu tiên buffer tĩnh + `snprintf`
  (luôn check return value) thay vì `cJSON`/`malloc` trong hot path.
- **Kconfig `#if CONFIG_X_ENABLE` stub pattern** (`uplink.c`, `mqtt_link.c`,
  `tower_light.c`) — tắt flag ở menuconfig thì TOÀN BỘ file biến thành bộ stub;
  phải stub đủ mọi hàm public khai trong `.h`.
- **Parity với `node_agent`** (Python, project sibling) — 2 implementation độc
  lập của cùng client nói chuyện với `edge_collector` qua `/node/v1/*`. Sửa
  logic dedup lệnh / backoff / hello ở một bên mà không đối chiếu bên kia là
  bug cross-implementation khó tái hiện.
- **`check_config_update()` trong `uplink.c` cố ý bị disable** — không bật lại
  khi chưa có cách phân biệt phạm vi cấu hình (từng gây sự cố ghi đè cấu hình
  RS485 tay thật của khách).

Chi tiết đầy đủ + quy trình làm việc: xem `CLAUDE.md` và `.obsidian-vault/`
(`Architecture/Overview.md`, `Architecture/Code-Map.md`).

## Project liên quan

- `node_agent` (Python) — client thay thế chạy trên Pi/PC, cùng contract `/node/v1/*`.
- `edge_collector` — server sở hữu contract `/node/v1/*`, chuyển tiếp dữ liệu lên Odoo.

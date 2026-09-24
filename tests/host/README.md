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

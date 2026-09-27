# 01 · 天气时钟

OLED 桌面时钟：显示时间（NTP 网络对时）+ 深圳实时天气（和风天气 API）。
2026-09-25 ~ 26 分步做出来的第一个项目。

## 走过的路

一步一步加上去的（每一步的串口调试输出在本地 `logs/`，未上传）：

- I2C 扫描 → OLED 点亮（SSD1306，128x64）
- 连 WiFi → NTP 对时（时区 +8）
- HTTPS 请求和风天气 → JSON 解析 → gzip 解压（和风无视 Accept-Encoding 一直返回压缩数据，用 ROM 自带 miniz 解压）→ 32x32 天气图标位图显示
- 加固：非阻塞服务函数（serviceWiFi / serviceNtp / serviceWeather / serviceDisplay）、WiFi 断线自动重连、错误状态上屏（NO WIFI / retrying）

## 接线

| OLED | ESP32-S3 |
|---|---|
| SDA | GPIO8 |
| SCL | GPIO9 |
| VCC | 3.3V |
| GND | GND |

## 编译与上传

1. VS Code 打开本文件夹
2. 底部 PlatformIO 工具栏：Build（对勾）→ Upload（箭头）→ Serial Monitor（插头）
3. 串口固定为板载 FT232：`/dev/cu.usbserial-A5069RR4`（写在 platformio.ini 里；换电脑或板子要改）

## 密钥

复制 `src/secrets.h.example` 为 `src/secrets.h`，填入自己的 WiFi 和和风天气信息
（注册 https://console.qweather.com 免费拿到 Key 和专属 API Host）。

## 工具

- `tools/make_icons.py`：把天气图标转成 C 位图数组（生成 `src/weather_icons.h`）
- `capture.py`：复位板子并抓取 N 秒串口输出（调试用）

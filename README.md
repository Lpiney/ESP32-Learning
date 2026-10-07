# ESP32 自学记录

大一自学电子工程（EE）的学习仓库。每一课是一个独立的 PlatformIO 工程，按时间顺序编号。

## 学习进度

| 编号 | 项目 | 学到的内容 | 日期 |
|---|---|---|---|
| 01 | 天气时钟 | I2C、OLED、WiFi、NTP 对时、HTTPS、JSON、断线自恢复 | 2026-09-25 ~ 26 |
| 02 | 三色 LED | GPIO 输出、限流电阻、面包板接线、交通灯 & 流水灯 | 2026-09-27 |
| 03 | 摇杆示波器 | SPI OLED（7 脚）、ADC 读模拟量、中点校准、死区、EMA 滤波、双缓冲绘图 | 2026-10-03 |

## 硬件

- 主控：ESP32-S3-N16R8（16MB Flash + 8MB PSRAM，板载 FT232RL）
- 显示：0.96" OLED（SSD1306，I2C，地址 0x3C）、0.96" OLED（SSD1306，7 脚 SPI）
- 传感器 / 执行器：MPU-6050、SG90 舵机、PS2 摇杆、LED、轻触按键
- 面包板、杜邦线、金属膜电阻包

> ⚠️ N16R8 的 PSRAM 走 Octal 模式，**GPIO33 ~ GPIO37 被 PSRAM 占用**，
> 接线时绝对不要用这几个脚。

## 开发环境

- VS Code + PlatformIO IDE 扩展
- platform = espressif32@7.1.3，framework = arduino

用哪一课，就用 VS Code「打开文件夹」打开哪一课的目录，然后编译 / 上传。
每课的接线和说明见该课目录里的 README.md。

## 关于密钥

需要 WiFi 密码 / API Key 的工程（如 01），密钥放在 `src/secrets.h`，
该文件被 `.gitignore` 排除、不会上传。克隆代码后：复制 `src/secrets.h.example`
为 `src/secrets.h`，填入自己的值即可编译。

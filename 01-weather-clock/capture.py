#!/usr/bin/env python3
"""临时工具: 复位 ESP32 并抓取 N 秒串口输出。

用法: python capture.py <秒数> [输出文件]
用完可以删掉。
"""
import sys
import time

import serial

PORT = "/dev/cu.usbserial-A5069RR4"
BAUD = 115200

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
OUT = sys.argv[2] if len(sys.argv) > 2 else None

ser = serial.Serial(PORT, BAUD, timeout=1)

# ESP32 自动复位电路: RTS 接 EN 脚, DTR 接 GPIO0。
# 拉低 EN 再松开 = 重启, 并保持 GPIO0 高电平 = 正常启动(不进下载模式)。
ser.setDTR(False)
ser.setRTS(True)
time.sleep(0.1)
ser.setRTS(False)
time.sleep(0.1)
ser.reset_input_buffer()

deadline = time.time() + SECONDS
lines = []
while time.time() < deadline:
    raw = ser.readline()  # timeout=1, 不会永久阻塞
    if not raw:
        continue
    text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
    print(text, flush=True)
    lines.append(text)

ser.close()

if OUT:
    with open(OUT, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\n[saved to {OUT}]", file=sys.stderr)

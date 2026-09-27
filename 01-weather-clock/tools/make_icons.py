#!/usr/bin/env python3
"""
天气图标生成器
==============

把"用几何图形描述的图标"光栅化成 32x32 的单色位图, 输出成 C 头文件。

【为什么要写这个脚本】
32x32 的单色位图 = 1024 个像素 = 128 字节。手写这些十六进制数是自虐:
改一个像素要在 16 进制和二进制之间来回换算, 而且看不出改完长什么样。

所以我们在高层描述 —— "在 (16,16) 画一个半径 8 的实心圆",
让脚本去算像素。脚本输出的每一行都带 ASCII 预览:

    0x00, 0x01, 0x80, 0x00,  // ........##......

你在下面改完跑一次, 直接看注释就知道画成什么样了, 不用烧到板子上试。

【用法】
    python tools/make_icons.py
会在 src/weather_icons.h 生成头文件, 同时在终端打印预览。

【单色位图是怎么存的】
每行 32 个像素 → 每 8 个像素打包成 1 个字节 → 每行 4 个字节。
字节的最高位 (bit7) 对应这一行最左边的像素。
    ########  ->  0xFF
    #.......  ->  0x80
    .......#  ->  0x01
Adafruit 的 drawBitmap() 就是这么读的。
"""

import math
import os

W = 32
H = 32


# ===============================================================
# 一块可以画画的画布
# ===============================================================

class Canvas:
    def __init__(self):
        # p[y][x], 1 = 亮, 0 = 灭
        self.p = [[0] * W for _ in range(H)]

    def px(self, x, y, v=1):
        """点亮/熄灭一个像素。坐标超出画面就忽略 (相当于自动裁剪)。"""
        x, y = int(round(x)), int(round(y))
        if 0 <= x < W and 0 <= y < H:
            self.p[y][x] = v

    def disc(self, cx, cy, r, v=1):
        """实心圆。v=0 时是"挖掉" —— 用来做月牙那种减法图形。"""
        for y in range(H):
            for x in range(W):
                if (x - cx) ** 2 + (y - cy) ** 2 <= r * r:
                    self.p[y][x] = v

    def rect(self, x0, y0, x1, y1, v=1):
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                self.px(x, y, v)

    def line(self, x0, y0, x1, y1, v=1, weight=1):
        """Bresenham 直线。weight>1 时画成方块笔刷, 就是加粗。"""
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while True:
            if weight <= 1:
                self.px(x, y, v)
            else:
                self.rect(x, y, x + weight - 1, y + weight - 1, v)
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def ray(self, cx, cy, ang, r0, r1, halfw=0.5, v=1):
        """从圆心射出去的一条光线, 用来画太阳。halfw 是半宽 (像素)。"""
        dx, dy = math.cos(ang), math.sin(ang)
        nx, ny = -dy, dx  # 垂直方向
        r = r0
        while r <= r1:
            t = -halfw
            while t <= halfw:
                self.px(cx + dx * r + nx * t, cy + dy * r + ny * t, v)
                t += 0.5
            r += 0.25

    def poly(self, pts, v=1):
        """填充多边形, 扫描线算法。pts 是 [(x, y), ...] 顶点列表。

        原理: 从上往下逐行扫描, 求出这一行和多边形各条边的交点,
        排序后两两配对, 配对之间就是多边形内部。
        """
        ys = [p[1] for p in pts]
        n = len(pts)
        for y in range(int(math.floor(min(ys))), int(math.ceil(max(ys))) + 1):
            xs = []
            for i in range(n):
                x0, y0 = pts[i]
                x1, y1 = pts[(i + 1) % n]
                if y0 == y1:
                    continue  # 水平边不参与交点计算
                if min(y0, y1) <= y < max(y0, y1):
                    xs.append(x0 + (y - y0) / (y1 - y0) * (x1 - x0))
            xs.sort()
            for i in range(0, len(xs) - 1, 2):
                for x in range(int(math.ceil(xs[i])), int(math.floor(xs[i + 1])) + 1):
                    self.px(x, y, v)

    def clip_bottom(self, y_cut):
        """把 y >= y_cut 的部分全部擦掉。给云朵削一个平底。"""
        for y in range(y_cut, H):
            for x in range(W):
                self.p[y][x] = 0


C = Canvas  # 简写


# ===============================================================
# 各个图标 —— 想改样子就改这里
# ===============================================================

def icon_sun():
    """太阳 (晴, 白天)"""
    c = C()
    c.disc(16, 16, 8)
    # 光芒从半径 11 画到 14, 端点落在 2..30 之间, 不顶到画面边缘
    for i in range(8):
        c.ray(16, 16, i * math.pi / 4, 11, 14, halfw=0.75)
    return c


def icon_moon():
    """月亮 (晴, 夜间) —— 用一个偏移的圆"挖"出月牙

    窍门: 挖掉的那个圆要比本体**更大**。两个等大的圆相减, 月牙两端是齐头的;
    用一个更大的圆去挖, 尖端才会收成一个点, 才像月亮。
    """
    c = C()
    c.disc(16, 16, 12)
    c.disc(24, 13, 17, v=0)  # 挖掉右上方
    return c


def icon_partly():
    """多云 (太阳从云后面露出来)"""
    c = C()
    # 先画太阳 (在右上角), 云会盖住它的一部分。
    # 只画朝上的 5 条光线: 横向和朝下的光线会飘在云外面, 看起来像噪点。
    # 注意角度要取负的 —— 屏幕坐标 y 是**向下**增大的, 所以"朝上"是负角度。
    c.disc(20, 11, 5)
    for deg in (55, 90, 125):
        c.ray(20, 11, -math.radians(deg), 7, 10, halfw=0.75)
    # 再画云 (前景, 盖住太阳下半部分)
    c.disc(9, 20, 5)
    c.disc(15, 17, 7)
    c.disc(21, 20, 5)
    c.rect(7, 19, 23, 30)
    c.clip_bottom(26)
    return c


def icon_cloud():
    """阴 / 多云 (一朵云)"""
    c = C()
    c.disc(10, 18, 6)
    c.disc(17, 14, 8)
    c.disc(24, 18, 6)
    c.rect(8, 17, 26, 30)
    c.clip_bottom(25)
    return c


def _rain_cloud(c):
    """雨/雪/雷共用: 一朵往上挪的云, 给下面腾地方"""
    c.disc(10, 12, 5)
    c.disc(16, 9, 7)
    c.disc(22, 12, 5)
    c.rect(8, 11, 24, 30)
    c.clip_bottom(18)


def icon_rain():
    """雨"""
    c = C()
    _rain_cloud(c)
    for x in (9, 16, 23):
        c.line(x, 22, x - 3, 28)
    return c


def icon_storm():
    """雷阵雨"""
    c = C()
    _rain_cloud(c)
    # 闪电用一个六边形填充。用"两条粗斜线"画出来会变成一根粗棒子,
    # 因为加粗的直线在拐弯处会糊成一团 —— 多边形才是对的工具。
    c.poly([(18, 16), (10, 25), (14, 25), (9, 31), (20, 22), (16, 22)])
    return c


def icon_snow():
    """雪 —— 每片雪花是一个小十字"""
    c = C()
    _rain_cloud(c)
    for cx, cy in ((9, 24), (16, 27), (23, 24)):
        c.line(cx - 2, cy, cx + 2, cy)
        c.line(cx, cy - 2, cx, cy + 2)
    return c


def icon_fog():
    """雾 / 霾"""
    c = C()
    for x0, y0, x1 in ((4, 8, 27), (2, 14, 24), (7, 20, 30), (4, 26, 25)):
        c.rect(x0, y0, x1, y0 + 2)
    return c


def icon_unknown():
    """认不出来的天气 —— 画个问号, 总比画错强"""
    c = C()
    c.disc(16, 8, 7)            # 问号钩子: 外圆
    c.disc(16, 8, 4, v=0)       # 挖空 -> 变成一个环
    c.rect(0, 13, 15, 31, v=0)  # 把环的左下半边擦掉, 钩子才开口
    c.rect(13, 12, 18, 22)      # 竖杆
    c.disc(15, 27, 3)           # 下面的点
    return c


ICONS = [
    ("ICON_SUN", icon_sun, "晴 (白天)"),
    ("ICON_MOON", icon_moon, "晴 (夜间)"),
    ("ICON_PARTLY", icon_partly, "多云 / 晴间多云"),
    ("ICON_CLOUD", icon_cloud, "阴 / 多云"),
    ("ICON_RAIN", icon_rain, "雨"),
    ("ICON_STORM", icon_storm, "雷阵雨"),
    ("ICON_SNOW", icon_snow, "雪"),
    ("ICON_FOG", icon_fog, "雾 / 霾"),
    ("ICON_UNKNOWN", icon_unknown, "未知"),
]


# ===============================================================
# 光栅化 -> 字节 -> C 代码
# ===============================================================

def to_bytes(c, x0=0, y0=0, w=W, h=H):
    """把画布上的一块区域打包成单色位图的字节序列。"""
    out = []
    for y in range(y0, y0 + h):
        for bx in range(x0, x0 + w, 8):
            byte = 0
            for i in range(8):
                if c.p[y][bx + i]:
                    byte |= 1 << (7 - i)  # 最左边的像素放最高位
            out.append(byte)
    return out


def ascii_rows(data, x0=0, w=W, h=H):
    """把字节序列还原成 ASCII 图案, 用于注释预览。"""
    rows = []
    bytes_per_row = w // 8
    for y in range(h):
        s = ""
        for j in range(bytes_per_row):
            b = data[y * bytes_per_row + j]
            for i in range(8):
                s += "#" if (b >> (7 - i)) & 1 else "."
        rows.append(s)
    return rows


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out_path = os.path.join(here, "..", "src", "weather_icons.h")

    lines = []
    lines.append("// ============================================================")
    lines.append("// 天气图标位图 —— 这个文件是 tools/make_icons.py 自动生成的")
    lines.append("//")
    lines.append("// 【不要手改这个文件】要改图标样子请改脚本, 然后:")
    lines.append("//     python tools/make_icons.py")
    lines.append("//")
    lines.append("// 【怎么读这些十六进制数】")
    lines.append("//   每一行 = 图标的一行 = 32 个像素 = 4 个字节。")
    lines.append("//   每个字节的高位 (bit7) 对应左边。所以:")
    lines.append("//       0x80 = 10000000 = 最左边 1 个像素亮")
    lines.append("//       0x01 = 00000001 = 最右边 1 个像素亮")
    lines.append("//       0xFF = 11111111 = 连续 8 个像素全亮")
    lines.append("//   每行右边的注释就是这个字节画出来长什么样, 一眼能对上。")
    lines.append("//")
    lines.append("// 【尺寸】32x32, 单色, 1 = 亮。用 display.drawBitmap(x, y, ICON_XXX, 32, 32, SSD1306_WHITE) 画。")
    lines.append("// ============================================================")
    lines.append("")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <Arduino.h>")
    lines.append("")

    previews = []
    for name, fn, desc in ICONS:
        c = fn()
        data = to_bytes(c)
        rows = ascii_rows(data)

        previews.append((name, desc, rows))

        lines.append("// ---- %s : %s ----" % (name, desc))
        lines.append("static const uint8_t %s[] PROGMEM = {" % name)
        for y in range(H):
            quad = data[y * 4 : y * 4 + 4]
            hexpart = " ".join("0x%02X," % b for b in quad)
            lines.append("    %-22s // %s" % (hexpart, rows[y]))
        lines.append("};")
        lines.append("")

    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")

    # 终端预览
    for name, desc, rows in previews:
        print("== %s (%s) ==" % (name, desc))
        for r in rows:
            print("   " + r)
        print()

    print("已生成: %s" % os.path.normpath(out_path))


if __name__ == "__main__":
    main()

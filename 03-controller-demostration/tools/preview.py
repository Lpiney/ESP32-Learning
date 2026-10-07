#!/usr/bin/env python3
"""生成 03 课 OLED 画面的预览图（SVG）。

把 src/main.cpp 里 renderFrame() 的绘图逻辑用 Python 照着跑一遍，
这样不用接硬件就能先看到画面长什么样、有没有元素重叠。

用法:
    python3 tools/preview.py            # 写 screen-preview.svg
"""
import math
import pathlib

W, H = 128, 64
S = 4            # 每个显示像素放大成 4 个 SVG 单位
PAD = 14
DEADZONE = 0.15


class Canvas:
    """照着 Adafruit_GFX 的接口实现，坐标越界就丢弃（和真库行为一致）。"""

    def __init__(self):
        self.ops = []

    def _put(self, s):
        self.ops.append(s)

    def _ok(self, x, y):
        return 0 <= x < W and 0 <= y < H

    def pixel(self, x, y, black=False):
        x, y = int(round(x)), int(round(y))
        if self._ok(x, y):
            fill = "#000" if black else "#fff"
            self._put(f'<rect x="{PAD + x*S}" y="{PAD + y*S}" width="{S}" '
                      f'height="{S}" fill="{fill}"/>')

    def hline(self, x, y, w):
        for i in range(w):
            self.pixel(x + i, y)

    def vline(self, x, y, h):
        for i in range(h):
            self.pixel(x, y + i)

    def line(self, x0, y0, x1, y1):
        # Bresenham，和 Adafruit_GFX 一样
        dx, dy = abs(x1 - x0), -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.pixel(x0, y0)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

    def fill_circle(self, cx, cy, r):
        for y in range(-r, r + 1):
            for x in range(-r, r + 1):
                if x * x + y * y <= r * r:
                    self.pixel(cx + x, cy + y)

    def hline_gap(self, x, y, w, gap):
        """对应 main.cpp 的 hLineWithGap()：中间挖掉 gap 像素的横线。"""
        mid = x + w // 2
        half = gap // 2
        x0, x1 = mid - half, mid + half
        for i in range(x, x + w):
            if x0 <= i < x1:
                continue
            self.pixel(i, y)

    def rect(self, x, y, w, h):
        self.hline(x, y, w)
        self.hline(x, y + h - 1, w)
        self.vline(x, y, h)
        self.vline(x + w - 1, y, h)

    def fill_rect(self, x, y, w, h):
        for j in range(h):
            self.hline(x, y + j, w)

    def text(self, x, y, s, inverted=False):
        """按 6x8 网格估算（真字体是 5x7 + 1px 间距），够看布局了。"""
        cw, ch = 6, 8
        if inverted:
            for i in range(len(s) * cw):
                for j in range(ch):
                    self.pixel(x + i, y + j)      # 反白底
        for i, ch_ in enumerate(s):
            if ch_ == " ":
                continue
            # 画一个占满字格的小方块，表示"这里有字"
            for dx in range(1, cw - 1):
                for dy in range(1, ch - 1):
                    self.pixel(x + i * cw + dx, y + dy,
                               black=inverted)


def dashed_arc(c, cx, cy, rx, ry, a0, a1, dash, gap):
    """和 main.cpp 的 dashedArc() 逐行等价（椭圆版）。"""
    period = dash + gap
    px = 0
    prev = False
    a = a0
    while a <= a1:
        on = (px % period) < dash
        if on or prev:
            c.pixel(cx + round(rx * math.cos(a)), cy + round(ry * math.sin(a)))
        prev = on
        px += 1
        a += 0.02


def render(nx, ny, state, out_l, out_r):
    """对应 main.cpp 的 renderFrame()。

    state 是左上角第一行：摇杆模式 "READY"/"WAIT"，
    电脑手柄模式 "LOCK"/"UNLOCK"/"NOLINK"。out_l/out_r 是实际输出的左右脉宽(us)。
    """
    c = Canvas()
    in_dead = math.hypot(nx, ny) < DEADZONE

    cx, cy, rx, ry = 32, 30, 13, 10

    # 8.1 行程圈（椭圆）+ 死区圈
    dashed_arc(c, cx, cy, rx, ry, 0.0, 2 * math.pi, 2, 3)
    rdx = max(3, round(DEADZONE * rx))
    rdy = max(3, round(DEADZONE * ry))
    dashed_arc(c, cx, cy, rdx, rdy, 0.0, 2 * math.pi, 1, 3)
    c.hline(cx - 5, cy, 11)
    c.vline(cx, cy - 5, 11)

    # 8.2 摇杆位置
    dx = cx + round(nx * rx)
    # 减号：屏幕 y 向下增大，约定"推上 = ny 正"，所以翻过来（和 main.cpp 一致）
    dy = cy - round(ny * ry)
    c.line(cx, cy, dx, dy)
    c.fill_circle(dx, dy, 2)

    # 8.3 左上角：解锁状态 + 死区状态
    c.text(2, 0, state)
    c.text(2, 9, "DEAD" if in_dead else "OUT")

    # 右栏：输出脉宽 + 摇杆位置
    c.text(56, 0, f"L={out_l}")
    c.text(56, 9, f"R={out_r}")
    c.text(56, 20, f"X{nx:+.2f}")
    c.text(56, 31, f"Y{ny:+.2f}")

    # 8.4 底部一维展开（中间缺口 = 中点）
    barx, barw, bary, barh = 18, 88, 48, 8
    barcy = bary + 2
    c.text(4, 50, "X")
    c.hline_gap(barx, barcy, barw, 6)
    c.fill_circle(barx + round((nx + 1) * 0.5 * barw), barcy, 2)

    c.text(4, 57, "Y")
    c.hline_gap(barx, barcy + barh, barw, 6)
    c.fill_circle(barx + round((ny + 1) * 0.5 * barw), barcy + barh, 2)

    return c, in_dead


def panel(c, title, sub):
    """把一块屏包成 SVG 分组。"""
    ox = PAD
    oy = PAD
    out = [f'<g><rect x="{ox-3}" y="{oy-3}" width="{W*S+6}" height="{H*S+6}" '
           f'fill="#101418" stroke="#3a4450" stroke-width="2"/>']
    out.append(f'<rect x="{ox}" y="{oy}" width="{W*S}" height="{H*S}" fill="#000"/>')
    out.extend(c.ops)
    out.append(f'<text x="{ox}" y="{oy-10}" fill="#8ab4f8" '
               f'font-family="monospace" font-size="13">{title}</text>')
    out.append('</g>')
    return "".join(out), sub


def main():
    # 和固件/motor_sim 一样的映射：左 = fwd-turn，右 = fwd+turn
    def pulses(nx, ny):
        from_sim = None  # 这里只需要几个例子，直接照公式算
        def dz(v):
            a = abs(v)
            if a <= DEADZONE:
                return 0.0
            n = (a - DEADZONE) / (1.0 - DEADZONE)
            return -n if v < 0 else n
        x, y = dz(nx), dz(ny)
        clamp = lambda u: round(1500 + max(-1.0, min(1.0, u)) * 500)
        return clamp(y - x), clamp(y + x)

    cases = [
        ((0.00, 0.00), "WAIT", 1500, 1500, "开机预热中：摇杆暂不响应，两路中立 1500"),
        ((0.00, 1.00), "READY", 2000, 2000, "Y=+1：两轮同速前进 2000/2000"),
        ((1.00, 0.00), "READY", 1000, 2000, "X=+1：原地右转（右路 2000）"),
        ((-1.00, 0.00), "READY", 2000, 1000, "X=-1：原地左转（左路 2000）"),
        ((0.42, -0.30), "READY", 1194, 1629, "斜推：前进中右转"),
        ((0.00, 0.00), "LOCK", 1500, 1500,
         "电脑手柄模式 · 锁着：按一下安全键才解锁，两路中立"),
        ((0.42, 0.30), "UNLOCK", 1629, 1194,
         "电脑手柄模式 · 已解锁：推杆就出力（再按一下锁上）"),
    ]

    groups = []
    for (nx, ny), state, ol, orr, sub in cases:
        c, dead = render(nx, ny, state, ol, orr)
        groups.append(panel(c, f"{state}  X={nx:+.2f} Y={ny:+.2f}", sub))

    svg_w = W * S + PAD * 2
    svg_h = (H * S + PAD * 2 + 22) * len(groups) + 40
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{svg_w}" '
             f'height="{svg_h}" viewBox="0 0 {svg_w} {svg_h}">',
             '<rect width="100%" height="100%" fill="#1b1f24"/>']

    y = PAD + 22
    for grp, sub in groups:
        parts.append(f'<g transform="translate(0,{y - PAD - 22})">{grp}</g>')
        parts.append(f'<text x="{PAD}" y="{y + H*S + 6}" fill="#7f8c9b" '
                     f'font-family="monospace" font-size="12">{sub}</text>')
        y += H * S + PAD * 2 + 22

    parts.append('</svg>')

    out = pathlib.Path(__file__).resolve().parent.parent / "screen-preview.svg"
    out.write_text("\n".join(parts), encoding="utf-8")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()

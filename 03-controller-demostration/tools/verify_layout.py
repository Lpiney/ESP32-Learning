#!/usr/bin/env python3
"""布局自检：确认 OLED 画面上的元素不会互相遮挡、也不会画出屏幕外。

为什么需要这个脚本：OLED 只有 128x64，元素一多很容易在某个摇杆位置上
撞在一起（比如摇杆的实心点跑到左上角，正好压在按键状态文字上）。
手动试很难穷举，所以这里把 renderFrame() 的几何照抄一遍，把摇杆所有
位置 + 两种按键状态全部扫一遍。

它检查三件事：
  1. 越界：任何元素超出 0~127 / 0~63
  2. 文字被图形压住：把图形栅格化成像素集合，看有没有像素落在文字的字格里
     （这是真正的像素级检查，不是拿外接矩形粗略估的）
  3. 图形压在圆环的弧线上：弧线只有 1px 宽，被别的东西盖住就看不见了

用法:
    python3 tools/verify_layout.py        # 有问题时退出码 1
"""
import math
import sys

W, H = 128, 64
DEADZONE = 0.15

# ---- 这些常量必须和 src/main.cpp 的 renderFrame() 保持一致 ----
CX, CY, RX, RY = 32, 30, 13, 10      # 注意是椭圆：纵向压扁到 11
BAR_X, BAR_W, BAR_Y, BAR_H = 18, 88, 48, 8
LBL_X, LBL_Y = 4, (50, 57)

# 新版画面（双电机遥控器）的文字格。每个元素是 (x, y, 字符数)。
# 全部用显式坐标列出，不再用一串 ROW_* 常量拼——那样一改就容易错位。
STATE_BOX = (2, 0, 5)        # "READY" / "WAIT"
TXT_L = (56, 0, 6)            # "L=1500"
TXT_R = (56, 9, 6)            # "R=1500"
TXT_X = (56, 20, 6)           # "X +0.42"
TXT_Y = (56, 31, 6)           # "Y -0.17"
TXT_S = (2, 9, 4)             # "DEAD" / "OUT "（左上角）

GLYPH_W, GLYPH_H = 6, 7     # GFX 默认字体：5x7 字形 + 1px 右边距


def px_line(x0, y0, x1, y1):
    """Bresenham，等价于 Adafruit_GFX 的 drawLine。"""
    out = set()
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        out.add((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return out


def px_ring(cx, cy, rx, ry, dash, gap):
    """等价于 main.cpp 的 dashedArc()。"""
    out = set()
    period = dash + gap
    i = 0
    prev = False
    a = 0.0
    while a <= 2 * math.pi:
        on = (i % period) < dash
        if on or prev:
            out.add((cx + int(round(rx * math.cos(a))),
                     cy + int(round(ry * math.sin(a)))))
        prev = on
        i += 1
        a += 0.02
    return out


def px_circle(cx, cy, r):
    return {(cx + x, cy + y)
            for y in range(-r, r + 1) for x in range(-r, r + 1)
            if x * x + y * y <= r * r}


def glyph_box(x, y, n_chars):
    """6x8 字格的像素范围（含首含尾）。字形实际高 7px，最后一行是字间距。"""
    return {(x + i, y + j)
            for i in range(n_chars * GLYPH_W) for j in range(GLYPH_H)}


def ring_pixels():
    """圆环（行程圈 + 死区圈）的像素。"""
    rdx = max(3, int(round(DEADZONE * RX)))
    rdy = max(3, int(round(DEADZONE * RY)))
    return px_ring(CX, CY, RX, RY, 2, 3) | px_ring(CX, CY, rdx, rdy, 1, 3)


def graphics(nx, ny):
    """这一帧里所有"图形"元素的像素（不含文字，也不含圆环）。"""
    g = set()
    # 十字准星 + 从圆心指向摇杆的连线。
    # 这两个是**故意**压在圆环上的（准星本来就要画到圈上），单独拿出来，
    # 这样"圆环被盖住"的检查里就不会把它们误报成问题。
    g |= px_line(CX - 5, CY, CX + 5, CY)
    g |= px_line(CX, CY - 5, CX, CY + 5)
    dx, dy = CX + int(round(nx * RX)), CY + int(round(ny * RY))
    g |= px_line(CX, CY, dx, dy)
    g |= px_circle(dx, dy, 2)                     # 实心点
    # 底部两条横线（中间留缺口当中点标记，没有竖线，所以不会顶到文字）+ 游标
    for cy in (BAR_Y + 2, BAR_Y + 2 + BAR_H):
        g |= px_line_gap(BAR_X, cy, BAR_W, 6)
    g |= px_circle(BAR_X + int(round((nx + 1) * 0.5 * BAR_W)), BAR_Y + 2, 2)
    g |= px_circle(BAR_X + int(round((ny + 1) * 0.5 * BAR_W)), BAR_Y + 2 + BAR_H, 2)
    return g


def px_line_gap(x, y, w, gap):
    """等价于 main.cpp 的 hLineWithGap()：中间挖掉 gap 像素的横线。"""
    out = set()
    mid = x + w // 2
    half = gap // 2
    x0, x1 = mid - half, mid + half
    for i in range(x, x + w):
        if x0 <= i < x1:
            continue
        out.add((i, y))
    return out


def text_pixels(state):
    """当前这一帧里所有"文字"占用的字格像素。

    state 是左上角第一行显示的字（"READY"/"WAIT"，电脑手柄模式下是
    "LOCK"/"UNLOCK"/"NOLINK"），长度不同，所以字格宽度按 len(state) 算。
    """
    t = set()
    t |= glyph_box(STATE_BOX[0], STATE_BOX[1], len(state))
    t |= glyph_box(*TXT_L)        # "L=2000"（圈内）
    t |= glyph_box(*TXT_R)        # "R=1000"（圈内）
    t |= glyph_box(*TXT_X)        # "X +0.42"
    t |= glyph_box(*TXT_Y)        # "Y -0.17"
    t |= glyph_box(*TXT_S)        # "DEAD" / "OUT "
    t |= glyph_box(LBL_X, LBL_Y[0], 1)   # 底部 "X"
    t |= glyph_box(LBL_X, LBL_Y[1], 1)   # 底部 "Y"
    return t


def check(nx, ny, state):
    problems = []
    g = graphics(nx, ny)
    ring = ring_pixels()
    t = text_pixels(state)

    oob_g = [p for p in g if not (0 <= p[0] < W and 0 <= p[1] < H)]
    oob_t = [p for p in t if not (0 <= p[0] < W and 0 <= p[1] < H)]
    if oob_g:
        problems.append(f"图形越界 {len(oob_g)}px 例如 {sorted(oob_g)[:3]}")
    if oob_t:
        problems.append(f"文字越界 {len(oob_t)}px 例如 {sorted(oob_t)[:3]}")

    # 文字被图形压住（真像素级）
    hit = t & g
    if hit:
        problems.append(f"文字被图形压住 {len(hit)}px 例如 {sorted(hit)[:3]}")

    # 文字压在圆环弧线上（弧线只有 1px 宽，被字盖住就断了）
    broken = t & ring
    if broken:
        problems.append(f"文字压住圆环 {len(broken)}px 例如 {sorted(broken)[:3]}")
    return problems


def main():
    total = 0
    bad = 0
    examples = []
    for i in range(0, 360, 2):
        for rr in (0.0, 0.05, 0.15, 0.2, 0.3, 0.5, 0.7, 0.85, 1.0, 1.414):
            # 注意：main.cpp 的 mapAxis() 会把每个轴各自夹到 ±1.0，
            # 所以有效的摇杆位置只在正方形 [-1,1]x[-1,1] 内（斜推到角上
            # 的 (1,1) 对应 mag=1.414，这是合法的"推到底"）。
            nx = max(-1.0, min(1.0, rr * math.cos(math.radians(i))))
            ny = max(-1.0, min(1.0, rr * math.sin(math.radians(i))))
            # 左上角第一行可能出现的字：摇杆模式 READY/WAIT；
            # 电脑手柄模式 LOCK/UNLOCK/NOLINK（最长 6 个字符，是最挤的那种）
            for state in ("READY", "WAIT", "LOCK", "UNLOCK", "NOLINK"):
                total += 1
                p = check(nx, ny, state)
                if p:
                    bad += 1
                    if len(examples) < 8:
                        examples.append((nx, ny, state, p))

    for nx, ny, state, p in examples:
        print(f"  ✗ ({nx:+.2f},{ny:+.2f}) {state}: " + "; ".join(p))

    print(f"\n扫描 {total} 帧（所有摇杆位置 x 两种状态），问题帧 {bad}")
    if bad:
        print("布局有问题，需要调整 src/main.cpp 里的坐标。")
        return 1
    print("布局自检通过：没有越界，文字没有被图形遮挡。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""摇杆 / 转向 实机体检 —— 一条命令，跑完给出结论和"要改哪一行"。

为什么要有这个脚本
------------------
2026-10-03 那次"左右转向怎么调都不对"，根因不是硬件，是**测量方法**本身：

  1. 没确认串口有没有被别的程序占着。macOS 允许多个进程同时打开同一个
     /dev/cu.*，**字节会被随机瓜分**，两边都只能收到残缺的行。
  2. 没确认波特率。串口停在 9600 时去读 115200 的数据，全是乱码 ——
     "能收到字节"不等于"数据是对的"。
  3. 解析用 ' X=' 去 split，'JX=' 先被匹配到，把原始 ADC 当成了归一化值。
  4. 拿自己写的模拟器当选真值，等于自己出题自己判卷。

这个脚本把这四条都堵死：先验链路（占用/波特率/行速率）→ 逐 token 解析 →
让人在真机上做动作 → 按测到的事实给结论。

**只读串口，不向板子发送任何东西。** 零第三方依赖。

用法
----
    python3 tools/joy_check.py                    # 自动找串口
    python3 tools/joy_check.py --port /dev/cu.usbserial-A5069RR4
    python3 tools/joy_check.py --selftest         # 不接硬件，只验判决逻辑
"""

from __future__ import annotations

import argparse
import pathlib
import statistics
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from motor_sim import (BOOT_IGNORE_MS, PULSE_NEUTRAL,  # noqa: E402
                       SerialReader, Telemetry, list_ports, parse_joy,
                       port_holders)

FIRMWARE_LINE_HZ = 20.0      # 固件 TELEMETRY_MS=50 -> 20 行/秒
SPAN_GOOD = 500              # ADC 跨度到这个数以上，说明这一轴确实读得到
SPAN_SMALL = 100             # 低于这个数基本等于"没动"

# 结论里用到的判定门槛
CENTER_MIN, CENTER_MAX = 300, 3800   # 中点落在这之外 = 这一路八成没接对
NOISE_WARN = 200                     # 松手时的抖动
PUSH_MIN = 0.5                       # 推到底时 |X| 至少要这么大


# ===========================================================================
#  采样
# ===========================================================================

class Recorder:
    """把串口行收下来，存成人能算的表格。

    逐 token 解析 key=value。**不要**用 line.split(" X=") 这种写法：
    行里同时有 JX= 和 X=，一不小心就把原始 ADC 当成了归一化值。
    """

    def __init__(self):
        self.rows = []          # (t, jx, jy, x, y, l, r, ready, in_dead)
        self.status = "启动中"
        self.joy_lines = 0
        self.other_lines = []
        self.first_joy_line = ""

    def on_line(self, line: str):
        t = Telemetry()
        if not parse_joy(line, t):
            if len(self.other_lines) < 6:
                self.other_lines.append(line)
            return
        self.joy_lines += 1
        if not self.first_joy_line:
            self.first_joy_line = line
        self.rows.append((time.monotonic(), t.raw_x, t.raw_y, t.x, t.y,
                          t.l_us, t.r_us, t.ready, t.in_dead))

    # ---- 取一段
    def mark(self) -> int:
        return len(self.rows)

    def since(self, mark: int) -> list:
        return self.rows[mark:]

    def wait_first(self, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.rows:
                return True
            time.sleep(0.05)
        return False

    def wait_ready(self, timeout: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.rows and self.rows[-1][7]:
                return True
            time.sleep(0.05)
        return False


def stat(rows, idx, tail: float | None = None):
    """一列的最小/最大/中位数。tail=N 只看最后 N 秒。"""
    if not rows:
        return None
    if tail is not None:
        end = rows[-1][0]
        rows = [r for r in rows if r[0] >= end - tail]
        if not rows:
            return None
    vals = [r[idx] for r in rows]
    return {"min": min(vals), "max": max(vals), "span": max(vals) - min(vals),
            "median": statistics.median(vals), "n": len(vals)}


def capture(rec: Recorder, seconds: float, title: str, hint: str) -> list:
    """提示 -> 等用户按回车 -> 采样若干秒（带实时进度）-> 返回这段的行。"""
    print(f"\n>>> {title}")
    print(f"    {hint}")
    input("    准备好了按回车开始 >>> ")
    mark = rec.mark()
    t0 = time.monotonic()
    while True:
        el = time.monotonic() - t0
        if el >= seconds:
            break
        rows = rec.since(mark)
        if rows:
            jx = stat(rows, 1)
            jy = stat(rows, 2)
            nx = rows[-1][3]
            ny = rows[-1][4]
            bar = f"JX {jx['min']:4d}~{jx['max']:4d} (跨度 {jx['span']:4d})   " \
                  f"JY {jy['min']:4d}~{jy['max']:4d} (跨度 {jy['span']:4d})   " \
                  f"X {nx:+.2f} Y {ny:+.2f}"
        else:
            bar = "（还没收到遥测…）"
        sys.stdout.write(f"\r    [{seconds - el:4.1f}s] {bar}      ")
        sys.stdout.flush()
        time.sleep(0.1)
    sys.stdout.write("\r" + " " * 100 + "\r")
    sys.stdout.flush()
    return rec.since(mark)


def ask(prompt: str, options: list) -> str:
    """单选题。options: [(代号, 说明)]，返回选中的代号。"""
    print(f"\n{prompt}")
    for key, label in options:
        print(f"    {key}) {label}")
    while True:
        ans = input("    输入代号后回车 >>> ").strip().lower()
        for key, _ in options:
            if ans == key:
                return key
        print("    没看懂，再输一次（就输括号里那个字母/数字）。")


# ===========================================================================
#  判决逻辑（纯函数，可以脱离硬件测：--selftest）
# ===========================================================================

def verdict_x_axis(lr_rows, ud_rows) -> dict:
    """X 轴到底读不读得到。返回 {level, span, swap, text}"""
    lr_x = stat(lr_rows, 1)
    lr_y = stat(lr_rows, 2)
    ud_y = stat(ud_rows, 2) if ud_rows else None
    if lr_x is None:
        return {"level": "none", "span": 0, "swap": False,
                "text": "这一段没收到遥测，测不了。"}
    span = lr_x["span"]
    if span >= SPAN_GOOD:
        return {"level": "ok", "span": span, "swap": False,
                "text": f"正常。推左右时 JX 跨度 {span} 计数（{lr_x['min']}→{lr_x['max']}）。"}
    if lr_y is not None and lr_y["span"] >= SPAN_GOOD * 0.6:
        return {"level": "swap", "span": span, "swap": True,
                "text": (f"★ 推左右的时候，变的是 JY（跨度 {lr_y['span']}），"
                         f"JX 几乎不动（跨度 {span}）\n"
                         "      → 左右这条信号线接到的不是 GPIO1。"
                         "URX / URY 两根线对调一下（或查 URX 那根线），再跑一次本脚本。")}
    if ud_y is not None and ud_y["span"] >= SPAN_GOOD and span < SPAN_SMALL:
        return {"level": "dead", "span": span, "swap": False,
                "text": (f"★ 上下推的时候 JY 正常（跨度 {ud_y['span']}），"
                         f"但左右推时 JX 不动（跨度 {span}）。\n"
                         "      同一段代码、相邻两个引脚、Y 能动 X 不动 —— "
                         "固件对两轴是完全对称的，先怀疑 X 那一路的信号本身。\n"
                         "      不用拆线就能分清是哪一边：把摇杆模块的 URX 和 URY "
                         "两根杜邦线对调，再跑一次。\n"
                         "        · 对调后推左右能让 JX 动 → GPIO1/固件这一路是好的，"
                         "问题在模块或 URX 那根线\n"
                         "        · 对调后 JX 还是不动   → 问题在 ESP32 这一路"
                         "（GPIO1 被别的东西占了？）")}
    return {"level": "small", "span": span, "swap": False,
            "text": (f"JX 跨度只有 {span} 计数，推左右时两个轴都没怎么动。\n"
                     "      是不是没推到底，或者只轻轻碰了一下？重跑一次，"
                     "这次把摇杆**慢慢推到底**再松开。")}


def verdict_x_sign(hold_rows) -> dict:
    """推右保持住的时候，屏幕上的 X 是正还是负。"""
    x = stat(hold_rows, 3, tail=1.5)
    jx = stat(hold_rows, 1, tail=1.5)
    if x is None or jx is None:
        return {"sign": 0, "text": "没测到。"}
    if jx["span"] > SPAN_GOOD * 0.8:
        return {"sign": 0, "text": (f"保持的那 1.5 秒里 JX 还在变（跨度 {jx['span']}），"
                                    "说明手没停稳，这次不算数。")}
    if x["median"] >= PUSH_MIN:
        return {"sign": +1, "text": f"推右 → X = {x['median']:+.2f}（正数，符合约定）"}
    if x["median"] <= -PUSH_MIN:
        return {"sign": -1, "text": f"推右 → X = {x['median']:+.2f}（**负数**，和约定相反）"}
    return {"sign": 0,
            "text": f"推右时 X 只有 {x['median']:+.2f}，几乎没推到底，这次不算数。"}


def recommend(x_sign: int, fwd: str, turn: str) -> list:
    """把事实翻成"要改哪一行"。

    x_sign：推右时屏幕上 X 的符号（按约定应该是 +1）
    fwd   ：'forward' 推上=前进 / 'spin' 一个往前一个往后 / 'unknown'
    turn  ：'right' 推右=船头右转 / 'left' / 'unknown'

    这里不猜物理，只用两件实测事实：
      · 推右时 X 的正负      —— 看屏幕/串口就知道，不需要电机
      · 推右时船头往哪边转  —— 只能实机看
    改法只有两种，而且都只改一处（或者两处一起改）：
      A. 改摇杆 X 的符号    → 屏幕上的 X 变正，同时左右出力对调
      B. 改 mixer 的两行    → 只对调左右出力，屏幕上的 X 不变
    """
    out = []
    if fwd == "spin":
        out.append("★ 先修前进方向，转向前的问题在此之前都测不准：")
        out.append("   推上到底时有一个电机在往后推（所以船原地打转，而不是往前走）。")
        out.append("   断电 → 把**往后退的那一路**电机的任意两根相线对调 → 上电重跑本脚本。")
        out.append("   （这么做是让两个电调对「2000 = 前进」的理解一致，之后才谈得上左右。）")
        return out

    JOY_FIX = ["      改 src/main.cpp 里 X 轴的符号常量：",
               "        #define JOY_X_SIGN      (-1.0f)   →   (+1.0f)",
               "      （★ 别再去 mapAxis(calX) 前面手写负号 —— 那和 JOY_X_SIGN 叠加",
               "        会变成双重取反，方向又翻回去。符号只能在一个地方管。）"]
    MIXER_FIX = ["    改 src/main.cpp 的 joyToMotorTargets()：把这两行的 +x / -x 对调。",
                 "      （现在写的是 targetL = unitToPulse(y + x); 和",
                 "        targetR = unitToPulse(y - x);  —— 对调后就是 y - x / y + x。）",
                 "    这两行的作用只是决定「哪一路出力多」，真船方向反了就换过来。",
                 "    必须同步 tools/motor_sim.py 的 motor_pulses() 和 "
                 "tools/test_firmware_logic.py，否则自检会报错。"]
    SYNC = "  改完跑一遍 python3 tools/test_firmware_logic.py，确认两边还一致。"

    if x_sign == 0:
        out.append("· 摇杆 X 的符号没测准（手没推到 ±0.5 以上，或者没稳住）。")
        out.append("  重跑一次本脚本，第 5 步把摇杆推到底再按住。")
        return out

    if turn == "unknown":
        out.append("· 转向没测（没接电机？）。这是个只跟读数有关的结论：")
        if x_sign == -1:
            out.append("   推右时屏幕显示 X 是负数 → 违反「X 正数 = 右」的约定。")
            out += JOY_FIX
            out.append("   ★ 这一改会让左右出力**对调**。等你接上电机，必须再跑一次本脚本")
            out.append("     的第 6 步确认转向 —— 别只看屏幕就收工，前两次就是这么反的。")
        else:
            out.append("   推右时屏幕显示 X 是正数，符号符合约定，不用动。")
        out.append("   （转向那一步只能实机看，接上电机后重跑本脚本。）")
        return out

    # 四种组合，一格一格写清楚，不让读者自己去推
    if x_sign == +1 and turn == "right":
        out.append("· 全都对上了：推右 → X 是正数 → 船头右转。")
        out.append("  **不需要改任何代码。**")
    elif x_sign == +1 and turn == "left":
        out.append("· 摇杆读数是对的（推右 = X 正数），是 mixer 把它接反了。")
        out += MIXER_FIX
        out.append(SYNC)
    elif x_sign == -1 and turn == "left":
        out.append("· 两处都反了，而且**改一处就够**：改摇杆的符号。")
        out += JOY_FIX
        out.append("   为什么一处就够：X 反了，左右出力自然也对调了，"
                   "推右就从左转变成右转。")
        out.append("   ★ 只改这一处，**不要**再去动 joyToMotorTargets()，"
                   "同时改两处等于没改。")
    else:   # x_sign == -1 and turn == "right"
        out.append("· 船现在是好的（推右 = 右转），只是屏幕上的 X 是镜像的。")
        out.append("  想统一的话必须**两处一起改**，只改一处会把船弄反：")
        out += JOY_FIX
        out += MIXER_FIX
        out.append("  也可以就这么用 —— 船没问题，只是屏幕上的 X 左右相反。")
        out.append(SYNC)
        return out
    return out


# ===========================================================================
#  流程
# ===========================================================================

def phase_link(rec: Recorder, reader: SerialReader, port: str) -> bool:
    print("\n[1/6] 串口体检")
    print(f"      端口 {port} @ 115200（本脚本自己设，不依赖上一个程序留下的值）")
    if not rec.wait_first(6.0):
        print("      ✘ 6 秒内一行固件遥测都没收到，先别往下测。看这几个数：")
        print(f"        收到字节 {reader.bytes_rx}，凑齐的行 {reader.lines_rx}")
        if reader.bytes_rx == 0:
            print("        → 一点数据都没有：板子在跑吗？插的是标 UART 的那个口吗？")
        elif reader.lines_rx == 0:
            print("        → 有字节但一行都没凑齐：多半是波特率不对，或者串口被抢了。")
        else:
            print("        → 有行但没有 JOY 行：烧的可能是别的固件。")
        if rec.other_lines:
            print("        非 JOY 行长这样：")
            for line in rec.other_lines[:3]:
                print(f"          | {line}")
        print("      （另外：板子刚上电 3 秒内是预热窗口，但这不影响收数据。）")
        return False

    time.sleep(2.0)
    rate = len(rec.rows) / max(0.5, rec.rows[-1][0] - rec.rows[0][0])
    ok = rate >= FIRMWARE_LINE_HZ * 0.75
    print(f"      行速率 {rate:.1f} 行/秒（固件是 {FIRMWARE_LINE_HZ:.0f} 行/秒）"
          f" {'✔' if ok else '✘'}")
    print("      固件遥测原样长这样（解析必须逐 token 来，不能用 ' X=' 去切）：")
    print(f"        | {rec.first_joy_line}")
    if not ok:
        print("      ✘ 行速率太低。macOS 允许多个程序同时打开同一个串口，")
        print("        字节会被瓜分，收到的行是残缺的 —— 这种数据**不能当证据**。")
        print(f"        先查是谁占着： lsof {port}")
        if input("      还是继续测吗？(y/N) >>> ").strip().lower() != "y":
            return False
    if rec.other_lines:
        print(f"      另外还收到 {len(rec.other_lines)} 行非遥测内容，例如：")
        for line in rec.other_lines[:3]:
            print(f"        | {line}")
    print("      等预热窗口过去…")
    if rec.wait_ready(BOOT_IGNORE_MS + 6000):
        print("      ✔ 固件已 READY（屏幕左上角也应该是 READY）")
    else:
        print("      ！一直没看到 READY=1：板子可能刚重启过，或卡在开机校准。")
    return True


def phase_center(rec: Recorder) -> None:
    rows = capture(rec, 4.0, "[2/6] 松手",
                   "两只手都离开摇杆，让它自己回中。")
    jx, jy = stat(rows, 1), stat(rows, 2)
    if jx is None:
        print("    这次没收到遥测。")
        return
    print(f"    中点  JX={jx['median']:.0f}  JY={jy['median']:.0f}   （固件校准后的理想值是各模块自己的中点）")
    print(f"    抖动  JX±{jx['span'] / 2:.0f}  JY±{jy['span'] / 2:.0f}"
          f"  {'✔' if max(jx['span'], jy['span']) < NOISE_WARN else '✘ 抖动偏大'}")
    for name, s in (("JX", jx), ("JY", jy)):
        if not (CENTER_MIN <= s["median"] <= CENTER_MAX):
            print(f"    ✘ {name} 中点 {s['median']:.0f} 离 2048 太远：这一路的电压"
                  "八成不是正常的中点（线没接/接错/推杆卡住）。")
    if max(jx["span"], jy["span"]) >= NOISE_WARN:
        print("    ！抖动这么大，通常是有只手还搭在摇杆上。")


def phase_axes(rec: Recorder):
    lr = capture(rec, 12.0, "[3/6] 左右推到底",
                 "把摇杆**慢慢推到底**再推到底（左 → 右 → 左…），来回 3 趟。")
    ud = capture(rec, 12.0, "[4/6] 上下推到底",
                 "现在上下推：底 → 顶 → 底…，来回 3 趟。")
    return lr, ud


def phase_hold(rec: Recorder):
    rows = capture(rec, 3.0, "[5/6] 推右到底并保持",
                   "把摇杆**推到最右边按住不动**，一直按到计时结束。")
    return rows


def phase_turn():
    print("\n[6/6] 转向和前进（需要接上电调 + 电机；没接就选 3 跳过）")
    print("      ⚠ 上电前先脱桨，手边留着电源开关。")
    fwd = ask("推上到底（前进方向），两个电机是什么表现？",
              [("1", "船/两个推进器都往前 → 前进正常"),
               ("2", "一个往前一个往后，原地打转"),
               ("3", "没接电机 / 看不出来")])
    turn = ask("推右到底，船头往哪边转？（从上往下看）",
               [("1", "右转（顺时针）"),
                ("2", "左转（逆时针）"),
                ("3", "没接电机 / 看不出来")])
    return {"1": "forward", "2": "spin", "3": "unknown"}[fwd], \
           {"1": "right", "2": "left", "3": "unknown"}[turn]


def report(axis, hold, fwd, turn) -> None:
    print("\n" + "=" * 62)
    print("结论")
    print("=" * 62)
    print("\n【A】X 轴读不读得到")
    print("     " + axis["text"])

    sign = verdict_x_sign(hold)
    print("\n【B】推右时 X 的正负（只看读数，不需要电机）")
    print("     " + sign["text"])

    print("\n【C】要改的地方")
    for line in recommend(sign["sign"], fwd, turn):
        print("   " + line)

    print("\n【D】下一步")
    if axis["level"] in ("ok",):
        print("   · 轴向没问题，按上面 【C】 改完，重跑本脚本确认。")
    else:
        print("   · 先把【A】里的问题解决掉，方向的事再谈 —— 读数不对的话，")
        print("     改方向就是猜，前两次就是这么反的。")


# ===========================================================================
#  脱机自测（验证判决逻辑，不需要硬件）
# ===========================================================================

def selftest() -> int:
    """把四种情况都喂进去，看建议对不对。"""
    def rows(x=None, jx=None, l=PULSE_NEUTRAL, r=PULSE_NEUTRAL):
        t = time.monotonic()
        return [(t, jx if jx is not None else 2048, 2048,
                 x if x is not None else 0.0, 0.0, l, r, True, False)]

    ok = True

    def expect(name, got, must_contain=(), must_not=()):
        nonlocal ok
        text = "\n".join(got)
        hit = all(s in text for s in must_contain) and \
            not any(s in text for s in must_not)
        print(f"  {'ok  ' if hit else 'FAIL'} {name}")
        if not hit:
            ok = False
            print("       ---- 实际输出 ----")
            for line in got:
                print("       " + line)

    print("判决逻辑自测（不接硬件）")

    MIXER_LINE = "targetL = unitToPulse(y + x);"
    SIGN_LINE = "JOY_X_SIGN"

    expect("推右=X正 且 推右=右转 → 不需要改",
           recommend(+1, "forward", "right"), ["不需要改任何代码"])

    expect("推右=X正 但 推右=左转 → 只改 mixer 两行",
           recommend(+1, "forward", "left"),
           [MIXER_LINE, "targetR = unitToPulse(y - x);"],
           must_not=[SIGN_LINE])

    expect("推右=X负 且 推右=左转 → 只改符号常量，且明说别同时改 mixer",
           recommend(-1, "forward", "left"),
           [SIGN_LINE, "同时改两处等于没改"],
           must_not=[MIXER_LINE])

    expect("推右=X负 但 推右=右转 → 两处一起改 / 或保持现状",
           recommend(-1, "forward", "right"),
           [SIGN_LINE, "两处一起改", "也可以就这么用"])

    expect("推上=打转 → 先对调那一路相线，不谈转向",
           recommend(+1, "spin", "left"), ["两根相线对调"],
           must_not=[SIGN_LINE, MIXER_LINE])

    expect("没接电机 + X 显示为负 → 只给读数结论并提醒后续实测",
           recommend(-1, "unknown", "unknown"),
           [SIGN_LINE, "必须再跑一次本脚本"])

    expect("改符号的建议里必须警告别手写负号（双重取反）",
           recommend(-1, "forward", "left"), ["别再去 mapAxis", "双重取反"])

    expect("没测准（X 落在死区附近）→ 让人重测，不给方向建议",
           recommend(0, "forward", "right"), ["没测准"],
           must_not=[MIXER_LINE, SIGN_LINE])

    # 轴向判决
    axis_rows_bad = rows(jx=1800) * 3
    lr = rows(jx=1800) * 5 + rows(jx=2700) * 5 + rows(jx=1200) * 5
    ud = rows(jx=1823) * 5
    a_ok = verdict_x_axis(lr, ud)
    expect("左右推 JX 有跨度 → 判为正常", [a_ok["text"]], ["正常"])
    swapped = verdict_x_axis([(t, 1828, v, 0, 0, 1500, 1500, True, False)
                              for t, v in
                              [(i * 0.01, 1200 + (i % 20) * 70) for i in range(20)]],
                             ud)
    expect("左右推时动的是 JY → 判为接线对调", [swapped["text"]],
           ["URX / URY 两根线对调"])
    dead = verdict_x_axis(axis_rows_bad, [(t, 2048, v, 0, 0, 1500, 1500, True, False)
                                          for t, v in
                                          [(i * 0.01, 1200 + (i % 20) * 70) for i in range(20)]])
    expect("JX 不动而 JY 正常 → 提示对调 URX/URY 来分清是哪一边",
           [dead["text"]], ["URX 和 URY", "两根杜邦线对调"])

    print(f"\n{'自测通过' if ok else '自测有失败项'}")
    return 0 if ok else 1


# ===========================================================================
#  命令行
# ===========================================================================

def pick_port() -> str | None:
    ports = [p for p in list_ports() if "Bluetooth" not in p and "debug-console" not in p]
    if not ports:
        return None
    if len(ports) == 1:
        return ports[0]
    print("找到多个串口，请用 --port 指定一个：")
    for p in ports:
        print("   ", p)
    return None


def main() -> int:
    p = argparse.ArgumentParser(
        description="摇杆/转向实机体检：先验链路，再验摇杆，最后给结论（只读串口）")
    p.add_argument("--port", help="ESP32 的串口，如 /dev/cu.usbserial-XXXX")
    p.add_argument("--selftest", action="store_true", help="不接硬件，只验判决逻辑")
    args = p.parse_args()

    if args.selftest:
        return selftest()

    port = args.port or pick_port()
    if not port:
        print("没找到可用的串口。用 --port 指定（参考 motor_sim.py --list 的输出）。")
        return 2

    print("=" * 62)
    print("摇杆 / 转向 实机体检")
    print("=" * 62)
    print("本脚本只读串口，不向板子发送任何东西。全程约 40 秒，按提示做动作即可。")

    holders = port_holders(port)
    if holders:
        print(f"\n！{port} 现在被这些程序占着：" + "、".join(holders))
        print("  macOS 允许多个程序同时打开同一个串口，字节会被瓜分，")
        print("  收到的行会残缺 —— 这样测出来的数据不能当证据。")
        if input("  还是继续吗？(y/N) >>> ").strip().lower() != "y":
            return 2

    rec = Recorder()
    reader = SerialReader(port, rec.on_line, lambda m: setattr(rec, "status", m))
    try:
        reader.start()
    except Exception as exc:
        print(f"\n打不开串口 {port}：{exc}")
        print("（端口号对不对？参考 python3 tools/motor_sim.py --list）")
        return 2

    try:
        if not phase_link(rec, reader, port):
            print("\n链路没过关，先不往下测 —— 前面两次就是拿这样的数据当证据，"
                  "结论全错。")
            return 2
        phase_center(rec)
        lr, ud = phase_axes(rec)
        hold = phase_hold(rec)
        fwd, turn = phase_turn()
    except KeyboardInterrupt:
        print("\n\n被打断了。")
        return 130
    finally:
        reader.close()

    axis = verdict_x_axis(lr, ud)
    report(axis, hold, fwd, turn)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

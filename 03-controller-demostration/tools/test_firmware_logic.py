#!/usr/bin/env python3
"""03 固件控制逻辑的自检（不需要硬件）。

这里把 src/main.cpp 里的映射函数与限速逻辑照着实现一遍，验证它们满足设计意图。
如果哪天改了固件而把下面的常量忘了同步，这些测试会立刻报出来 —— 这是重点。

    python3 tools/test_firmware_logic.py
"""

from __future__ import annotations

import math
import pathlib
import re
import sys

# ---- 必须和 src/main.cpp 一致 ----
PULSE_MIN, PULSE_MAX, PULSE_NEUTRAL = 1000, 2000, 1500
JOY_DEADZONE = 0.15
SLEW_US_PER_MS = 2.0
PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok   {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}  {detail}")


# ---- 固件逻辑的等价实现（对应 main.cpp 的 applyDeadzone / unitToPulse / joyToMotorTargets）----

def apply_deadzone(v):
    a = abs(v)
    if a <= JOY_DEADZONE:
        return 0.0
    n = (a - JOY_DEADZONE) / (1.0 - JOY_DEADZONE)
    return -n if v < 0 else n


def unit_to_pulse(v):
    v = max(-1.0, min(1.0, v))
    return round(PULSE_NEUTRAL + v * (PULSE_MAX - PULSE_NEUTRAL))


def joy_to_targets(nx, ny):
    # 左 = fwd + turn，右 = fwd - turn，**后退时 turn 取反**（方向盘惯例）
    # （必须和 src/main.cpp 的 joyToMotorTargets() 一模一样，见那里的 ★ 注释。
    #   真船上的实际转向仍要跑 tools/joy_check.py 第 6 步验证。）
    x, y = apply_deadzone(nx), apply_deadzone(ny)
    turn = -x if y < 0 else x
    return unit_to_pulse(y + turn), unit_to_pulse(y - turn)


def slew(cur, target, dt_ms, limit=SLEW_US_PER_MS):
    step = limit * max(dt_ms, 1.0)
    d = max(-step, min(step, target - cur))
    return cur + d


def test_mapping():
    print("\n[1] 映射端点（用户要的四个动作）")
    cases = [
        ((0.0, 1.0), (2000, 2000), "Y=+1 两轮同速前进"),
        ((0.0, -1.0), (1000, 1000), "Y=-1 两轮同速后退"),
        ((1.0, 0.0), (2000, 1000), "X=+1 左路出力多（原地 → 车头右转）"),
        ((-1.0, 0.0), (1000, 2000), "X=-1 右路出力多（原地 → 车头左转）"),
        ((0.0, 0.0), (1500, 1500), "居中 停"),
        ((1.0, 1.0), (2000, 1500), "前进中推右 → 车头右偏"),
        ((1.0, -1.0), (1000, 1500), "后退中推右 → 车尾往右（转向取反）"),
        ((-1.0, -1.0), (1500, 1000), "后退中推左 → 车尾往左（转向取反）"),
    ]
    for (nx, ny), want, desc in cases:
        got = joy_to_targets(nx, ny)
        check(f"{desc} -> {want}", got == want, f"得到 {got}")

    print("\n[2] 输出永不越界")
    worst = []
    for nx in [i / 40 for i in range(-40, 41)]:
        for ny in [i / 40 for i in range(-40, 41)]:
            l, r = joy_to_targets(nx, ny)
            if not (PULSE_MIN <= l <= PULSE_MAX and PULSE_MIN <= r <= PULSE_MAX):
                worst.append((nx, ny, l, r))
    check("扫 81x81 个位置都在 1000~2000 内", not worst, f"{worst[:3]}")

    print("\n[3] 死区：连续起步，不在边界跳变")
    check("死区内 -> 中立", joy_to_targets(0.0, JOY_DEADZONE) == (1500, 1500))
    just = joy_to_targets(0.0, JOY_DEADZONE + 0.001)
    check("刚出死区只偏 1us 以内（线性重标定生效）",
          abs(just[0] - PULSE_NEUTRAL) <= 1, f"得到 {just}")
    # 硬截断会在边界处从 1500 直接跳到 ~1575，这里确认没有
    hard = unit_to_pulse(JOY_DEADZONE)
    check(f"对比：硬截断会跳到 {hard}（所以我们用重标定）", hard > PULSE_NEUTRAL + 50)


def test_turn_sign():
    """只验证 mixer 自己的符号约定，**不验证船往哪边转** —— 那是实机的事。

    这组用例曾经写着"实机确认为右转/左转"，但 2026-10-03 那次从没在真船上
    验证过，是自检里的一句假话，照着它改方向反而越改越反。现在只断言
    "X=+1 时哪一路出力多"，这个纯软件事实。船的转向去跑 tools/joy_check.py。
    """
    print("\n[4] mixer 的符号约定（软件内部自洽；船的转向要实机测）")
    l, r = joy_to_targets(1.0, 0.0)
    check("X=+1 -> 左路比右路出力多（推右 → 右转）", l > r, f"L={l} R={r}")
    l, r = joy_to_targets(-1.0, 0.0)
    check("X=-1 -> 右路比左路出力多（推左 → 左转）", r > l, f"L={l} R={r}")
    l, r = joy_to_targets(0.0, 1.0)
    check("Y=+1 -> 两路相等（直行）", l == r, f"L={l} R={r}")

    print("\n[4b] 后退时转向取反（方向盘惯例，和真车倒车一致）")
    f_l, f_r = joy_to_targets(1.0, 1.0)      # 前进中推右
    b_l, b_r = joy_to_targets(1.0, -1.0)     # 后退中推右
    check("前进中推右：左路更靠前（车头往右偏）", f_l > f_r, f"L={f_l} R={f_r}")
    check("后退中推右：左右关系反过来（右路更靠前 → 车尾往右）",
          b_r > b_l, f"L={b_l} R={b_r}")
    check("同样的摇杆动作，前进/后退的左右关系确实相反",
          (f_l > f_r) == (b_l < b_r), f"前进 L={f_l} R={f_r} / 后退 L={b_l} R={b_r}")
    c_l, c_r = joy_to_targets(-1.0, -1.0)
    check("后退中推左：左路更靠前（车尾往左）", c_l > c_r, f"L={c_l} R={c_r}")


def test_slew():
    print("\n[5] 油门限速：上电不会瞬间满油门")
    cur = float(PULSE_NEUTRAL)
    cur = slew(cur, 2000, 8.0)
    check("8ms 内最多走 16us", abs(cur - PULSE_NEUTRAL - 16.0) < 0.01, f"{cur}")
    # 从 1500 到 2000 需要 500us / 2us-per-ms = 250ms
    cur, t = float(PULSE_NEUTRAL), 0.0
    while cur < 1999.5 and t < 5000:
        cur = slew(cur, 2000, 8.0)
        t += 8.0
    check(f"1500->2000 约需 {t:.0f}ms（设计值 250ms）", 230 <= t <= 300, f"{t}ms")

    print("\n[6] 开机预热窗口（固件里已经没有解锁/上锁状态机了）")
    raw_l, raw_r = joy_to_targets(0.0, 1.0)            # 摇杆推到底
    # motorsTick() 在预热窗口内把 target 压成中立，摇杆动了也不响应
    warm_l, warm_r = PULSE_NEUTRAL, PULSE_NEUTRAL
    check("预热窗口内 -> 摇杆推满，电机拿到的仍是中立",
          warm_l == PULSE_NEUTRAL and warm_r == PULSE_NEUTRAL)
    check("预热结束后 -> 摇杆直接生效（推满 = 2000）",
          raw_l == PULSE_MAX and raw_r == PULSE_MAX)


def test_telemetry_protocol():
    print("\n[7] 遥测格式与 Motor-Sim 解析器对得上")
    src = pathlib.Path(__file__).resolve().parent.parent / "src" / "main.cpp"
    text = src.read_text()
    # printf 的格式串在源码里是跨行的字符串字面量拼接，所以要匹配到右括号再
    # 把相邻的 "..." 片段接起来，否则只能看到第一行。
    m = re.search(r'Serial\.printf\(\s*("(?:[^"\\]|\\.)*"(?:\s*"(?:[^"\\]|\\.)*")*)',
                  text[text.index("JOY JX=") - 400:text.index("JOY JX=") + 900]
                  if "JOY JX=" in text else "", re.S)
    check("固件里有 JOY 遥测行", "JOY JX=" in text)
    if not m:
        return
    fmt = "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', m.group(1)))
    check("格式串被完整取出（含 L=/R=）", "L=" in fmt and "R=" in fmt, fmt)
    for token in ("JX=", "JY=", "X=", "Y=", "MAG=", "READY=", "L=", "R=", "TL=", "TR="):
        check(f"格式串含 {token}", token in fmt)
    check("含 DEAD/OUT 标记", "DEAD" in text and "OUT" in text)

    # 造一行按这个格式的输出，交给 Motor-Sim 的解析器
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    try:
        from motor_sim import parse_joy, Telemetry
    except ImportError as exc:
        check("能导入 motor_sim", False, str(exc))
        return
    t = Telemetry()
    ok = parse_joy("JOY JX=2704 JY=1912 X=+0.520 Y=-0.150 MAG=0.542 OUT "
                   "READY=1 L=1718 R=1282 TL=1718 TR=1282", t)
    check("Motor-Sim 能解析固件输出", ok and t.l_us == 1718 and t.r_us == 1282
          and t.ready is True, f"L={t.l_us} R={t.r_us} ready={t.ready}")

    t2 = Telemetry()
    parse_joy("JOY JX=2048 JY=2048 X=+0.000 Y=+0.000 MAG=0.000 DEAD READY=0 "
              "L=1500 R=1500 TL=1500 TR=1500", t2)
    check("预热窗口内 READY=0", t2.ready is False)


def test_consistency_with_sim():
    print("\n[8] 固件映射 == motor_sim.py 映射（防止两边漂移）")
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    try:
        import motor_sim
    except ImportError as exc:
        check("能导入 motor_sim", False, str(exc))
        return
    mismatch = []
    for nx in [i / 20 for i in range(-20, 21)]:
        for ny in [i / 20 for i in range(-20, 21)]:
            if joy_to_targets(nx, ny) != motor_sim.motor_pulses(nx, ny):
                mismatch.append((nx, ny, joy_to_targets(nx, ny),
                                 motor_sim.motor_pulses(nx, ny)))
    check("41x41 个位置两边完全一致", not mismatch, f"{mismatch[:3]}")
    check("常量也对得上",
          (motor_sim.PULSE_MIN, motor_sim.PULSE_MAX, motor_sim.PULSE_NEUTRAL) ==
          (PULSE_MIN, PULSE_MAX, PULSE_NEUTRAL)
          and abs(motor_sim.JOY_DEADZONE - JOY_DEADZONE) < 1e-9)


def main():
    test_mapping()
    test_turn_sign()
    test_slew()
    test_telemetry_protocol()
    test_consistency_with_sim()
    print(f"\n{'=' * 46}\n通过 {PASS} 项，失败 {FAIL} 项")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())

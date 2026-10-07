#!/usr/bin/env python3
"""Motor-Sim —— 摇杆双电机遥控器的电脑端平面模拟器（只读）。

它做什么
--------
从 03 固件的串口遥测里读摇杆和左右脉宽，在电脑上把这个「略微长方形的物体」
按差速运动学跑起来，让你在接电机之前先看清运动对不对。

    ESP32 (03 固件)  ──── JOY ... L=.. R=.. ────>  本程序（只画，不回写）

**本程序只读，不向板子发送任何东西**。电机是 03 固件自己直接驱动的
（GPIO4/5），电脑这边纯粹是监视器 + 平面仿真。

为什么不用 pyserial
-------------------
用标准库的 termios 直接配置串口就够了，所以这个脚本**零第三方依赖**，
任何 Python 3.9+ 都能跑。

用法
----
    # 看有哪些串口
    python3 tools/motor_sim.py --list

    # 接上板子，实时仿真
    python3 tools/motor_sim.py --port /dev/cu.usbserial-A5069RR4

    # 没接板子也能看界面：本地造数据（WASD 控制模拟摇杆）
    python3 tools/motor_sim.py --mock

按键
----
    W A S D   仅 --mock 下用来模拟摇杆
    0         复位到原点
    T         清空航迹
    Q / Esc   退出

--mock 照着固件的行为来：**上电有一个 3 秒预热窗口**（BOOT_IGNORE_MS），
这期间按 WASD 车不动、输出固定 1500；窗口一过就一直有效，**不需要按任何键**。
固件里已经没有"解锁/上锁"状态机了。

自己占多少 CPU，看窗口右边面板里的「本程序 CPU」那一行：空闲时应该接近 0%，
如果一直贴着 100%，说明串口读取又变成忙等了。
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys
from collections import deque
from dataclasses import dataclass, field

# ---- 必须和 src/main.cpp 保持一致 ----
PULSE_MIN = 1000
PULSE_MAX = 2000
PULSE_NEUTRAL = 1500
JOY_DEADZONE = 0.15
BOOT_IGNORE_MS = 3000      # 和固件的 BOOT_IGNORE_MS 一致

# 仿真物理参数（只影响画面，不影响板子）
WHEELBASE_M = 0.9
MAX_LINEAR_MPS = 0.9
ACCEL_TAU_S = 0.22
TELEMETRY_STALE_S = 0.5      # 多久没数据算断流（画面上标红）
VIEW_HALF_M = 2.4            # 视野半径（米）


# ===========================================================================
#  1. 映射（和固件逐行对应，用来自检 / --mock 造数据）
# ===========================================================================

def apply_deadzone(v: float, deadzone: float = JOY_DEADZONE) -> float:
    """死区 + 线性重标定（硬截断会在死区边界跳变，所以必须重标定）。"""
    a = abs(v)
    if a <= deadzone:
        return 0.0
    n = (a - deadzone) / (1.0 - deadzone)
    return -n if v < 0 else n


def motor_pulses(nx: float, ny: float) -> tuple[int, int]:
    """摇杆 -> (左脉宽, 右脉宽)。

    左 = fwd + turn，右 = fwd - turn（和固件逐字一致）。

    **后退时 turn 取反**（方向盘惯例）：前进时推右，车头往右偏；后退时推右，
    车尾往右走 —— 和真车倒车一致。不取反就是"坦克惯例"：不管前进后退，
    车体总是顺时针转。两者都自洽，固件里选的是方向盘惯例，这里必须跟它一致。
    """
    def clamp(v):
        return int(round(PULSE_NEUTRAL + max(-1.0, min(1.0, v)) * (PULSE_MAX - PULSE_NEUTRAL)))
    x = apply_deadzone(nx)
    y = apply_deadzone(ny)
    turn = -x if y < 0 else x
    return clamp(y + turn), clamp(y - turn)


def scene_fingerprint(robot: "Robot", body: str, note: str, waiting: bool) -> tuple:
    """画面的指纹：这些量里任何一个变了，画面才需要重画。

    取整是关键：小车停下时 `v` 是指数衰减、永远不精确等于 0，位置会一直
    以微米级漂移。取整到人眼看不出来的精度，静止画面才能真的不动 ——
    否则 33 帧/秒的无脑重画只是换了个形式而已。
    """
    return (round(robot.x, 3), round(robot.y, 3), round(robot.theta, 3),
            round(robot.v, 2), len(robot.trail), body, note, waiting)


def pulses_to_thrust(us: int) -> float:
    """脉宽 -> 推力(-1~+1)，1500 为 0。"""
    if us >= PULSE_NEUTRAL:
        return min(1.0, (us - PULSE_NEUTRAL) / (PULSE_MAX - PULSE_NEUTRAL))
    return max(-1.0, (us - PULSE_NEUTRAL) / (PULSE_NEUTRAL - PULSE_MIN))


# ===========================================================================
#  2. 差速运动学
# ===========================================================================

@dataclass
class Robot:
    """略微长方形的小车。x 向右、y 向上，theta=0 朝 +x，逆时针为正。

    注意屏幕 y 是向下的，画的时候要翻一下。
    """
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    v: float = 0.0
    wheelbase: float = WHEELBASE_M
    max_speed: float = MAX_LINEAR_MPS
    accel_tau: float = ACCEL_TAU_S
    trail: deque = field(default_factory=lambda: deque(maxlen=1200))
    _last: tuple | None = None

    def reset(self):
        self.x = self.y = self.theta = self.v = 0.0
        self.trail.clear()
        self._last = None

    def clear_trail(self):
        self.trail.clear()
        self._last = None

    def step(self, l_us: int, r_us: int, dt: float):
        l, r = pulses_to_thrust(l_us), pulses_to_thrust(r_us)
        target_v = 0.5 * (l + r) * self.max_speed

        # 角速度用标准差速公式：右路出力多 -> 逆时针(正) -> 屏幕上是左转。
        #
        # ★ 这一条**还没有和真硬件校准过**（2026-10-03 那次左右转向一直没对准，
        #   因为当时拿这个模拟器当选真值了，方法本来就是错的 —— 自己出题自己判卷）。
        #   正确做法：接上电机跑 tools/joy_check.py，按真船上看到的现象决定
        #   这里和 motor_pulses() 的符号要不要反过来，两边必须一起改。
        omega = (r - l) * self.max_speed / self.wheelbase

        alpha = 1.0 if self.accel_tau <= 0 else min(1.0, dt / self.accel_tau)
        self.v += (target_v - self.v) * alpha

        self.theta = math.atan2(math.sin(self.theta + omega * dt),
                                math.cos(self.theta + omega * dt))
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt

        if self._last is None or math.hypot(self.x - self._last[0],
                                            self.y - self._last[1]) > 0.02:
            self.trail.append((self.x, self.y))
            self._last = (self.x, self.y)

    def turn_rate_deg(self, l_us: int, r_us: int) -> float:
        """转动角速度(°/s)，符号与 step() 一致。"""
        l, r = pulses_to_thrust(l_us), pulses_to_thrust(r_us)
        return math.degrees((r - l) * self.max_speed / self.wheelbase)


# ===========================================================================
#  3. 遥测解析
# ===========================================================================

class Telemetry:
    def __init__(self):
        self.raw_x = 2048
        self.raw_y = 2048
        self.x = 0.0
        self.y = 0.0
        self.mag = 0.0
        self.in_dead = True
        self.ready = False
        self.l_us = PULSE_NEUTRAL
        self.r_us = PULSE_NEUTRAL
        self.tl_us = PULSE_NEUTRAL
        self.tr_us = PULSE_NEUTRAL
        # 输入来源（PC_CONTROL 模式加的字段，老固件不发这几个 token）
        self.pc = False        # True = 输入来自电脑上的手柄
        self.en = True         # 安全键按住 / 允许出力
        self.link = True       # 输入在线（PC 模式下 = 没断流）
        self.seen = False


def parse_joy(line: str, t: Telemetry) -> bool:
    """解析一行 JOY 遥测。格式：

    JOY JX=.. JY=.. X=.. Y=.. MAG=.. DEAD|OUT LOCK=0|1 HOLD=0|1 L=.. R=.. TL=.. TR=..

    故意写得宽松：以后固件加字段也不会把这里弄坏。
    """
    parts = line.split()
    if not parts or parts[0].upper() != "JOY":
        return False

    kv: dict[str, str] = {}
    bare: set[str] = set()
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            kv[k.upper()] = v
        else:
            bare.add(p.upper())

    def num(key, default):
        try:
            return int(kv[key])
        except (KeyError, ValueError):
            return default

    def flt(key, default):
        try:
            return float(kv[key])
        except (KeyError, ValueError):
            return default

    t.raw_x = num("JX", t.raw_x)
    t.raw_y = num("JY", t.raw_y)
    t.x = flt("X", t.x)
    t.y = flt("Y", t.y)
    t.mag = flt("MAG", t.mag)
    t.in_dead = "DEAD" in bare
    # 新版固件没有解锁状态机了：只有一个 READY（开机预热窗口过了没）。
    # 老字段名也认，方便看旧日志。
    t.ready = num("READY", num("LOCK", 1)) == 1 and num("HOLD", 0) == 0
    t.l_us = num("L", PULSE_NEUTRAL)
    t.r_us = num("R", PULSE_NEUTRAL)
    t.tl_us = num("TL", t.l_us)
    t.tr_us = num("TR", t.r_us)
    # PC/EN/LINK 是 PC_CONTROL 模式追加在行尾的字段。老固件不发 ->
    # 默认 pc=False / en=True / link=True，行为和历史版本一致。
    t.pc = num("PC", 0) == 1
    t.en = num("EN", 1) == 1
    t.link = num("LINK", 1) == 1
    t.seen = True
    return True


# ===========================================================================
#  4. 串口读取（标准库 termios，零依赖）
# ===========================================================================

def list_ports() -> list[str]:
    """列出可能是 ESP32 的串口设备。用 dict 去重并保持顺序：
    下面的 glob 模式有重叠（cu.usb* 也会匹配 cu.usbserial*），不去重会重复打印。"""
    dev = pathlib.Path("/dev")
    pats = ("cu.usb*", "cu.wch*", "cu.SLAB*", "cu.usbserial*", "cu.usbmodem*")
    found: dict[str, None] = {}
    for pat in pats:
        for p in sorted(dev.glob(pat)):
            found[str(p)] = None
    return list(found)


def port_holders(port: str | None) -> list[str]:
    """谁正占着这个串口。macOS 允许多个进程同时打开同一个 /dev/cu.*，
    **字节会被随机瓜分**（两边都收到残缺的行），所以读之前必须先查这个。

    没有端口（--mock 纯模拟）时返回空列表 —— 别把 None 塞给 lsof，
    那会抛 TypeError，而纯模拟本来就跟串口无关。"""
    import subprocess
    if not port:
        return []
    try:
        out = subprocess.run(["lsof", port], capture_output=True, text=True,
                             timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    names = []
    for line in out.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 2 and parts[1].isdigit():
            names.append(f"{parts[0]} (pid {parts[1]})")
    return names


class SerialReader:
    """后台线程按行读串口。用 termios 配成 raw + 115200，不需要 pyserial。"""

    def __init__(self, port: str, on_line, on_status):
        self.fd = -1
        self.buf = b""
        self.bytes_rx = 0        # 累计收到的字节数（判断"到底是没数据还是全是垃圾"）
        self.lines_rx = 0        # 累计收到的完整行数
        self._on_line = on_line
        self._on_status = on_status
        self._stop = False
        self._thread = None
        self.port = port

    def _open(self):
        import termios
        fd = os.open(self.port, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY)
        try:
            attrs = termios.tcgetattr(fd)
            # iflag, oflag, cflag, lflag, ispeed, ospeed, cc
            attrs[0] = 0                      # 关掉所有输入处理（不做换行转换）
            attrs[1] = 0
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0                      # 非规范模式、不回显
            # ★ 必须显式设 115200。串口的波特率是**设备上残留的**：上一个程序
            #   设成多少就留着多少，刚插上时可能是驱动默认的 9600。9600 读
            #   115200 的数据只会得到乱码 —— 症状是"能收到字节，但没有一行
            #   是完整的"，曾经被误判成硬件故障。
            attrs[4] = attrs[5] = termios.B115200
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
            termios.tcflush(fd, termios.TCIFLUSH)
        except Exception:
            os.close(fd)
            raise
        return fd

    def start(self):
        import threading
        self.fd = self._open()
        self._on_status(f"已打开 {self.port}")
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        import select
        import time
        while not self._stop:
            # ★ 用 select 等数据，不是拿 os.read 反复去问。
            #   "读不到就 continue" 是忙等：实测能吃掉一整个核（99%），
            #   这就是之前风扇狂转的原因。select 让内核在没数据时把线程挂起，
            #   同一个循环实测 0%。
            try:
                ready, _, _ = select.select([self.fd], [], [], 0.5)
            except (OSError, ValueError):
                if not self._stop:
                    self._on_status("串口已断开")
                return
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, 512)
            except BlockingIOError:
                time.sleep(0.005)      # 个别驱动的假唤醒，兜一下，不是主路径
                continue
            except OSError as exc:
                if not self._stop:
                    self._on_status(f"串口读取失败：{exc}")
                return
            if not chunk:
                continue
            self.bytes_rx += len(chunk)
            self.buf += chunk
            while b"\n" in self.buf:
                raw, self.buf = self.buf.split(b"\n", 1)
                line = raw.decode("utf-8", errors="replace").strip()
                if line:
                    self.lines_rx += 1
                    self._on_line(line)

    def close(self):
        self._stop = True
        if self._thread is not None:
            self._thread.join(timeout=0.6)
        if self.fd >= 0:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = -1


# ===========================================================================
#  5. 本地造数据（--mock）
# ===========================================================================

class MockSource:
    """没接板子时用的假固件：照搬固件的预热窗口和油门限速。

    它照着固件的行为来：**上电后有一个预热窗口**（BOOT_IGNORE_MS），
    这期间摇杆不响应、输出中立；窗口一过就一直有效，不需要按任何键。

    这样即使没有硬件，也能把预热、运动学和界面都验证一遍；
    只有"真电机转不转"验证不了。
    """

    def __init__(self):
        self.keys: set[str] = set()
        self.now = 0.0
        self.ready_at = BOOT_IGNORE_MS / 1000.0   # 固件的 BOOT_IGNORE_MS
        self.ready = False
        self.out_l = float(PULSE_NEUTRAL)
        self.out_r = float(PULSE_NEUTRAL)

    def press(self, key: str, down: bool):
        key = key.lower()
        if down:
            self.keys.add(key)
        else:
            self.keys.discard(key)

    def sample(self, dt: float) -> str:
        self.now += dt
        if not self.ready and self.now >= self.ready_at:
            self.ready = True

        tx = (1.0 if "d" in self.keys else 0.0) - (1.0 if "a" in self.keys else 0.0)
        ty = (1.0 if "w" in self.keys else 0.0) - (1.0 if "s" in self.keys else 0.0)
        nx = max(-1.0, min(1.0, tx))
        ny = max(-1.0, min(1.0, ty))
        raw_x = int(2048 + nx * 1535)
        raw_y = int(2048 + ny * 1535)
        mag = math.hypot(nx, ny)
        dead = mag < JOY_DEADZONE

        tl, tr = motor_pulses(nx, ny)
        if not self.ready:                      # 预热窗口内不响应
            tl = tr = PULSE_NEUTRAL
        # 固件的 SLEW_US_PER_MS = 2.0
        max_step = 2.0 * max(dt * 1000.0, 1.0)
        for attr, target in (("out_l", tl), ("out_r", tr)):
            cur = getattr(self, attr)
            d = max(-max_step, min(max_step, target - cur))
            setattr(self, attr, cur + d)

        return (f"JOY JX={raw_x} JY={raw_y} X={nx:+.3f} Y={ny:+.3f} "
                f"MAG={mag:.3f} {'DEAD' if dead else 'OUT'} "
                f"READY={1 if self.ready else 0} "
                f"L={round(self.out_l)} R={round(self.out_r)} TL={tl} TR={tr}")


# ===========================================================================
#  6. 界面
# ===========================================================================

class App:
    def __init__(self, args):
        import tkinter as tk
        import time
        self.tk = tk
        self.args = args
        self.robot = Robot()
        self.telem = Telemetry()
        self.status = "启动中"
        self.last_rx = 0.0
        self.started_at = time.monotonic()
        self.cpu_start = time.process_time()
        self.running = True
        self.ticks = 0
        self.redraws = 0          # 真正重画的次数（和 tick 次数比较，看省了多少）
        self.reader = None
        self.holders = []         # 打开时正占着串口的别的程序
        self._holders_at = 0.0    # 上次重查占用情况的时刻
        self.joy_rx = 0           # 解析成功的 JOY 行数（区别于"收到的字节数"）
        self.mock = MockSource() if args.mock else None
        self._last_tick = None
        self._scene_key = None    # 画面指纹：没变就不重画
        self._panel_key = None
        self._build()

    # ---------- 界面搭建 ----------
    def _build(self):
        tk = self.tk
        self.root = tk.Tk()
        self.root.title("Motor-Sim · 摇杆双电机平面模拟器（只读）")
        self.root.protocol("WM_DELETE_WINDOW", self.quit)

        pad = 10
        outer = tk.Frame(self.root)
        outer.pack(padx=pad, pady=pad)

        self.canvas = tk.Canvas(outer, width=720, height=560, bg="#0d1117",
                                highlightthickness=1, highlightbackground="#30363d")
        self.canvas.grid(row=0, column=0, sticky="n")

        side = tk.Frame(outer)
        side.grid(row=0, column=1, sticky="n", padx=(pad, 0))
        self.text = tk.Text(side, width=48, height=34, bg="#0d1117", fg="#c9d1d9",
                            font=("Menlo", 11), relief="flat", highlightthickness=1,
                            highlightbackground="#30363d")
        self.text.pack()
        keys = ("--mock 模式可用的键：\n"
                "  W A S D  推/拉模拟摇杆\n"
                "  0 复位   T 清航迹   Q/Esc 退出\n\n"
                if self.args.mock else
                "按键：\n"
                "  0 复位   T 清航迹   Q/Esc 退出\n"
                "  （摇杆本身不用按键，预热完就生效）\n\n")
        tk.Label(side, justify="left", font=("Menlo", 10), fg="#8b949e",
                 bg="#0d1117",
                 text=(keys +
                       "本程序只读：不回写任何指令。\n"
                       "电机由 03 固件自己驱动。")
                 ).pack(anchor="w", pady=(6, 0))

        self.root.bind("<KeyPress>", lambda e: self.on_key(e, True))
        self.root.bind("<KeyRelease>", lambda e: self.on_key(e, False))
        self.canvas.focus_set()

    # ---------- 串口 ----------
    def connect(self):
        if self.args.mock:
            self.status = "本地造数据（--mock）"
            return
        self.holders = port_holders(self.args.port)
        if self.holders:
            # 不拦着不让跑：只是把"数据古怪"的原因摆在用户眼前，省得又去怀疑硬件
            self.status = "串口被占：" + "、".join(self.holders)
        try:
            self.reader = SerialReader(self.args.port, self.on_line,
                                       self._set_status)
            self.reader.start()
        except Exception as exc:
            self.status = f"打不开串口：{exc}"
            self.reader = None

    def _set_status(self, msg):
        # 端口被别人占着的时候，这条警告比"已打开 xxx"重要，别被盖掉
        if not self.holders:
            self.status = msg

    def on_line(self, line: str):
        import time
        if parse_joy(line, self.telem):
            self.last_rx = time.monotonic()
            self.joy_rx += 1

    def _refresh_holders(self, now):
        """隔几秒重查一次串口占用。

        占用是**动态**的：用户随时可能把另一个程序关掉。只在启动时查一次的话，
        面板会一直挂着「串口被占」这句已经过时的话，明明冲突早就没了 ——
        反过来又会让人以为有问题。"""
        if self.mock is not None or self.reader is None:
            return
        if now - self._holders_at < 3.0:
            return
        self._holders_at = now
        live = port_holders(self.args.port)
        if live == self.holders:
            return
        self.holders = live
        self.status = ("串口被占：" + "、".join(live)) if live else \
            "串口已独占（之前占用的程序退出了）"

    def _link_watchdog(self, now):
        """4 秒还解析不出一行 JOY，就把"到底卡在哪"直接写在面板上。

        这一步是为了不再重犯一个错误：拿到残缺/乱码数据时以为串口正常，
        然后把它当成硬件故障的证据。有数据 ≠ 数据是对的。
        """
        if self.mock is not None or self.reader is None or self.telem.seen:
            return
        if now - self.started_at < 4.0 or self.holders:
            return
        r = self.reader
        if r.bytes_rx == 0:
            self.status = "完全没有数据：板子在跑吗？插的是标 UART 的那个口吗？"
        elif r.lines_rx == 0:
            self.status = (f"收到 {r.bytes_rx} 字节但一行都没凑齐：波特率不对"
                           "（刚插上时可能是 9600）或串口被抢了")
        else:
            self.status = (f"收到 {r.lines_rx} 行但没有一行是固件遥测："
                           "烧的可能是别的固件，或波特率不对")

    def on_key(self, event, down: bool):
        key = (event.keysym or "").lower()
        if self.mock is not None and key in ("w", "a", "s", "d"):
            self.mock.press(key, down)
            return
        if not down:
            return
        if key in ("q", "escape"):
            self.quit()
        elif key == "0":
            self.robot.reset()
        elif key == "t":
            self.robot.clear_trail()

    def quit(self):
        self.running = False

    # ---------- 主循环 ----------
    def tick(self):
        import time
        now = time.monotonic()
        if self._last_tick is None:
            self._last_tick = now
        dt = min(0.1, now - self._last_tick)     # 卡顿时别让积分炸掉
        self._last_tick = now

        if self.mock is not None:
            parse_joy(self.mock.sample(dt), self.telem)
            self.last_rx = now

        # 用固件报的实际输出脉宽来跑仿真：屏幕上的运动和电机拿到的完全一致
        self.robot.step(self.telem.l_us, self.telem.r_us, dt)
        self.ticks += 1

        # tick() 必须一直按 30ms 跑：串口解析和物理积分都挂在这上面。
        # 省 CPU 靠的是"跳过重画"，不是减少 tick。
        self._refresh_holders(now)
        self._link_watchdog(now)
        self.draw(now)
        if self.running:
            self.root.after(30, self.tick)
        else:
            self.shutdown()

    # ---------- 绘制 ----------
    def world_to_screen(self, wx, wy):
        cw = int(self.canvas["width"])
        ch = int(self.canvas["height"])
        scale = min(cw, ch) / (2.0 * VIEW_HALF_M)
        return cw / 2 + wx * scale, ch / 2 - wy * scale      # y 翻转

    def _state(self, now):
        """当前的状态色和横幅文案。指纹要用到它们，所以单独算出来。"""
        stale = self.telem.seen and (now - self.last_rx) > TELEMETRY_STALE_S
        if stale or not self.telem.seen:
            body, note = "#f85149", "没有遥测：检查串口 / 是否插错 Type-C 口"
        elif not self.telem.ready:
            # 开机预热窗口。这期间摇杆被忽略、输出中立 —— 最容易被误会成
            # "摇杆坏了"，所以直接在画面上写清楚还要等多久。
            body, note = "#d29922", f"开机预热中（约 {BOOT_IGNORE_MS/1000:.0f} 秒）：摇杆暂不响应，马上就好"
        elif self.telem.in_dead:
            # PC 模式下 DEAD 的含义是"没有在出力"：断流 / 安全键没按住 / 摇杆在中点
            body, note = "#8b949e", ("READY：没有在出力（断流、安全键松开、或摇杆在中点）"
                                     if self.telem.pc else
                                     "READY：摇杆在死区内，电机中立")
        else:
            body, note = "#3fb950", "READY：电机受控"
        if self.mock is not None:
            note += "　（--mock 模拟数据）"
        waiting = (not self.telem.seen) or (not self.telem.ready)
        return body, note, waiting

    def draw(self, now):
        """按 30ms 被调用，但**只在画面真的变了的时候才重画**。

        以前是每帧无脑 delete("all") + 全量重画，摇杆不动时也在画，
        光 WindowServer 就要吃不少 CPU。现在先算一个"画面指纹"：
        指纹一样就直接跳过，肉眼看不到的变化（小数点后三位以内）不算变化。
        """
        body, note, waiting = self._state(now)
        key = scene_fingerprint(self.robot, body, note, waiting)
        if key != self._scene_key:
            self._scene_key = key
            self._draw_scene(body, note, waiting)
            self.redraws += 1
        self.draw_panel(now)

    def _draw_scene(self, body, note, waiting):
        c = self.canvas
        c.delete("all")
        cw, ch = int(c["width"]), int(c["height"])

        # 网格 0.5m
        n = int(VIEW_HALF_M / 0.5)
        for k in range(-n, n + 1):
            sx, sy = self.world_to_screen(k * 0.5, k * 0.5)
            c.create_line(sx, 0, sx, ch, fill="#1b2230")
            c.create_line(0, sy, cw, sy, fill="#1b2230")
        # 原点
        ox, oy = self.world_to_screen(0, 0)
        c.create_line(ox - 8, oy, ox + 8, oy, fill="#3d4757")
        c.create_line(ox, oy - 8, ox, oy + 8, fill="#3d4757")

        # 航迹
        if len(self.robot.trail) > 1:
            pts = []
            for (tx, ty) in self.robot.trail:
                pts.extend(self.world_to_screen(tx, ty))
            c.create_line(*pts, fill="#2f6f8f", width=2, smooth=True)

        # 车体：略微长方形 0.34 x 0.22 m
        hl, hw = 0.17, 0.11
        th = self.robot.theta
        ct, st = math.cos(th), math.sin(th)
        poly = []
        for (px, py) in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)):
            wx = self.robot.x + px * ct - py * st
            wy = self.robot.y + px * st + py * ct
            poly.extend(self.world_to_screen(wx, wy))
        c.create_polygon(*poly, outline=body, fill="#161b22", width=2)

        # 机头箭头
        ax, ay = self.world_to_screen(self.robot.x + hl * ct,
                                      self.robot.y + hl * st)
        bx, by = self.world_to_screen(self.robot.x, self.robot.y)
        c.create_line(bx, by, ax, ay, fill=body, width=2, arrow="last")

        # 左右侧标记（红=左，蓝=右），和面板上的 L/R 对应
        for sign, colour in ((-hw, "#f85149"), (hw, "#58a6ff")):
            wx = self.robot.x - sign * st
            wy = self.robot.y + sign * ct
            px, py = self.world_to_screen(wx, wy)
            c.create_oval(px - 3, py - 3, px + 3, py + 3, fill=colour, outline="")

        c.create_text(10, 10, anchor="nw", fill="#8b949e", font=("Menlo", 10),
                      text=f"x={self.robot.x:+.2f}m  y={self.robot.y:+.2f}m  "
                           f"朝向={math.degrees(self.robot.theta):+.0f}°  "
                           f"速度={self.robot.v:+.2f}m/s")

        # 状态横幅：预热/断流时用大号字压在画面正上方，避免"怎么不动"的误会
        c.create_text(cw / 2, 30, fill=body,
                      font=("Menlo", 13 if waiting else 11, "bold"),
                      text=note)

    def draw_panel(self, now):
        import time
        t = self.telem
        age = (now - self.last_rx) if t.seen else -1
        state = "READY" if t.ready else "WAIT"

        def bar(us, width=21):
            half = (width - 1) // 2
            cells = ["-"] * width
            cells[half] = "|"
            n = int(round((us - PULSE_NEUTRAL) / (PULSE_MAX - PULSE_NEUTRAL) * half))
            n = max(-half, min(half, n))
            for i in range(1, abs(n) + 1):
                cells[half + (i if n > 0 else -i)] = "="
            return "".join(cells)

        # 面板里的会变的量都取整到"人眼能分辨的程度"，否则每 30ms 都在改文本，
        # 指纹就永远失效了。
        elapsed = max(1e-6, now - self.started_at)
        cpu_pct = int(round((time.process_time() - self.cpu_start) / elapsed * 100))
        redraw_hz = self.redraws / elapsed
        age_s = "未收到" if age < 0 else f"{age:.1f} s 前"

        lines = [
            "== 链路 ==",
            f"  端口    {'--mock 本地' if self.mock else self.args.port}",
            f"  状态    {self.status}",
            f"  遥测    {age_s}",
            f"  本程序 CPU {cpu_pct:>4d}%   重画 {redraw_hz:.0f}/秒",
            "",
            "== 输入 ==",
            ("  来源    电脑手柄（pad_bridge 发来的 nx/ny）"
             if t.pc else f"  来源    板载摇杆  JX={t.raw_x:<5d} JY={t.raw_y:<5d}"),
            f"  归一化  X={t.x:+.3f}  Y={t.y:+.3f}",
            f"  合力    {t.mag:.3f}  {'DEAD 死区' if t.in_dead else 'OUT 出死区'}",
            (f"  安全键  {'按住' if t.en else '松开→中立'}"
             f"   链路 {'在线' if t.link else '断流→中立'}") if t.pc else "",
            "",
            "== 电机（板子实际输出）==",
            f"  预热    {state}",
            f"  左 L    {t.l_us:>4d}us {bar(t.l_us)} {pulses_to_thrust(t.l_us):+.2f}",
            f"  右 R    {t.r_us:>4d}us {bar(t.r_us)} {pulses_to_thrust(t.r_us):+.2f}",
            f"  目标    TL={t.tl_us} TR={t.tr_us}",
            f"  转弯    {self.robot.turn_rate_deg(t.l_us, t.r_us):+.0f} °/s",
            "",
            "== 参考（映射表，和固件同一套）==",
            f"  推上 Y=+1 -> {motor_pulses(0, 1)[0]}/{motor_pulses(0, 1)[1]} 两路同速前进",
            f"  推右 X=+1 -> {motor_pulses(1, 0)[0]}/{motor_pulses(1, 0)[1]} 左路多 → 右转",
            f"  推左 X=-1 -> {motor_pulses(-1, 0)[0]}/{motor_pulses(-1, 0)[1]} 右路多 → 左转",
            f"  后退中推右 -> {motor_pulses(1, -1)[0]}/{motor_pulses(1, -1)[1]} 车尾往右",
            "  （后退时左右取反 —— 方向盘惯例，同真车倒车）",
            "  ↑ 真船方向待实测：接上电机跑 tools/joy_check.py",
            "",
            "  只读：不回写任何指令。",
            f"  来源：{'本地造的（--mock）' if self.mock else '板子串口遥测'}",
        ]
        if lines == self._panel_key:
            return                     # 文本没变就别重排一次 Text 控件
        self._panel_key = lines
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        self.text.insert("1.0", "\n".join(lines))
        self.text.configure(state="disabled")

    def shutdown(self):
        if getattr(self, "reader", None) is not None:
            self.reader.close()
        self.root.destroy()

    def run(self):
        self.connect()
        self.root.after(30, self.tick)
        self.root.mainloop()


# ===========================================================================
#  7. 命令行
# ===========================================================================

def build_parser():
    p = argparse.ArgumentParser(
        description="摇杆双电机遥控器的电脑端平面模拟器（只读，零依赖）")
    p.add_argument("--port", help="ESP32 的串口，如 /dev/cu.usbserial-XXXX")
    p.add_argument("--list", action="store_true", help="列出可用串口后退出")
    p.add_argument("--mock", action="store_true",
                   help="不接板子，本地造遥测（WASD 控制模拟摇杆）")
    return p


def main() -> int:
    args = build_parser().parse_args()

    if args.list:
        ports = list_ports()
        if ports:
            print("可用串口：")
            for p in ports:
                print("   ", p)
        else:
            print("没找到串口。确认板子插好、用的是标着 UART 的那个 Type-C 口。")
        return 0

    if not args.mock and not args.port:
        print("请给 --port（或用 --list 看看有哪些口），也可以用 --mock 离线看界面。",
              file=sys.stderr)
        return 2

    print("Motor-Sim：本程序只读，不会向板子发送任何指令。")
    if args.mock:
        print("纯模拟模式：不接板子、不碰串口，用 W A S D 当摇杆。")
    else:
        holders = port_holders(args.port)
        if holders:
            # macOS 允许多个进程同时打开同一个 /dev/cu.*，字节会被随机瓜分，
            # 两边都只能收到残缺的行。宁可先说清楚，也别把乱码当成硬件故障。
            print(f"！{args.port} 现在被这些程序占着：" + "、".join(holders))
            print("  请先关掉它们，否则收到的数据可能是残缺的。")
        print("提示：串口按 115200 打开，和固件的 Serial.begin 一致。")
    App(args).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

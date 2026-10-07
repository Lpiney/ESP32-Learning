#!/usr/bin/env python3
"""pad_bridge —— Xbox 手柄 → 电脑 → 串口 → ESP32（比赛现场那条链路）

数据流向：

    Xbox 手柄 ──USB/蓝牙──> 电脑（本脚本） ──串口 115200──> ESP32 ──> 电调
                              ↑                                 │
                              └──────── 遥测（L/R、状态）────────┘

本脚本每 20ms 发一行：

    CMD nx=+0.42 ny=-0.17 en=1
      nx/ny   左摇杆，-1~+1（推右 = +1，推上 = +1）
      en      1 = 已解锁（允许出力）；0 = 锁着 -> 板子强制中立

**安全键是开关式的**（默认锁住）：按一下解锁、再按一下锁上。
锁没锁在两处都看得见：板上 OLED 左上角（LOCK / UNLOCK / NOLINK）和
脚本的状态行。手柄一掉线会自动重新锁上。

**两条安全线都由板子实现**（脚本挂了也有效）：
    1. 断流超过 200ms -> 强制中立
    2. 锁着（en=0）   -> 强制中立

用法
----
    pip install pygame pyserial

    # ① 先认按键和轴（只读手柄，不碰串口）—— 找出安全键是哪个按钮
    python3 tools/pad_bridge.py --show-input

    # ② 正常跑。启动时会让你按一下想当安全键的那个键（它自己学编号），
    #    然后**默认是锁着的**：按一下那个键解锁，再按一下锁上。
    python3 tools/pad_bridge.py

    # ③ 第一次调试，不想管锁
    python3 tools/pad_bridge.py --no-deadman

    # ④ 不接板子，只测手柄读数
    python3 tools/pad_bridge.py --no-serial
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

# ---- 手柄读数约定（和固件的约定必须一致）----
# 目标：推右 = nx>0，推上 = ny>0。
#
# 原始轴 -> 上面这套约定的换算系数，**实机实测出来的**：
#
#   2026-10-06，Xbox Series X 手柄 / macOS / SDL 2.28 / pygame 2.6.1：
#     轴 0（左右）：推右 = 原始轴为正  -> PAD_X_SIGN = +1
#     轴 1（上下）：推上 = 原始轴为**负** -> PAD_Y_SIGN = -1
#
#   ⚠ 这一对符号来回错过好几次，教训是：**别照文档猜，也别照别的平台猜**。
#     SDL 的"手柄标准映射"文档说 Y 是"下 = 正"，但 pygame 的 get_axis() 读的是
#     **原始 HID 轴**，这只手柄的原始轴恰好是"上 = 负"。判断依据只有一条：
#     用 --show-input 看『会发出去』那一栏 —— 推上必须是 Y 正、推右必须是 X 正。
#
# 换手柄/换平台后：跑 --show-input 核对上面那一条，不对就改这两个系数
# （或者临时加 --invert-x / --invert-y）。**别去改固件里的符号**。
PAD_X_SIGN = +1.0
PAD_Y_SIGN = -1.0

DEFAULT_AXIS_X = 0
DEFAULT_AXIS_Y = 1
DEFAULT_DEADMAN_BUTTON = 5      # Xbox 手柄上通常是 RB；用 --show-input 确认

CMD_HZ = 50


# ===========================================================================
#  手柄
# ===========================================================================

class Pad:
    """读 Xbox 手柄。轴/按键编号都可以用 --show-input 现场确认。"""

    def __init__(self, axis_x, axis_y, deadman_button, invert_x, invert_y):
        self.axis_x = axis_x
        self.axis_y = axis_y
        self.deadman_button = deadman_button
        self.invert_x = invert_x
        self.invert_y = invert_y
        self.js = None
        self.name = ""

    def open(self) -> bool:
        import pygame
        pygame.init()
        pygame.joystick.init()
        self._pygame = pygame
        if pygame.joystick.get_count() == 0:
            return False
        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.name = self.js.get_name()
        return True

    def pump(self):
        """处理热插拔：比赛现场手柄掉了再插回来不用重启脚本。

        两个坑，都踩过：
        1. JOYDEVICEADDED 事件带的设备号，pygame 各版本属性名不一样
           （pygame 2.6 是 device_index，不是 device）—— 硬读一个名字会在
           插上手柄的瞬间抛 AttributeError，脚本当场就崩。
        2. 这是比赛现场跑的东西，**任何单个事件出问题都不许把整个桥接搞崩**，
           所以每个事件单独 try，出错就跳过。
        """
        pygame = self._pygame
        try:
            events = pygame.event.get()
        except Exception:
            return
        for ev in events:
            try:
                if ev.type == pygame.JOYDEVICEREMOVED:
                    self.js = None
                elif ev.type == pygame.JOYDEVICEADDED:
                    idx = getattr(ev, "device_index", None)
                    if idx is None:
                        idx = getattr(ev, "device", 0)
                    self.js = pygame.joystick.Joystick(int(idx))
                    self.js.init()
            except Exception:
                continue        # 单个事件处理失败，不影响其它事件和主循环

    @property
    def connected(self) -> bool:
        return self.js is not None

    def read(self):
        """返回 (nx, ny, deadman, raw_axes, raw_button_state)。"""
        if self.js is None:
            return 0.0, 0.0, False, [], False
        axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]

        def axis(i):
            return axes[i] if 0 <= i < len(axes) else 0.0

        nx = axis(self.axis_x) * PAD_X_SIGN * (-1.0 if self.invert_x else 1.0)
        ny = axis(self.axis_y) * PAD_Y_SIGN * (-1.0 if self.invert_y else 1.0)
        nx = max(-1.0, min(1.0, nx))
        ny = max(-1.0, min(1.0, ny))

        nb = self.js.get_numbuttons()
        pressed = [self.js.get_button(i) for i in range(nb)]
        idx = self.deadman_button
        # idx 可能是 None（还没学/还没指定），所以先判空 —— 之前这里
        # 直接 0 <= None 抛过 TypeError，把 --show-input 整个搞崩了。
        deadman = bool(pressed[idx]) if (idx is not None and 0 <= idx < nb) else False
        return nx, ny, deadman, axes, pressed

    def pressed_buttons(self) -> set:
        """当前按下的按钮号集合。"""
        if self.js is None:
            return set()
        return {i for i in range(self.js.get_numbuttons()) if self.js.get_button(i)}

    def snapshot(self):
        """给 --show-input 用：原样的轴和按键，一个都不加工。"""
        if self.js is None:
            return []
        axes = [self.js.get_axis(i) for i in range(self.js.get_numaxes())]
        buttons = [i for i in range(self.js.get_numbuttons()) if self.js.get_button(i)]
        return axes, buttons


# ===========================================================================
#  串口（板子那一头）
# ===========================================================================

# macOS 上永远存在、但和板子无关的串口（系统调试口、蓝牙入口）
_NOT_A_BOARD = ("debug-console", "Bluetooth-Incoming-Port", "Bluetooth-PDA-Sync")
# USB 转串口芯片（FTDI / CP210x / CH34x）—— **这个项目要的就是它**，
# 因为固件的 Serial 走 UART 口，只有这类桥接芯片才看得见。
_BRIDGE_LIKE = ("usbserial", "slab", "wch", "ftdi", "cp210")
# 别的 USB 串口：ESP32 自带的 USB-CDC（"usbmodem"）也在这类里，但本项目的
# 固件没开 USB CDC，所以它上面收不到遥测 —— 挑串口时要让桥接芯片优先。
_USB_LIKE = ("usbserial", "usbmodem", "slab", "wch", "usb")


def find_ports() -> list:
    """列出机器上的串口。优先用 pyserial 的枚举（跨平台）；没有就退回 /dev 扫描。"""
    try:
        from serial.tools import list_ports as slp
        return [p.device for p in slp.comports()]
    except Exception:
        try:
            from motor_sim import list_ports as dev_ports
            return dev_ports()
        except Exception:
            return []


def pick_port(names: list):
    """从一串串口名里挑出板子那个，挑不出来就返回 None 让调用方去问。

    纯函数（只吃字符串），所以自测里可以直接喂真实的串口列表。
    macOS 的列表里总混着蓝牙、耳朵里的蓝牙耳机、系统调试口这些东西 ——
    每次都要手敲 --port 太烦，能自动认出来就自动认。
    """
    cand = [n for n in names if not any(s in n for s in _NOT_A_BOARD)]
    # 桥接芯片（usbserial/SLAB/wch）优先：本项目固件的 Serial 走 UART 口，
    # 只有在它上面才看得到遥测；ESP32 自带的 usbmodem 收不到。
    bridge = [n for n in cand if any(k in n.lower() for k in _BRIDGE_LIKE)]
    if len(bridge) == 1:
        return bridge[0], cand
    usb_like = [n for n in cand if any(k in n.lower() for k in _USB_LIKE)]
    if len(usb_like) == 1:
        return usb_like[0], cand
    if len(cand) == 1:
        return cand[0], cand
    return None, cand


class Latch:
    """开关式安全键：**默认锁住**，按一下解锁，再按一下锁住。

    为什么不用"按住才出力"：那样两只手都得占用（一只手一直按着肩键），
    比赛现场反而容易出现"以为按住了其实松了"。开关式按一下就行，
    锁没锁在 OLED 和状态行上都看得见。

    安全兜底不变，而且更严：**手柄一掉线就自动重新锁上**。
    """

    def __init__(self, locked: bool = True):
        self.locked = locked
        self._was_down = False
        self.changed = None        # 本次 update() 里是否翻转了（给上层打印用）

    def update(self, key_down: bool) -> bool:
        """喂入"安全键此刻是否按下"，返回"是否允许出力"。

        只在**上升沿**（刚按下去那一刻）翻转，所以按住不放不会来回跳。
        """
        self.changed = None
        down = bool(key_down)
        if down and not self._was_down:
            self.locked = not self.locked
            self.changed = self.locked
        self._was_down = down
        return not self.locked

    def force_lock(self):
        """强制锁上（手柄掉线、脚本要退出时用）。"""
        if not self.locked:
            self.locked = True
            self.changed = True


def diagnose_no_telemetry(port: str) -> str:
    """串口开着但一直没遥测 —— 把最可能的原因直接讲出来。

    原来的行为是永远显示"等板子遥测…"，用户只能干等。这类"工具自己知道
    原因却不说"的坑，这个项目踩过好几次（见 docs/ERROR-REPORT.md）。
    """
    p = (port or "").lower()
    if "usbmodem" in p:
        return ("这个口是 ESP32 **自带的 USB 口**（/dev/cu.usbmodem*）。"
                "本固件的 Serial 走 **UART 口** —— 板子上标 UART 的那个 Type-C，"
                "设备名是 /dev/cu.usbserial-*。\n"
                "  换插另一个口，然后**重新跑本脚本**（端口是打开时绑定的）。")
    if any(k in p for k in ("usbserial", "slab", "wch", "ftdi", "cp210")):
        return ("口看着是对的（USB 转串口芯片）。那检查一下：板子通电了吗？"
                "烧的是 03 的固件吗（会一直打 JOY 行）？串口是不是被串口监视器"
                "或 motor_sim 占着？")
    return "没收到遥测：确认板子通电、跑的是 03 固件（会一直打 JOY 行）。"


def cmd_line(nx: float, ny: float, en: bool) -> str:
    """电脑发给板子的那一行。格式必须和固件里 parseCmdLine() 对得上 ——
    tools/test_pc_protocol.py 会拿这个函数生成的字符串去喂真实的固件解析代码。"""
    return f"CMD nx={nx:+.3f} ny={ny:+.3f} en={1 if en else 0}\n"


class Board:
    """和板子的串口链路：发指令 + 收遥测。"""

    def __init__(self, port):
        self.port = port
        self.ser = None
        self.buf = b""
        self.telem = None          # 最近一条解析成功的遥测
        self.last_rx = 0.0
        self.bad_lines = 0

    def open(self):
        import serial
        # 打开串口可能会让板子复位一次（FTDI 的 DTR/RTS 接了自动复位电路）——
        # 这是正常的：板子会重新走一遍开机流程，几秒后就开始发遥测。
        self.ser = serial.Serial(self.port, 115200, timeout=0)
        try:
            self.ser.dtr = False
            self.ser.rts = False
        except Exception:
            pass

    def send(self, nx, ny, en):
        self.ser.write(cmd_line(nx, ny, en).encode("ascii"))

    def poll(self):
        """把串口里能读到的都读掉，更新 self.telem。"""
        from motor_sim import Telemetry, parse_joy
        chunk = self.ser.read(4096)
        if not chunk:
            return
        self.buf += chunk
        while b"\n" in self.buf:
            raw, self.buf = self.buf.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            t = Telemetry()
            if parse_joy(line, t):
                self.telem = t
                self.last_rx = time.monotonic()
            else:
                self.bad_lines += 1

    def close(self):
        if self.ser is not None:
            try:
                self.send(0.0, 0.0, False)      # 走之前先让板子回中立
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass


# ===========================================================================
#  两种运行方式
# ===========================================================================

def show_input(pad: Pad) -> int:
    """只读手柄：同时打印**原始轴**和**换算后会发出去的值**。

    原来只打原始轴，于是"原始轴是什么方向"和"发出去的 X/Y 是什么方向"要人在
    脑子里换算 —— 这个项目已经因为"换算时猜符号"来回折腾好几次了。现在两栏
    并排显示，一眼就能看出符号对不对。
    """
    print("手柄诊断模式：只读手柄、不碰串口，不发任何东西。")
    print("请依次做三个动作，每个停 2 秒：① 左摇杆推到最上 ② 推到最下 ③ 推到最右")
    print("")
    print("看右边『会发出去』那一栏：")
    print("  推上 -> Y 必须是**正**数；推下 -> Y 是负数；推右 -> X 是**正**数。")
    print("  哪一条不对，说明对应的符号系数要翻（改 pad_bridge.py 顶上的")
    print("  PAD_X_SIGN / PAD_Y_SIGN，或者临时加 --invert-x / --invert-y）。")
    print("另外按一按各个按钮，右边会列出按下的按钮号。\n")
    while True:
        pad.pump()
        if not pad.connected:
            print("\r等手柄接入…（USB 插上，或蓝牙配对）" + " " * 30, end="", flush=True)
            time.sleep(0.3)
            continue
        nx, ny, _, axes, _ = pad.read()
        buttons = sorted(pad.pressed_buttons())
        moving = " ".join(f"轴{i}={v:+.2f}" for i, v in enumerate(axes) if abs(v) > 0.25)
        line = (f"原始 {moving:44s} | 会发出去 X{nx:+.2f} Y{ny:+.2f} | "
                f"按钮 {buttons if buttons else ''}")
        print(f"\r{line[:170]:170s}", end="", flush=True)
        time.sleep(0.05)


def run_bridge(pad: Pad, board, use_deadman: bool, period: float) -> int:
    print("桥接模式：手柄 → 串口 → 板子。Ctrl-C 退出。")
    latch = Latch(locked=True)
    if use_deadman:
        print(f"安全键 = 按钮 {pad.deadman_button}（开关式）："
              f"现在**锁着**，按一下解锁，再按一下锁上。")
        print("手柄一掉线会自动重新锁上。")
    if board is None:
        print("！--no-serial：不接板子，只看手柄读数。")
    time.sleep(0.4)

    sent = 0
    t_last = 0.0
    t_draw = 0.0
    t_start = time.monotonic()
    warned = False
    try:
        while True:
            now = time.monotonic()
            pad.pump()

            if pad.connected:
                nx, ny, key_down, _, _ = pad.read()
            else:
                nx, ny, key_down = 0.0, 0.0, False      # 手柄掉了 -> 立刻回中立

            if not use_deadman:
                en = True                              # --no-deadman：不锁
            elif not pad.connected:
                latch.force_lock()                     # 掉线自动重新锁上（安全优先）
                en = False
            else:
                en = latch.update(key_down)
                if latch.changed is not None:
                    print("\n" + ("已**锁定**（再按一下解锁）" if latch.changed
                                  else "已**解锁**（推摇杆就出力；再按一下锁上）"))

            # 固定的发送节拍：不跟着手柄的刷新率走，板子那边看到的是稳定 50Hz
            if now - t_last >= period:
                t_last = now
                if board is not None:
                    try:
                        board.send(nx, ny, en)
                        sent += 1
                    except Exception as exc:
                        print(f"\n串口写失败：{exc}")
                        return 2

            if board is not None:
                board.poll()
                # 开了 4 秒还一条遥测都没有 -> 把原因讲清楚，别让人干等
                if (board.telem is None and not warned
                        and now - t_start > 4.0):
                    warned = True
                    print("\n\n" + diagnose_no_telemetry(board.port))
                    print("（脚本还在跑，插对口之后需要重跑。）")

            # 显示跟发送是两回事：发 50Hz，显示 10Hz 就够看了（刷太快反而看不清）
            if now - t_draw >= 0.1:
                t_draw = now
                draw_status(pad, board, nx, ny, en, sent, use_deadman,
                            latched=latch.locked)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n\n退出（板子那边会因为断流自动回中立）。")
        return 0


def draw_status(pad, board, nx, ny, en, sent, use_deadman, latched=True):
    pad_txt = "手柄 ✓" if pad.connected else "手柄 ✗（回中立）"
    if not use_deadman:
        lock_txt = "未启用（一直可用）"
    elif not pad.connected:
        lock_txt = "已锁定（手柄掉了）"
    else:
        lock_txt = "已解锁：可出力" if en else "已锁定：推杆也不动"

    if board is None:
        board_txt = "未接板子"
    elif board.telem is None:
        board_txt = "等板子遥测…（刚打开串口时板子会重启一次）"
    else:
        t = board.telem
        age = (time.monotonic() - board.last_rx) * 1000
        board_txt = (f"板子 L={t.l_us:4d} R={t.r_us:4d} "
                     f"{'LINK✓' if t.link else 'LINK✗断流'} "
                     f"{'已解锁' if t.en else '已锁定'} "
                     f"（{age:.0f}ms 前）")

    line = (f"{pad_txt} | 摇杆 X{nx:+.2f} Y{ny:+.2f} | {lock_txt:22s} | "
            f"已发{sent:6d} | {board_txt}")
    print(f"\r{line[:170]:170s}", end="", flush=True)


# ===========================================================================
#  命令行
# ===========================================================================

# ===========================================================================
#  自测（不需要手柄、不需要 pygame、不碰串口）
# ===========================================================================

class _FakeEvent:
    def __init__(self, type_, **kw):
        self.type = type_
        self.__dict__.update(kw)


class _FakeJoystick:
    """假手柄：轴 0=左摇杆X，轴 1=左摇杆Y，按钮 5 = 安全键。"""

    def __init__(self, idx=0, axes=(0.0, 0.0), buttons=()):
        self.idx = idx
        self.axes = list(axes) + [0.0] * 6
        self.buttons = [1 if i in buttons else 0 for i in range(11)]

    def init(self):
        pass

    def get_name(self):
        return f"FakePad{self.idx}"

    def get_numaxes(self):
        return len(self.axes)

    def get_axis(self, i):
        return self.axes[i]

    def get_numbuttons(self):
        return len(self.buttons)

    def get_button(self, i):
        return self.buttons[i]


class _FakePygame:
    JOYDEVICEADDED = 1536
    JOYDEVICEREMOVED = 1537

    def __init__(self, events=(), joystick=None):
        self._events = list(events)
        self._js = joystick
        self.joystick = _FakePygame._JoystickModule(joystick)

    class _JoystickModule:
        def __init__(self, js):
            self._js = js

        def Joystick(self, idx):          # noqa: N802 (pygame 的接口名)
            if self._js is None:
                raise RuntimeError("没有手柄")
            return self._js

    @property
    def event(self):
        return self

    def get(self):
        ev, self._events = self._events, []
        return ev


def selftest() -> int:
    ok = True

    def check(name, cond, detail=""):
        nonlocal ok
        print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"   {detail}" if not cond and detail else ""))
        ok = ok and cond

    def make_pad(axes=(0.0, 0.0), buttons=(), **kw):
        p = Pad(kw.get("axis_x", 0), kw.get("axis_y", 1),
                kw.get("deadman", DEFAULT_DEADMAN_BUTTON),
                kw.get("inv_x", False), kw.get("inv_y", False))
        p._pygame = _FakePygame()          # 自测里不初始化真的 pygame
        p.js = _FakeJoystick(axes=axes, buttons=buttons)
        p.name = "FakePad"
        return p

    print("pad_bridge 自测（不需要手柄 / pygame / 串口）")

    # ---- 1. 热插拔事件：属性名差异不能再让脚本崩 ----
    print("\n[1] 热插拔事件（曾经在这里崩过）")
    probe = _FakeEvent(_FakePygame.JOYDEVICEADDED, device_index=0)
    check("（这条用例有效的前提）这种事件确实没有 .device 属性",
          not hasattr(probe, "device"))
    for attr in ("device_index", "device"):
        p = make_pad()
        p.js = None
        fake = _FakePygame(events=[_FakeEvent(_FakePygame.JOYDEVICEADDED, **{attr: 0})],
                           joystick=_FakeJoystick(0))
        p._pygame = fake
        try:
            p.pump()
            check(f"事件只带 {attr} 时能接上手柄", p.js is not None)
        except Exception as exc:
            check(f"事件只带 {attr} 时能接上手柄", False, f"{type(exc).__name__}: {exc}")

    p = make_pad()
    p.js = None
    p._pygame = _FakePygame(events=[_FakeEvent(_FakePygame.JOYDEVICEADDED, device_index=None)],
                            joystick=_FakeJoystick(0))
    try:
        p.pump()
        check("事件两个属性都没有时也不崩", True)
    except Exception as exc:
        check("事件两个属性都没有时也不崩", False, f"{type(exc).__name__}: {exc}")

    p = make_pad()
    p._pygame = _FakePygame(events=[_FakeEvent(_FakePygame.JOYDEVICEREMOVED, device_index=0)],
                            joystick=_FakeJoystick(0))
    p.pump()
    check("拔掉手柄 -> 标记为未连接（上层会立刻回中立）", p.js is None and not p.connected)

    p = make_pad()
    p.js = _FakeJoystick(0)
    p._pygame = _FakePygame(events=[_FakeEvent(999, whatever=1)])   # 不认识的类型
    try:
        p.pump()
        check("不认识的事件类型直接忽略", True)
    except Exception as exc:
        check("不认识的事件类型直接忽略", False, f"{type(exc).__name__}: {exc}")

    # ---- 2. 摇杆读数约定：推右 = +nx，推上 = +ny ----
    print("\n[2] 读数约定 —— 要求：推上 ny 为正、推右 nx 为正")
    p = make_pad(axes=(0.83, 0.0))
    nx, ny, en, _, _ = p.read()
    check("轴0=+0.83（推右）-> nx=+0.83", abs(nx - 0.83) < 1e-6, f"nx={nx}")

    p = make_pad(axes=(0.0, -0.66))     # 实测：这只手柄推上 = 轴1 **负**数
    ny = p.read()[1]
    check("轴1=-0.66（推上）-> ny=+0.66（Y 要为正）", abs(ny - 0.66) < 1e-6, f"ny={ny}")

    p = make_pad(axes=(0.0, 0.66))      # 推下 -> 原始轴为正
    ny = p.read()[1]
    check("轴1=+0.66（推下）-> ny=-0.66", abs(ny + 0.66) < 1e-6, f"ny={ny}")

    # 这一条**不能写死数值**：基准符号（PAD_X_SIGN/PAD_Y_SIGN）是会按实测改的，
    # 写死就会像上次那样，改完常量测试就红。只管"invert 是不是把结果取了反"。
    base_nx, base_ny, _, _, _ = make_pad(axes=(0.5, 0.5)).read()
    inv_nx, inv_ny, _, _, _ = make_pad(axes=(0.5, 0.5), inv_x=True, inv_y=True).read()
    check("--invert-x / --invert-y 就是把结果取反（和基准符号无关）",
          abs(inv_nx + base_nx) < 1e-6 and abs(inv_ny + base_ny) < 1e-6
          and abs(base_nx) > 0.1,
          f"基准 nx={base_nx} ny={base_ny} / invert 后 nx={inv_nx} ny={inv_ny}")

    p = make_pad(axes=(1.5, -1.5))
    nx, ny, _, _, _ = p.read()
    check("超出 ±1 的读数被夹住（有些手柄会过冲）", nx <= 1.0 and ny <= 1.0,
          f"nx={nx} ny={ny}")

    # ---- 3. 安全键 ----
    print("\n[3] 安全键")
    p = make_pad(axes=(0.0, 0.0))
    check("没按 -> en=False", p.read()[2] is False)
    p = make_pad(axes=(0.0, 0.0), buttons=(DEFAULT_DEADMAN_BUTTON,))
    check(f"按住按钮 {DEFAULT_DEADMAN_BUTTON} -> en=True", p.read()[2] is True)
    p = make_pad(axes=(0.0, 0.0), buttons=(3,), deadman=99)
    check("安全键号超范围时不崩，按未按处理", p.read()[2] is False)

    p = make_pad(axes=(0.5, -0.5))
    p.deadman_button = None          # --show-input 走这条路：还没学安全键
    try:
        nx, ny, en, _, _ = p.read()
        check("安全键还没指定（None）时也不崩 —— --show-input 靠它",
              abs(nx - 0.5) < 1e-6 and en is False, f"nx={nx} en={en}")
    except Exception as exc:
        check("安全键还没指定（None）时也不崩 —— --show-input 靠它", False,
              f"{type(exc).__name__}: {exc}")

    # ---- 3b. 开关式安全键（默认锁住 / 按一下翻转 / 掉线自动锁上）----
    print("\n[3b] 开关式安全键")
    la = Latch()
    check("一开始是锁着的 -> 不出力", la.update(False) is False)

    check("按一下 -> 解锁，出力", la.update(True) is True)
    check("按住不放（连续 True）不会来回跳", la.update(True) is True and not la.changed)
    check("松开 -> 仍然解锁（开关式，不是按住式）", la.update(False) is True)

    check("再按一下 -> 重新锁上", la.update(True) is False)
    check("松开后仍锁着", la.update(False) is False)

    la2 = Latch()
    la2.update(True)                      # 解锁
    la2.force_lock()
    check("掉线自动重新锁上", la2.locked and la2.update(False) is False)
    check("没解锁时 force_lock 不产生假事件", Latch().update(False) is False)

    # ---- 4. 发给板子的字符串 ----
    print("\n[4] 指令格式")
    check("松手/松开安全键", cmd_line(0.0, 0.0, False) == "CMD nx=+0.000 ny=+0.000 en=0\n",
          repr(cmd_line(0.0, 0.0, False)))
    check("推右前 + 按住", cmd_line(0.42, -0.17, True) == "CMD nx=+0.420 ny=-0.170 en=1\n",
          repr(cmd_line(0.42, -0.17, True)))

    # ---- 5. 串口自动挑选（macOS 的串口列表里全是杂音）----
    print("\n[5] 串口自动挑选")
    mac_list = ["/dev/cu.debug-console", "/dev/cu.iFLYBUDSPro2",
                "/dev/cu.Bluetooth-Incoming-Port", "/dev/cu.usbserial-A5069RR4"]
    chosen, cands = pick_port(mac_list)
    check("真实 macOS 列表（4 个）里挑出板子那个",
          chosen == "/dev/cu.usbserial-A5069RR4", f"选中 {chosen}")

    chosen, cands = pick_port(["/dev/cu.debug-console", "/dev/cu.Bluetooth-Incoming-Port"])
    check("只剩系统/蓝牙口时不乱选（返回 None 让用户指定）",
          chosen is None and cands == [], f"selected={chosen} cands={cands}")

    chosen, _ = pick_port(["/dev/cu.usbserial-A", "/dev/cu.usbserial-B"])
    check("两个**同类**口都在、分不清哪个是板子时不猜（返回 None）",
          chosen is None, f"selected={chosen}")

    chosen, _ = pick_port(["/dev/ttyS0"])
    check("只有一个候选（哪怕名字不像）就用它", chosen == "/dev/ttyS0", f"selected={chosen}")

    # 插错口的真实场景：板子同时有 usbmodem（自带 USB）和 usbserial（UART 桥）
    two_ports = ["/dev/cu.debug-console", "/dev/cu.iFLYBUDSPro2",
                 "/dev/cu.usbmodem101", "/dev/cu.usbserial-A5069RR4"]
    chosen, _ = pick_port(two_ports)
    check("两个口都在时，选 USB 转串口的那个（不是自带 USB 口）",
          chosen == "/dev/cu.usbserial-A5069RR4", f"选中 {chosen}")

    check("插了自带 USB 口时，诊断信息会直接说是哪个口的问题",
          "自带的 USB 口" in diagnose_no_telemetry("/dev/cu.usbmodem101")
          and "UART" in diagnose_no_telemetry("/dev/cu.usbmodem101"))
    check("插对口却没数据时，提醒查板子/固件/串口被占",
          "占着" in diagnose_no_telemetry("/dev/cu.usbserial-A5069RR4"))

    # ---- 6. 安全键自动学习（不靠"按钮 5 一定是 RB"这个猜测）----
    print("\n[6] 安全键自动学习")

    def scripted(seq):
        it = iter(seq)
        return lambda: next(it, seq[-1])

    idx, why = learn_deadman(scripted([set(), set(), {5}, {5}]),
                             timeout=0.5, fallback=None, sleep=lambda s: None)
    check("按了一下按钮 5 -> 学到 5", idx == 5, f"got={idx} ({why})")

    idx, why = learn_deadman(scripted([{2}, {2}, {2, 9}, {2, 9}]),
                             timeout=0.5, fallback=None, sleep=lambda s: None)
    check("启动时已经按着 2、后来又按 9 -> 学到 9（不会把 2 学进去）",
          idx == 9, f"got={idx} ({why})")

    idx, why = learn_deadman(scripted([set()]), timeout=0.05, fallback=None,
                             sleep=lambda s: None)
    check("一直不按 -> 返回 None（上层退回默认编号）", idx is None, f"got={idx}")

    idx, why = learn_deadman(scripted([set(), {7, 3}]), timeout=0.5, fallback=None,
                             sleep=lambda s: None)
    check("同时按了两个 -> 取小的那个（有确定性，不乱）", idx == 3, f"got={idx}")

    print(f"\n{'自测通过' if ok else '自测有失败项'}")
    return 0 if ok else 1


def learn_deadman(poll, timeout=12.0, fallback=None, sleep=time.sleep):
    """让用户按一下"想当安全键"的按钮，把这个编号学出来。

    为什么不写死编号：Xbox 手柄在 SDL 下的按钮编号大体是 RB=5，但手柄固件、
    有线/蓝牙、不同平台的映射都可能不一样 —— 而这个项目已经在"照文档猜一个
    没实测过的编号"上翻过车了（见 docs/ERROR-REPORT.md）。按一下最可靠。

    poll: 无参函数，返回"此刻按下的按钮号集合"（自测里给脚本化的序列）。
    返回 (按钮号 或 None, 说明)。
    """
    t0 = time.monotonic()
    baseline = set(poll())          # 启动时已经按住的键不算，免得学错
    while time.monotonic() - t0 < timeout:
        new = set(poll()) - baseline
        if new:
            idx = min(new)
            return idx, f"学到：安全键 = 按钮 {idx}"
        sleep(0.02)
    return fallback, "没检测到按键，改用默认编号"


def missing_deps():
    """说清楚"装到哪个 Python 里" —— 机器上常有 conda / Homebrew / 系统三个
    python3，pip 装进一个、脚本用另一个跑，是最常见的一种"我明明装了啊"。"""
    print("这个 Python 里没有 pygame / pyserial。")
    print(f"  当前解释器：{sys.executable}")
    print("  装到**它**里面（用同一个 python 去装，注意引号）：")
    print(f'    "{sys.executable}" -m pip install pygame pyserial')
    print("  或者直接换成已经装好的那个跑（一般是你敲 pip 时用的那个）：")
    print("    python3 tools/pad_bridge.py")


def main() -> int:
    p = argparse.ArgumentParser(
        description="Xbox 手柄 -> 电脑 -> 串口 -> ESP32（每 20ms 发一行 CMD）")
    p.add_argument("--port", help="串口设备（不填就自动找）")
    p.add_argument("--show-input", action="store_true",
                   help="只读手柄，打印轴/按键编号（用来找安全键是几号）")
    p.add_argument("--no-deadman", action="store_true",
                   help="不需要按住安全键（第一次调试用，小心）")
    p.add_argument("--no-serial", action="store_true", help="不接板子，只测手柄")
    p.add_argument("--deadman", type=int, default=None,
                   help="安全键的按钮号。不给就启动时让你按一下（推荐；学不到才退回 "
                        f"{DEFAULT_DEADMAN_BUTTON}）")
    p.add_argument("--axis-x", type=int, default=DEFAULT_AXIS_X, help="左摇杆 X 的轴号")
    p.add_argument("--axis-y", type=int, default=DEFAULT_AXIS_Y, help="左摇杆 Y 的轴号")
    p.add_argument("--invert-x", action="store_true", help="X 轴取反（推右变成负数时用）")
    p.add_argument("--invert-y", action="store_true", help="Y 轴取反（推上变成负数时用）")
    p.add_argument("--selftest", action="store_true",
                   help="不接手柄/不碰串口，只测本脚本的逻辑（手感约定、热插拔、安全键）")
    args = p.parse_args()

    if args.selftest:
        return selftest()

    try:
        import pygame  # noqa: F401
    except ImportError:
        missing_deps()
        return 2

    pad = Pad(args.axis_x, args.axis_y, args.deadman, args.invert_x, args.invert_y)
    if not pad.open():
        print("没找到手柄。USB 插上、或先蓝牙配对；然后用 --show-input 确认。")
        return 2
    print(f"手柄：{pad.name}")

    if args.show_input:
        return show_input(pad)

    # 安全键：不靠"按钮 5 一定是 RB"这个猜测，让用户按一下学出来
    if args.no_deadman:
        print("！--no-deadman：不需要按任何键就会出力。")
    elif args.deadman is None:
        print("请**按一下你想当安全键的那个按钮**（建议右肩键 RB）。")
        print(f"（{12} 秒内没按就用默认编号 {DEFAULT_DEADMAN_BUTTON}）")
        idx, why = learn_deadman(lambda: (pad.pump(), pad.pressed_buttons())[1])
        pad.deadman_button = DEFAULT_DEADMAN_BUTTON if idx is None else idx
        print(why + f"（当前安全键 = 按钮 {pad.deadman_button}）")
    else:
        pad.deadman_button = args.deadman
        print(f"安全键 = 按钮 {pad.deadman_button}（你指定的）")

    board = None
    if not args.no_serial:
        try:
            import serial  # noqa: F401
        except ImportError:
            missing_deps()
            return 2
        port = args.port
        if not port:
            chosen, cands = pick_port(find_ports())
            if chosen:
                port = chosen
                print(f"串口：自动选中 {port}"
                      + (f"（从 {len(cands)} 个候选里挑的）" if len(cands) > 1 else ""))
            elif not cands:
                print("没找到串口。板子插上了吗？先用 --no-serial 只测手柄。")
                return 2
            else:
                print("找到多个像板子的串口，用 --port 指定一个：")
                for x in cands:
                    print("   ", x)
                return 2
        board = Board(port)
        try:
            board.open()
        except Exception as exc:
            print(f"打不开串口 {port}：{exc}")
            print("（被别的程序占着？先关掉串口监视器 / motor_sim）")
            return 2
        print(f"串口：{port} @ 115200")

    try:
        return run_bridge(pad, board, not args.no_deadman, 1.0 / CMD_HZ)
    finally:
        if board is not None:
            board.close()


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""电脑端 ↔ 固件 的**指令协议**一致性测试（不需要硬件）。

为什么值得单独测
----------------
这条链路是：pad_bridge.py 拼字符串 -> 串口 -> 固件 parseCmdLine() 解析。
两边的格式一旦对不上，症状是"电机完全不动"，而且从两边看都像"我这边没问题"。
这个项目前面已经栽过一次（遥测解析用 split(" X=") 把 JX 切错了），所以：

  不靠眼睛比对，**把固件里那段解析代码本身抠出来，用宿主编译器编译，
  再喂进 pad_bridge.py 真实生成的那些字符串**。

跑法：
    python3 tools/test_pc_protocol.py
"""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
MAIN_CPP = HERE.parent / "src" / "main.cpp"

sys.path.insert(0, str(HERE))
from pad_bridge import cmd_line  # noqa: E402

PASS = FAIL = 0


def check(name, cond, detail=""):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  ok   {name}")
    else:
        FAIL += 1
        print(f"  FAIL {name}  {detail}")


# ---------------------------------------------------------------------------
#  1. 从 src/main.cpp 里抠出真实的解析代码
# ---------------------------------------------------------------------------

def extract(text: str, start_marker: str, end_marker: str) -> str:
    i = text.index(start_marker)
    j = text.index(end_marker, i)
    return text[i:j + len(end_marker)]


def build_harness() -> str:
    src = MAIN_CPP.read_text()

    struct_src = extract(src, "struct PcInput {", "};")
    parser_src = extract(src, "static bool parseCmdLine(const char *s) {", "\n}")

    return f"""// 由 tools/test_pc_protocol.py 生成：把固件里真实的解析代码拿来在电脑上跑
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

static uint32_t g_now = 0;
static uint32_t millis() {{ return g_now; }}

{struct_src}

static PcInput pc;
static uint32_t pcCmds = 0, pcBadLines = 0;

{parser_src}

int main() {{
  std::string line;
  while (std::getline(std::cin, line)) {{
    if (!line.empty() && line.back() == '\\r') line.pop_back();
    bool ok = (g_now = 1000, parseCmdLine(line.c_str()));   // 模拟 millis() 有值
    std::printf("%d %.4f %.4f %d\\n", ok ? 1 : 0, pc.nx, pc.ny, pc.enable ? 1 : 0);
    std::fflush(stdout);
  }}
  return 0;
}}
"""


def compile_harness(work: pathlib.Path) -> pathlib.Path:
    src = work / "harness.cpp"
    src.write_text(build_harness())
    exe = work / "harness"
    compiler = "c++"
    r = subprocess.run([compiler, "-std=c++11", "-O0", "-o", str(exe), str(src)],
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stderr[:2000])
        raise SystemExit("抠出来的固件代码编译不过 —— 说明 main.cpp 里的函数结构变了，"
                         "请同步更新这个测试的抠取标记")
    return exe


# ---------------------------------------------------------------------------
#  2. 用例
# ---------------------------------------------------------------------------

def run_cases(exe: pathlib.Path, lines: list) -> list:
    out = subprocess.run([str(exe)], input="\n".join(lines) + "\n",
                         capture_output=True, text=True)
    res = []
    for ln in out.stdout.strip().splitlines():
        ok, nx, ny, en = ln.split()
        res.append((ok == "1", float(nx), float(ny), en == "1"))
    return res


def main() -> int:
    print("指令协议一致性测试（电脑端 → 固件解析器）\n")
    work = pathlib.Path(tempfile.mkdtemp(prefix="pcproto_"))
    exe = compile_harness(work)
    print(f"[1] 从 main.cpp 抠出 parseCmdLine() 编译到宿主机 —— {work}")

    # ---- pad_bridge 真实生成的三种典型指令 ----
    print("\n[2] pad_bridge.cmd_line() 生成的字符串，交给固件解析器")
    bridge_cases = [
        (0.0, 0.0, False, "松手 + 安全键松开"),
        (+0.420, -0.170, True, "推右前 + 安全键按住"),
        (-1.000, +1.000, True, "左下角推到底"),
        (-0.001, +0.999, True, "很小的负数（容易在字符串格式上出问题）"),
    ]
    lines = [cmd_line(nx, ny, en).rstrip("\n") for nx, ny, en, _ in bridge_cases]
    got = run_cases(exe, lines)
    for (nx, ny, en, desc), (ok, gx, gy, gen) in zip(bridge_cases, got):
        check(f"{desc}: 解析成功且数值一致", ok and abs(gx - nx) < 5e-4
              and abs(gy - ny) < 5e-4 and gen == en,
              f"发出 nx={nx} ny={ny} en={int(en)} -> 解析出 ok={ok} "
              f"nx={gx} ny={gy} en={int(gen)}")

    # ---- 容错：坏行不能把电机带飞 ----
    print("\n[3] 坏行必须被拒绝（宁可不动，也不要按半个指令出力）")
    bad = [
        ("CMD nx=+0.5 ny=+0.5", "缺 en"),
        ("CMD ny=+0.5 en=1", "缺 nx"),
        ("CMD nx=abc ny=+0.5 en=1", "数值不是数字（strtof 会给出 0，但字段在）"),
        ("JOY JX=1828 JY=1875 X=+0.000 Y=-0.001", "这是遥测行，不是指令"),
        ("CMD", "空的 CMD"),
        ("", "空行"),
    ]
    got = run_cases(exe, [b for b, _ in bad])
    for (line, desc), (ok, _, _, _) in zip(bad, got):
        # "数值不是数字"那条：字段齐全，板子会接受（值变 0 = 中立）—— 这也可以接受，
        # 因为结果是"不动"，比拒绝更安全。所以只有它允许 ok=True。
        allow_ok = "不是数字" in desc
        check(f"{desc} -> {'接受' if allow_ok else '拒绝'}", ok == allow_ok,
              f"实际 ok={ok}")

    # ---- 容错：多出来的字段不能把解析弄坏（以后加字段的前提）----
    print("\n[4] 追加未知字段不影响解析（向前兼容）")
    extra = ["CMD nx=+0.25 ny=-0.25 en=1 extra=9 foo=bar",
             "CMD seq=42 nx=+0.25 ny=-0.25 en=1"]
    got = run_cases(exe, extra)
    for line, (ok, gx, gy, gen) in zip(extra, got):
        check(f"仍能解析: {line[:40]}…", ok and abs(gx - 0.25) < 5e-4
              and abs(gy + 0.25) < 5e-4 and gen, f"ok={ok} nx={gx} ny={gy} en={gen}")

    # ---- 顺序无关 ----
    print("\n[5] 字段顺序无所谓")
    got = run_cases(exe, ["CMD en=1 ny=+0.5 nx=-0.5"])
    ok, gx, gy, gen = got[0]
    check("en/ny/nx 换顺序照样解析", ok and gx == -0.5 and gy == 0.5 and gen,
          f"ok={ok} nx={gx} ny={gy} en={gen}")

    print(f"\n{'=' * 46}\n通过 {PASS} 项，失败 {FAIL} 项")
    return 1 if FAIL else 0


if __name__ == "__main__":
    raise SystemExit(main())

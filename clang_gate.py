#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tools/clang_gate.py — 纯算法层 clang 语法门禁

用途: 在不依赖 Arm 交叉编译器 / CubeMX HAL 头的前提下, 对拓扑闭包里的
"纯算法层" .c 文件做 clang -fsyntax-only, 作为 CI 的编译健康检查。

原理:
  scaffold.py gen 产出的 build/gen/<topo>/CMakeLists.txt 已列出闭包全部
  .c 源 + include 路径。本脚本复用它们, 提取非硬件后端的源做语法编译。

硬件后端排除 (这些文件需要真实 HAL/driverlib 头, 由平台工程 + CMake
工具链提供, 不在通用 CI 编译范围):
  - BSP/bsp_*_stm32.c    (依赖 ST 系列宏 + HAL 头)
  - BSP/bsp_*_c2000.c    (依赖 C2000 driverlib)
  - BSP/bsp_hrtim.c      (HRTIM 硬件)
  - Devices/... 里直接 include 平台 HAL 的子类 (由 gen_app/P1 平台裁剪解决,
    当前显式排除已知待修项, 见 EXCLUDE_SUFFIX)

已知待修 (HardC 自身 bug, 见 docs/debug/ROADMAP.md / 本次 P0 排查):
  comp_pi_reg4.h 已随 PID 重构删除, 但 mod_supercap/mod_current_share
  头文件仍 include 旧名 → 这两个模块暂不进语法门禁 (列入 BUGLIST 待 P1 修复)。

CLI:
  python tools/clang_gate.py            # 测全部 ready 拓扑
  python tools/clang_gate.py buck       # 只测 buck
  --clang <path>                        # 指定 clang 可执行 (默认 PATH 找 clang)

仅供 local/CI 使用; 忽略文件名/路径含测试 stub 的项。
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

# 硬件后端排除前缀 (文件在闭包路径中的相对路径)
BSP_EXCLUDE = (
  "BSP/bsp_delay.c",
  "BSP/bsp_hrtim.c",
  "BSP/bsp_adc_stm32.c",
  "BSP/bsp_gpio_stm32.c",
  "BSP/bsp_watchdog_stm32.c",
  "BSP/bsp_watchdog_c2000.c",
  "BSP/bsp_c2000_epwm.c",
  "BSP/bsp_c2000_adc.c",
  "BSP/bsp_irq_stm32.c",
  "BSP/bsp_irq_c2000.c",
  "BSP/bsp_uart_stm32.c",
  "BSP/bsp_uart_c2000.c",
  "BSP/bsp_i2c_stm32.c",
  "BSP/bsp_spi_stm32.c",
  "BSP/bsp_spi_c2000.c",
  "BSP/bsp_can_stm32.c",
  "BSP/bsp_can_c2000.c",
  "BSP/bsp_gpio_c2000.c",
)

# 已知待修模块 — 当前为空 (P0 的 comp_pi_reg4.h 残留已修复: include 改 pid_reg4.h/comp_pid.h)
KNOWN_BROKEN = ()

# 生成骨架路径
GEN_DIR = Path("build/gen")
# 拓扑 (仅 status:ready, 且已在 Config/projects/ 有工程)
READY_TOPOS = ("buck", "acdc_sixswitch", "supercap_3ph")


def find_clang(explicit: str | None) -> str:
  if explicit:
    p = Path(explicit)
    if not p.is_file():
      sys.exit(f"[FAIL] 指定 clang 不存在: {explicit}")
    return str(p)
  clang = shutil.which("clang")
  if not clang:
    sys.exit("[FAIL] 未找到 clang, 请加入 PATH 或 --clang <path>")
  return clang


def parse_cmakelist(cmake_path: Path) -> tuple[list[str], list[str]]:
  """从骨架 CMakeLists.txt 提取 (.c 源, include 路径)."""
  if not cmake_path.is_file():
    sys.exit(f"[FAIL] 骨架不存在: {cmake_path} (先运行 python YmaC/scaffold.py gen)")
  text = cmake_path.read_text(encoding="utf-8", errors="ignore")
  sources = re.findall(r"^\s{2}([A-Za-z0-9_./-]+\.c)\s*$", text, re.M)
  # include 路径: PUBLIC 段里非 BSP 根、非 .c 的行
  in_inc = False
  includes = []
  for line in text.splitlines():
    s = line.strip()
    if s.startswith("target_include_directories"):
      in_inc = True
      continue
    if in_inc:
      if s == "" or s.startswith(")"):
        break
      if not s.endswith(".c") and not s.startswith("//"):
        includes.append(s)
  return sources, includes


def main() -> int:
  ap = argparse.ArgumentParser()
  ap.add_argument("topo", nargs="?", default=None, help="拓扑名, 默认测全部 ready")
  ap.add_argument("--clang", default=None, help="clang 可执行路径")
  args = ap.parse_args()

  clang = find_clang(args.clang)
  topos = [args.topo] if args.topo else READY_TOPOS

  root = Path(os.getcwd()).resolve()
  total_fail = 0
  for topo in topos:
    cmake_path = GEN_DIR / topo / "CMakeLists.txt"
    sources, includes = parse_cmakelist(cmake_path)
    inc_flags = ["-I" + inc for inc in includes] + ["-IApp"]
    fname = f"\n=== {topo} ({len(sources)} 源) ==="
    print(fname)
    skipped = 0
    fails = 0
    for src in sources:
      if src in BSP_EXCLUDE or src in KNOWN_BROKEN:
        skipped += 1
        print(f"  [SKIP] {src.split('/')[-1]}")
        continue
      full = root / src
      if not full.is_file():
        print(f"  [FAIL] 源不存在: {src}")
        fails += 1
        total_fail += 1
        continue
      r = subprocess.run(
        [clang, "-std=c99", "-fsyntax-only", *inc_flags, str(full)],
        capture_output=True, text=True,
        encoding="utf-8", errors="replace")
      if r.returncode != 0:
        err = r.stderr or r.stdout or "(无输出)"
        first_err = next((l for l in err.splitlines() if "error:" in l),
                         err.splitlines()[-1]).strip()
        print(f"  [FAIL] {src.split('/')[-1]} :: {first_err}")
        fails += 1
        total_fail += 1
      else:
        print(f"  [OK]   {src.split('/')[-1]}")
    print(f"  -> {len(sources)-skipped-fails} 通过 / {fails} 失败 / {skipped} 跳过(硬件后端/待修)")

  if total_fail:
    print(f"\n语法门禁失败: {total_fail} 个文件 (含源缺失). 见上方 [FAIL].")
    return 1
  print("\n语法门禁全部通过.")
  return 0


if __name__ == "__main__":
  sys.exit(main())

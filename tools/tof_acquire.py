#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hilab_PID ToF 1cm 梯度采集脚本（配合 8 通道滤波固件）

固件 VOFA FireWater 帧（每 50ms，20Hz）：
  raw_mm, filt_mm, pos_cm_x10, status, valid_cnt, alpha_milli, seg, hold

串口命令：
  A<milli>  设置 EMA 系数（20~1000，1000=直通）
  Z         以当前滤波值标定零点；Z<mm> 显式设置零点距离
  S<id>     设置测试段号（0~99，随 ch6 回传）
  H0/H1     关闭/使能以当前角度为目标的平台保持
  R         复位滤波器与保持环运行态

用法：
  python tools/tof_acquire.py --port COM13 --zero-cm 3 --t-min 3 --t-max 25 --samples 300
  python tools/tof_acquire.py --port COM13 --zero-cm 3 --t-min 3 --t-max 25 --hold
  python tools/tof_acquire.py --port COM13 --step-from 8 --step-to 18 --step-frames 600

测试约定：
  - 最近有效距离 3–5cm，0–2cm 不作为有效点；测试前必须 Z 标定；
  - 有效帧太少时，可先复位/重新上电再重试（临时处置）；
  - 每个梯度点采集 --samples 帧（默认 300 帧 = 15s @20Hz）；
  - --zero-cm 是标定零点时小球所在的刻度（相对测距模块的 cm 读数）；
  - CSV 的 nominal_cm = 当前刻度 - zero_cm，与固件 pos_cm_x10 同一坐标；
  - 阶跃段 seg=99，采集开始后 2s 打印提示并响铃，此时快速移动小球；
  - 分析：python tools/tof_eval.py --csv <csv> --zero-cm <zero> --t-min <tmin> --step-cm <step>
"""

import argparse
import csv
import json
import os
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial，请先执行：pip install pyserial")
    sys.exit(1)

FRAME_RE = re.compile(
    r"^\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,"
    r"\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*,\s*(-?\d+)\s*$"
)


def parse_args():
    p = argparse.ArgumentParser(description="ToF 1cm gradient acquisition")
    p.add_argument("--port", default="COM13", help="串口，例如 COM13")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--t-min", type=int, default=0, help="起始刻度 cm（相对测距模块）")
    p.add_argument("--t-max", type=int, default=25, help="结束刻度 cm")
    p.add_argument("--step-cm", type=int, default=1, help="梯度步长 cm")
    p.add_argument("--samples", type=int, default=300, help="每个梯度点采集帧数")
    p.add_argument("--zero-cm", type=int, default=0, help="标定零点所在刻度 cm")
    p.add_argument("--alpha", type=int, default=300, help="测试前设置的 EMA 千分比")
    p.add_argument("--hold", action="store_true", help="标定后发送 H1 使能平台保持")
    p.add_argument("--step-from", type=float, default=8.0, help="阶跃起点 cm")
    p.add_argument("--step-to", type=float, default=18.0, help="阶跃终点 cm")
    p.add_argument("--step-frames", type=int, default=600, help="阶跃段采集帧数")
    p.add_argument("--step-marker-ms", type=int, default=2000, help="阶跃提示时刻（采集开始后 ms）")
    p.add_argument("--outdir", default=os.path.join("tools", "data"))
    return p.parse_args()


def open_port(port, baud):
    ser = serial.Serial(port, baud, timeout=0.2)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send(ser, cmd):
    ser.write((cmd + "\n").encode("ascii"))
    ser.flush()
    print(f"  >> {cmd}")


def collect(ser, seg, nominal_cm, n_frames, rows):
    """采集 n_frames 个 8 通道帧；返回实际帧数。"""
    got = 0
    t0 = time.perf_counter()
    while got < n_frames:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("ascii", "ignore").strip()
        except UnicodeDecodeError:
            continue
        m = FRAME_RE.match(line)
        if not m:
            continue
        vals = [int(g) for g in m.groups()]
        t_ms = int((time.perf_counter() - t0) * 1000)
        # t_ms, seg, nominal_cm, raw, filt, pos_cm_x10, status, valid, alpha, hold, fw_seg
        rows.append([t_ms, seg, round(nominal_cm, 2)] + vals[:6] + [vals[7], vals[6]])
        got += 1
        if got % 100 == 0:
            print(f"    seg={seg} {got}/{n_frames} 帧")
    return got


def main():
    args = parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    try:
        ser = open_port(args.port, args.baud)
    except Exception as exc:
        print(f"串口打开失败：{args.port} -> {exc}")
        sys.exit(1)
    print(f"串口已打开：{args.port} @ {args.baud}")

    send(ser, "R")
    time.sleep(0.2)
    if 20 <= args.alpha <= 1000:
        send(ser, f"A{args.alpha}")
        time.sleep(0.2)

    input(f"请将小球放在 {args.zero_cm} cm 零点刻度并稳定，按回车发送 Z 标定零点...")
    send(ser, "Z")
    time.sleep(0.8)
    if args.hold:
        send(ser, "H1")
        time.sleep(0.8)
        print("  平台保持已使能（H1）；如电机 EMI 影响 I2C，可发送 H0 关闭。")

    positions = list(range(args.t_min, args.t_max + 1, args.step_cm))
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(args.outdir, f"tof_grad_{stamp}.csv")
    meta_path = os.path.join(args.outdir, f"tof_grad_{stamp}.json")
    rows = []

    print("\n===== 1cm 梯度采集开始 =====")
    print(f"刻度序列（cm）：{positions[0]} → {positions[-1]}，步进 {args.step_cm}cm")
    print("规则：小球稳定在目标刻度后按回车；采集期间不要触碰装置。\n")

    for seg, pos in enumerate(positions, start=1):
        nominal = pos - args.zero_cm
        input(f"[{seg}/{len(positions)}] 将小球放到 {pos} cm 刻度并稳定，按回车开始采集...")
        send(ser, f"S{seg}")
        time.sleep(0.5)  # 等待手离开 + 滤波器收敛
        collect(ser, seg, nominal, args.samples, rows)
        print(f"  完成 seg={seg} nominal={nominal}cm")

    print("\n===== 阶跃响应测试（延迟分析，seg=99） =====")
    input(f"将小球放在 {args.step_from} cm 并稳定，按回车开始；{args.step_marker_ms/1000:.1f}s 后响铃提示移动...")
    send(ser, "S99")
    time.sleep(0.5)

    def beep_hint():
        print("\a")
        print(f"  >>> 现在快速、平稳地把小球从 {args.step_from} cm 推到 {args.step_to} cm 并保持 <<<")

    timer = threading.Timer(args.step_marker_ms / 1000.0, beep_hint)
    timer.start()
    collect(ser, 99, args.step_to - args.zero_cm, args.step_frames, rows)
    timer.cancel()
    ser.close()

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "t_ms", "seg", "nominal_cm", "raw_mm", "filt_mm", "pos_cm_x10",
            "status", "valid_cnt", "alpha_milli", "hold", "fw_seg",
        ])
        w.writerows(rows)

    meta = {
        "created": stamp,
        "port": args.port,
        "baud": args.baud,
        "zero_cm": args.zero_cm,
        "t_min": args.t_min,
        "t_max": args.t_max,
        "step_cm": args.step_cm,
        "samples_per_point": args.samples,
        "alpha_milli": args.alpha,
        "hold": bool(args.hold),
        "step_from_cm": args.step_from,
        "step_to_cm": args.step_to,
        "step_marker_ms": args.step_marker_ms,
        "frame_layout": "raw_mm,filt_mm,pos_cm_x10,status,valid_cnt,alpha_milli,seg,hold",
        "total_frames": len(rows),
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)

    print(f"\n采集完成：{len(rows)} 帧")
    print(f"CSV : {csv_path}")
    print(f"META: {meta_path}")
    print("下一步：")
    print(f"  python tools/tof_eval.py --csv \"{csv_path}\" --meta \"{meta_path}\"")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hilab_PID ToF 滤波效果评估 / 报告生成

输入：tools/tof_acquire.py 生成的 8 通道 CSV
输出：HTML 报告 + JSON 指标摘要

评估维度：
  1) 有效数据占比：status==0、量程内、静态段无 >jump 跳变；
  2) 位置准确性：每个 1cm 梯度点的滤波输出 vs 实际刻度，MAE/最大误差/误差直方图；
  3) 延迟：阶跃段 raw/filtered 50% 响应时刻，及滤波附加延迟；
  4) 滤波效果：逐段标准差、跳变次数、平均绝对一阶差分（平滑度）。

用法：
  python tools/tof_eval.py --csv tools/data/tof_grad_xxx.csv --meta tools/data/tof_grad_xxx.json
  python tools/tof_eval.py --demo

依赖：matplotlib（pip install matplotlib）
"""

import argparse
import base64
import csv
import html
import io
import json
import math
import os
import random
import statistics as st
import sys
import time

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    print("缺少 matplotlib，请先执行：pip install matplotlib")
    sys.exit(1)

matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False

MIN_MM = 10
MAX_MM = 250
JUMP_MM = 30.0
STEP_SEG = 99


# ============================ 数据读取 ====================================

def load_csv(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        rd = csv.DictReader(f)
        for r in rd:
            try:
                rows.append({
                    "t_ms": int(r["t_ms"]),
                    "seg": int(r["seg"]),
                    "nominal_cm": float(r["nominal_cm"]),
                    "raw_mm": int(r["raw_mm"]),
                    "filt_mm": int(r["filt_mm"]),
                    "pos_cm_x10": int(r["pos_cm_x10"]),
                    "status": int(r["status"]),
                    "valid_cnt": int(r["valid_cnt"]),
                    "alpha_milli": int(r["alpha_milli"]),
                    "hold": int(r["hold"]),
                    "fw_seg": int(r.get("fw_seg", r["seg"])),
                })
            except (KeyError, ValueError):
                continue
    return rows


def load_meta(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def seg_rows(rows, seg):
    return sorted([r for r in rows if r["seg"] == seg], key=lambda x: x["t_ms"])


def static_segments(rows):
    return sorted({r["seg"] for r in rows if r["seg"] != STEP_SEG})


# ============================ 指标计算 ====================================

def in_range(raw):
    return MIN_MM <= raw <= MAX_MM


def static_segment_metrics(rows, seg, zero_cm, jump_mm=JUMP_MM):
    items = seg_rows(rows, seg)
    if not items:
        return None
    nominal = st.median([r["nominal_cm"] for r in items])
    raw_rel = []
    filt_pos = []
    raw_vals = []
    filt_vals = []
    valid_total = 0
    valid_jump = 0
    prev_raw = None
    jump_raw_cnt = 0
    jump_filt_cnt = 0
    prev_f = None

    for r in items:
        ok = (r["status"] == 0 and in_range(r["raw_mm"]))
        if not ok:
            continue
        valid_total += 1
        raw = float(r["raw_mm"])
        f = float(r["filt_mm"])
        raw_vals.append(raw)
        filt_vals.append(f)
        raw_rel.append(raw / 10.0)
        filt_pos.append(r["pos_cm_x10"] / 10.0)

        jump_ok = (prev_raw is None) or (abs(raw - prev_raw) <= jump_mm)
        if jump_ok:
            valid_jump += 1
        prev_raw = raw

        if prev_f is not None:
            if abs(raw - prev_f) > 15.0:
                jump_raw_cnt += 1
            if abs(f - prev_f) > 15.0:
                jump_filt_cnt += 1
        prev_f = f

    if not raw_vals:
        return None

    # 原始值的相对位置 = raw_mm/10 - zero；nominal 已经相对 zero。
    raw_rel_cm = [v - zero_cm for v in raw_rel]
    err_filt = [p - nominal for p in filt_pos]
    err_raw = [p - nominal for p in raw_rel_cm]

    def madiff(vals):
        return st.mean([abs(b - a) for a, b in zip(vals, vals[1:])]) if len(vals) > 1 else 0.0

    return {
        "seg": seg,
        "nominal": nominal,
        "n_total": len(items),
        "n_valid": valid_total,
        "n_valid_jump": valid_jump,
        "mean_filt": st.mean(filt_pos),
        "mean_raw": st.mean(raw_rel_cm),
        "std_filt": st.pstdev(filt_vals) if len(filt_vals) > 1 else 0.0,
        "std_raw": st.pstdev(raw_vals) if len(raw_vals) > 1 else 0.0,
        "err_filt": st.mean(err_filt),
        "err_raw": st.mean(err_raw),
        "mae_filt": st.mean(abs(e) for e in err_filt),
        "mae_raw": st.mean(abs(e) for e in err_raw),
        "max_err_filt": max(abs(e) for e in err_filt),
        "max_err_raw": max(abs(e) for e in err_raw),
        "jump_raw": jump_raw_cnt,
        "jump_filt": jump_filt_cnt,
        "smooth_raw": madiff(raw_vals),
        "smooth_filt": madiff(filt_vals),
        "err_samples": err_filt,
    }


def altitude_metrics(rows, zero_cm, jump_mm=JUMP_MM):
    segs = static_segments(rows)
    stats = []
    for s in segs:
        m = static_segment_metrics(rows, s, zero_cm, jump_mm)
        if m:
            stats.append(m)
    if not stats:
        return None

    # 有效占比的分母必须包含“整段都无效（如 0~2cm 超最小量程）”的帧
    total = sum(1 for r in rows if r["seg"] != STEP_SEG)
    valid = sum(m["n_valid_jump"] for m in stats)
    errors = [e for m in stats for e in m["err_samples"]]
    mean_err = [m["err_filt"] for m in stats]
    mae = st.mean(abs(e) for e in mean_err)
    max_err_mean = max(abs(e) for e in mean_err)
    max_err_sample = max(m["max_err_filt"] for m in stats)
    max_err_raw_sample = max(m["max_err_raw"] for m in stats)
    mae_raw = st.mean(m["mae_raw"] for m in stats)

    std_raw = st.mean(m["std_raw"] for m in stats)
    std_filt = st.mean(m["std_filt"] for m in stats)
    jump_raw = sum(m["jump_raw"] for m in stats)
    jump_filt = sum(m["jump_filt"] for m in stats)
    smooth_raw = st.mean(m["smooth_raw"] for m in stats)
    smooth_filt = st.mean(m["smooth_filt"] for m in stats)

    return {
        "segments": stats,
        "total_static": total,
        "valid_static": valid,
        "valid_pct": 100.0 * valid / total if total else 0.0,
        "mae_filt": mae,
        "max_err_filt": max_err_mean,
        "max_err_filt_sample": max_err_sample,
        "max_err_raw_sample": max_err_raw_sample,
        "mae_raw": mae_raw,
        "errors": errors,
        "std_raw": std_raw,
        "std_filt": std_filt,
        "noise_reduction": (std_raw / std_filt) if std_filt > 1e-9 else 0.0,
        "jump_raw": jump_raw,
        "jump_filt": jump_filt,
        "jump_reduction": (jump_raw / jump_filt) if jump_filt > 0 else float("inf") if jump_raw > 0 else 1.0,
        "smooth_raw": smooth_raw,
        "smooth_filt": smooth_filt,
        "smooth_reduction": (smooth_raw / smooth_filt) if smooth_filt > 1e-9 else 0.0,
    }


def crossing_time(items, key, level, rising):
    for r in items:
        v = r[key]
        if rising and v >= level:
            return r["t_ms"]
        if (not rising) and v <= level:
            return r["t_ms"]
    return None


def step_metrics(rows, marker_ms=2000):
    items = seg_rows(rows, STEP_SEG)
    if not items:
        return None
    pre = [r for r in items if 1200 <= r["t_ms"] <= marker_ms - 100]
    post = items[-100:]
    if not pre or not post:
        return None
    raw = [float(r["raw_mm"]) for r in items]
    filt = [float(r["filt_mm"]) for r in items]
    pre_med = st.median([r["raw_mm"] for r in pre])
    post_med = st.median([r["raw_mm"] for r in post])
    if abs(post_med - pre_med) < 20:
        return None
    rising = post_med > pre_med
    mid = (pre_med + post_med) / 2.0
    t_raw50 = crossing_time(items, "raw_mm", mid, rising)
    t_filt50 = crossing_time(items, "filt_mm", mid, rising)
    if t_raw50 is None or t_filt50 is None:
        return None

    # 10%-90% 上升时间
    lo = pre_med + (post_med - pre_med) * 0.10
    hi = pre_med + (post_med - pre_med) * 0.90
    t_raw10 = crossing_time(items, "raw_mm", lo, rising)
    t_raw90 = crossing_time(items, "raw_mm", hi, rising)
    t_f10 = crossing_time(items, "filt_mm", lo, rising)
    t_f90 = crossing_time(items, "filt_mm", hi, rising)

    overshoot = 0.0
    if abs(post_med - pre_med) > 1e-9:
        peak = max(filt) if rising else min(filt)
        overshoot = max(0.0, (peak - post_med) / (post_med - pre_med) * 100.0) if rising else max(0.0, (post_med - peak) / (pre_med - post_med) * 100.0)

    return {
        "pre_mm": pre_med,
        "post_mm": post_med,
        "step_mm": post_med - pre_med,
        "t_raw50": t_raw50,
        "t_filt50": t_filt50,
        "delay_raw_ms": t_raw50 - marker_ms,
        "delay_filt_ms": t_filt50 - marker_ms,
        "delay_added_ms": t_filt50 - t_raw50,
        "rise_raw_ms": (t_raw90 - t_raw10) if (t_raw10 is not None and t_raw90 is not None) else None,
        "rise_filt_ms": (t_f90 - t_f10) if (t_f10 is not None and t_f90 is not None) else None,
        "overshoot_pct": overshoot,
        "pre": pre_med,
        "post": post_med,
        "t": [r["t_ms"] for r in items],
        "raw": raw,
        "filt": filt,
    }


# ============================ 离线 alpha 扫描 =============================

def simulate_filter(items, alpha, zero_cm):
    """复刻固件 median-5 + EMA，返回 [(t_ms, filtered_cm)]，无效帧跳过。"""
    buf = []
    ema = None
    out = []
    for r in items:
        if r["status"] != 0 or not in_range(r["raw_mm"]):
            continue
        buf.append(float(r["raw_mm"]))
        if len(buf) > 5:
            buf.pop(0)
        med = sorted(buf)[len(buf) // 2]
        if ema is None:
            ema = med
        else:
            ema += alpha * (med - ema)
        out.append((r["t_ms"], ema / 10.0 - zero_cm))
    return out


def sweep_alpha(rows, zero_cm, t_min, step_cm, marker_ms):
    segs = static_segments(rows)
    results = []
    for alpha_milli in range(50, 1001, 50):
        alpha = alpha_milli / 1000.0
        errs = []
        stds = []
        for s in segs:
            items = seg_rows(rows, s)
            if not items:
                continue
            nominal = st.median([r["nominal_cm"] for r in items])
            sim = simulate_filter(items, alpha, zero_cm)
            vals = [v for _, v in sim]
            if not vals:
                continue
            errs.append(st.mean(vals) - nominal)
            stds.append(st.pstdev(vals) if len(vals) > 1 else 0.0)
        if not errs:
            continue
        mae = st.mean(abs(e) for e in errs)
        noise = st.mean(stds)
        step = step_metrics(rows, marker_ms)
        delay_added = step["delay_added_ms"] if step else 0.0
        score = mae + 0.05 * noise + 0.0005 * max(0.0, delay_added)
        results.append({
            "alpha_milli": alpha_milli,
            "mae_cm": mae,
            "noise_cm": noise,
            "delay_added_ms": delay_added,
            "score": score,
        })
    if not results:
        return [], None
    best = min(results, key=lambda x: x["score"])
    return results, best


# ============================ 图表 ========================================

def fig_to_b64(fig):
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=130, bbox_inches="tight")
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def make_charts(rows, alt, step, sweep):
    charts = {}

    # 1) 全程 raw/filtered（按样本序号拼接，避免每段 t_ms 归零）
    t = []
    raw = []
    filt = []
    bounds = []
    idx = 0
    for s in static_segments(rows):
        items = seg_rows(rows, s)
        bounds.append(idx)
        for r in items:
            t.append(idx)
            raw.append(r["raw_mm"] / 10.0)
            filt.append(r["pos_cm_x10"] / 10.0)
            idx += 1
    fig, ax = plt.subplots(figsize=(11, 3.6))
    ax.plot(t, raw, color="#c33", lw=0.9, label="原始 (cm)")
    ax.plot(t, filt, color="#369", lw=1.2, label="滤波后 (cm)")
    for b in bounds[1:]:
        ax.axvline(b, color="#ddd", lw=0.6)
    ax.set_xlabel("样本序号（按 1cm 梯度段顺序）")
    ax.set_ylabel("位置 (cm)")
    ax.set_title("全程原始 vs 滤波后位置")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(alpha=0.25)
    charts["timeseries"] = fig_to_b64(fig)

    # 2) 逐段误差
    segs = alt["segments"]
    x = [m["nominal"] for m in segs]
    y_raw = [m["err_raw"] for m in segs]
    y_filt = [m["err_filt"] for m in segs]
    fig, ax = plt.subplots(figsize=(10, 3.6))
    ax.plot(x, y_raw, "o-", color="#c33", ms=3, lw=0.9, label="原始误差")
    ax.plot(x, y_filt, "s-", color="#369", ms=3, lw=1.2, label="滤波后误差")
    ax.axhline(0, color="#888", lw=0.8)
    ax.set_xlabel("实际位置 (cm)")
    ax.set_ylabel("均值误差 (cm)")
    ax.set_title("每个 1cm 梯度点的位置误差")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.25)
    charts["error"] = fig_to_b64(fig)

    # 3) 逐段噪声
    fig, ax = plt.subplots(figsize=(10, 3.6))
    width = 0.38
    xs = list(range(len(segs)))
    ax.bar([v - width / 2 for v in xs], [m["std_raw"] for m in segs], width, color="#c33", label="原始 std")
    ax.bar([v + width / 2 for v in xs], [m["std_filt"] for m in segs], width, color="#369", label="滤波后 std")
    ax.set_xticks(xs)
    ax.set_xticklabels([f"{m['nominal']:.0f}" for m in segs], fontsize=8)
    ax.set_xlabel("实际位置 (cm)")
    ax.set_ylabel("标准差 (mm)")
    ax.set_title("逐段噪声：原始 vs 滤波后")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.25)
    charts["noise"] = fig_to_b64(fig)

    # 4) 误差直方图
    fig, ax = plt.subplots(figsize=(8, 3.4))
    ax.hist(alt["errors"], bins=40, color="#369", edgecolor="white")
    ax.axvline(0, color="#888", lw=0.8)
    ax.set_xlabel("滤波后误差 (cm)")
    ax.set_ylabel("样本数")
    ax.set_title("滤波后误差分布直方图")
    ax.grid(alpha=0.25)
    charts["hist"] = fig_to_b64(fig)

    # 5) 阶跃
    if step:
        fig, ax = plt.subplots(figsize=(10, 3.6))
        ax.plot(step["t"], [v / 10.0 for v in step["raw"]], color="#c33", lw=1.0, label="原始")
        ax.plot(step["t"], [v / 10.0 for v in step["filt"]], color="#369", lw=1.4, label="滤波后")
        ax.axvline(step["t_raw50"], color="#c33", ls="--", lw=0.8, label="raw 50%")
        ax.axvline(step["t_filt50"], color="#369", ls="--", lw=0.8, label="filter 50%")
        ax.set_xlabel("时间 (ms)")
        ax.set_ylabel("距离 (cm)")
        ax.set_title("阶跃响应：原始 vs 滤波后")
        ax.legend(fontsize=8)
        ax.grid(alpha=0.25)
        charts["step"] = fig_to_b64(fig)

    # 6) alpha 扫描
    if sweep:
        fig, ax = plt.subplots(figsize=(9, 3.4))
        ax.plot([r["alpha_milli"] for r in sweep], [r["mae_cm"] for r in sweep], "o-", color="#369", ms=3, label="MAE (cm)")
        ax.set_xlabel("EMA alpha (千分比)")
        ax.set_ylabel("MAE (cm)")
        ax2 = ax.twinx()
        ax2.plot([r["alpha_milli"] for r in sweep], [r["noise_cm"] for r in sweep], "s--", color="#c33", ms=3, label="噪声 std (cm)")
        ax2.set_ylabel("噪声 std (cm)")
        ax.set_title("alpha 扫描：精度 vs 噪声")
        ax.grid(alpha=0.25)
        charts["sweep"] = fig_to_b64(fig)

    return charts


# ============================ HTML 报告 ===================================

def fmt(v, nd=2, unit=""):
    if v is None:
        return "N/A"
    return f"{v:.{nd}f}{unit}"


def build_html(rows, alt, step, sweep, best, charts, meta, source_label):
    demo = bool(meta.get("demo", False))
    banner = ""
    if demo:
        banner = (
            '<div class="banner">仿真数据：本报告用于验证采集/分析链路与滤波器设计，'
            '不是实物测量结果。实物测试后请用真实 CSV 重新生成。</div>'
        )

    def img(key, caption):
        if key not in charts:
            return ""
        return f'<figure><img src="data:image/png;base64,{charts[key]}" alt="{html.escape(caption)}"/><figcaption>{html.escape(caption)}</figcaption></figure>'

    seg_rows_html = ""
    for m in alt["segments"]:
        seg_rows_html += (
            "<tr>"
            f"<td>{m['seg']}</td><td>{m['nominal']:.1f}</td>"
            f"<td>{m['n_valid']}</td><td>{m['n_valid_jump']}</td>"
            f"<td>{m['mean_raw']:.2f}</td><td>{m['mean_filt']:.2f}</td>"
            f"<td>{m['err_raw']:+.2f}</td><td>{m['err_filt']:+.2f}</td>"
            f"<td>{m['std_raw']:.2f}</td><td>{m['std_filt']:.2f}</td>"
            f"<td>{m['jump_raw']}</td><td>{m['jump_filt']}</td>"
            "</tr>"
        )

    sweep_html = ""
    for r in sweep:
        cls = ' class="best"' if best and r["alpha_milli"] == best["alpha_milli"] else ""
        sweep_html += (
            f"<tr{cls}><td>{r['alpha_milli']}</td><td>{r['mae_cm']:.3f}</td>"
            f"<td>{r['noise_cm']:.3f}</td><td>{r['delay_added_ms']:.0f}</td>"
            f"<td>{r['score']:.4f}</td></tr>"
        )

    step_html = ""
    if step:
        step_html = f"""
        <table>
          <tr><th>指标</th><th>原始</th><th>滤波后</th><th>差值</th></tr>
          <tr><td>50% 响应时刻 (ms)</td><td>{step['t_raw50']}</td><td>{step['t_filt50']}</td><td>{step['delay_added_ms']:.0f}</td></tr>
          <tr><td>相对阶跃提示的延迟 (ms)</td><td>{step['delay_raw_ms']:.0f}</td><td>{step['delay_filt_ms']:.0f}</td><td>{step['delay_added_ms']:.0f}</td></tr>
          <tr><td>10–90% 上升时间 (ms)</td><td>{fmt(step['rise_raw_ms'],0)}</td><td>{fmt(step['rise_filt_ms'],0)}</td><td>—</td></tr>
          <tr><td>超调 (%)</td><td>—</td><td>{step['overshoot_pct']:.1f}</td><td>—</td></tr>
          <tr><td>阶跃幅值 (mm)</td><td colspan="3">{step['step_mm']:.1f}</td></tr>
        </table>
        <p class="note">绝对延迟包含“响铃提示 → 人手移动”的反应时间，因此原始与滤波的绝对延迟都偏大；
        两者之差的 <b>{step['delay_added_ms']:.0f} ms</b> 才是滤波器附加延迟，是更有意义的指标。</p>
        """
    else:
        step_html = '<p class="warn">未检测到有效阶跃段（seg=99），请检查采集脚本与移动幅度。</p>'

    best_alpha = best["alpha_milli"] if best else None
    rec_line = (
        f"建议 alpha = <b>{best_alpha}</b> / 1000"
        if best_alpha is not None
        else "样本不足，未给出 alpha 建议"
    )
    step_delay_text = f"{step['delay_added_ms']:.0f} ms" if step else "N/A"

    html_doc = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>Hilab_PID ToF 滤波评估报告</title>
<style>
body{{font-family:"Microsoft YaHei",sans-serif;max-width:1100px;margin:24px auto;padding:0 18px;color:#222;background:#fafafa;line-height:1.65}}
h1{{font-size:22px;border-bottom:2px solid #369;padding-bottom:8px}}
h2{{font-size:17px;margin-top:28px;border-left:4px solid #369;padding-left:8px}}
h3{{font-size:15px;margin-top:20px}}
table{{border-collapse:collapse;font-size:13px;margin:10px 0;width:100%}}
td,th{{border:1px solid #c8cdd4;padding:5px 8px;text-align:center}}
th{{background:#e8eef7}}
tr.best td{{background:#e6f7ee;font-weight:600}}
figure{{margin:14px 0;background:#fff;border:1px solid #ddd;padding:8px}}
figure img{{width:100%;height:auto;display:block}}
figcaption{{font-size:12px;color:#555;text-align:center;margin-top:6px}}
.note{{font-size:13px;color:#555}}
.warn{{color:#b33;font-size:13px}}
.banner{{background:#fff3cd;border-left:4px solid #e0a800;padding:10px 14px;margin:12px 0;font-size:13px}}
.kpi{{display:flex;gap:12px;flex-wrap:wrap;margin:12px 0}}
.card{{background:#fff;border:1px solid #ddd;border-radius:8px;padding:10px 14px;min-width:150px}}
.card b{{display:block;font-size:20px;color:#2a5d9f}}
.card span{{font-size:12px;color:#666}}
pre{{background:#f1f3f5;padding:10px;overflow:auto;font-size:12px}}
</style>
</head>
<body>
<h1>Hilab_PID ToF 滤波效果评估报告</h1>
<p class="note">生成时间：{time.strftime('%Y-%m-%d %H:%M:%S')}　|　数据：{html.escape(source_label)}</p>
{banner}

<h2>1. 结论摘要</h2>
<div class="kpi">
  <div class="card"><span>静态段有效占比</span><b>{alt['valid_pct']:.1f}%</b></div>
  <div class="card"><span>滤波后 MAE</span><b>{alt['mae_filt']:.2f} cm</b></div>
  <div class="card"><span>最大单样本误差</span><b>{alt['max_err_filt_sample']:.2f} cm</b></div>
  <div class="card"><span>噪声抑制</span><b>{alt['noise_reduction']:.2f}×</b></div>
  <div class="card"><span>滤波附加延迟</span><b>{step_delay_text}</b></div>
  <div class="card"><span>建议 alpha</span><b>{best_alpha if best_alpha is not None else 'N/A'}</b></div>
</div>
<p>{rec_line}。滤波器采用 <b>5 点中值 + 一阶 EMA</b>，未使用卡尔曼。</p>

<h2>2. 有效数据占比</h2>
<p>统计范围：所有 1cm 静态梯度段（不含阶跃段）。有效条件：<code>status==0</code>、
原始距离在 {MIN_MM}–{MAX_MM} mm、且相邻有效样本跳变不超过 {JUMP_MM:.0f} mm。</p>
<table>
<tr><th>静态总帧数</th><th>有效帧数</th><th>有效占比</th></tr>
<tr><td>{alt['total_static']}</td><td>{alt['valid_static']}</td><td>{alt['valid_pct']:.2f}%</td></tr>
</table>

<h2>3. 位置预测准确性</h2>
<p>误差 = 滤波输出位置 − 实际刻度位置（cm）。MAE 为各梯度点均值误差的绝对平均；
“最大段均值误差”用于评估系统性偏差，“最大单样本误差”用于评估噪声尖峰。</p>
<table>
<tr><th>指标</th><th>原始</th><th>滤波后</th></tr>
<tr><td>MAE (cm)</td><td>{alt['mae_raw']:.2f}</td><td>{alt['mae_filt']:.2f}</td></tr>
<tr><td>最大段均值误差 (cm)</td><td>—</td><td>{alt['max_err_filt']:.2f}</td></tr>
<tr><td>最大单样本误差 (cm)</td><td>{alt['max_err_raw_sample']:.2f}</td><td>{alt['max_err_filt_sample']:.2f}</td></tr>
</table>
{img('error', '每个 1cm 梯度点的均值位置误差')}
{img('hist', '滤波后误差分布直方图')}

<h3>逐梯度点明细</h3>
<table>
<tr><th>seg</th><th>实际位置(cm)</th><th>有效帧</th><th>跳变门限后有效</th>
<th>raw均值(cm)</th><th>滤波均值(cm)</th><th>raw误差(cm)</th><th>滤波误差(cm)</th>
<th>raw std(mm)</th><th>滤波 std(mm)</th><th>raw跳变</th><th>滤波跳变</th></tr>
{seg_rows_html}
</table>

<h2>4. 数据延迟分析</h2>
{step_html}
{img('step', '阶跃响应：原始与滤波后 50% 响应时刻')}

<h2>5. 滤波效果验证</h2>
<table>
<tr><th>指标</th><th>原始</th><th>滤波后</th><th>改善</th></tr>
<tr><td>平均段内标准差 (mm)</td><td>{alt['std_raw']:.2f}</td><td>{alt['std_filt']:.2f}</td><td>{alt['noise_reduction']:.2f}×</td></tr>
<tr><td>15mm 以上跳变次数</td><td>{alt['jump_raw']}</td><td>{alt['jump_filt']}</td><td>{alt['jump_reduction']:.2f}×</td></tr>
<tr><td>平均绝对一阶差分 (mm)</td><td>{alt['smooth_raw']:.2f}</td><td>{alt['smooth_filt']:.2f}</td><td>{alt['smooth_reduction']:.2f}×</td></tr>
</table>
{img('noise', '逐段噪声：原始 vs 滤波后')}
{img('timeseries', '全程原始 vs 滤波后位置')}

<h2>6. 参数优化</h2>
<p>扫描 alpha=50…1000（千分比），用同一组原始数据离线复刻 median-5 + EMA。
综合评分 = MAE + 0.05×噪声std + 0.0005×滤波附加延迟(ms)。</p>
<table>
<tr><th>alpha</th><th>MAE(cm)</th><th>噪声std(cm)</th><th>附加延迟(ms)</th><th>score</th></tr>
{sweep_html}
</table>
{img('sweep', 'alpha 扫描：MAE 与噪声')}
<p class="note">在线部署：串口发送 <code>A&lt;milli&gt;</code> 立即生效（无需重烧）；
永久部署：把 <code>TOF_FILTER_ALPHA_DEFAULT_MILLI</code> 改为推荐值后重新编译。
当前固件默认值 300。</p>

<h2>7. 测距模块模式与参数建议</h2>
<ul>
<li>保持 <b>SHORT 模式</b>：250mm 近距离最优、抗环境光；本测试台行程在 ±8cm 量级，足够。</li>
<li>ROI 保持 <b>8×8</b>（TL 4,11 → BR 11,4）：近距离最小量程约 35mm；继续缩小 ROI 会降低信噪比。</li>
<li>测量预算 16ms + 帧周期 20ms：满足库要求的“帧周期 ≥ 预算 + 4ms”；采样 20Hz 留足余量。</li>
<li>必须用 <code>range_status</code> 门控：0 才进入滤波器；5/7 等弱回波帧保持上一有效输出。</li>
<li>测试梯度建议 0–25cm、步长 1cm；0–3cm 可能因最小量程被拒绝，报告中按有效帧统计。</li>
<li>若成片 5/7 或 -1/-2，先查供电/接地/I2C 上拉与电机 EMI，再调滤波参数。</li>
</ul>

<h2>8. 方法与注意事项</h2>
<ul>
<li>“原始延迟”以采集脚本的响铃提示为时间参考，包含人的反应时间；<b>滤波附加延迟</b>（filtered − raw 50% 时刻）才是滤波器本身的代价。</li>
<li>有效占比只统计静态梯度段；阶跃段的大幅移动会被跳变门限判为异常，故不混入。</li>
<li>误差直方图使用所有静态有效样本；逐点表使用段均值。</li>
<li>本报告中的绝对值只有在真实物理测试后才有意义；仿真报告仅验证分析链路。</li>
</ul>

<h2>9. 复现命令</h2>
<pre>python tools/tof_acquire.py --port COM13 --t-min 0 --t-max 25 --samples 300
python tools/tof_eval.py --csv tools/data/tof_grad_xxx.csv --meta tools/data/tof_grad_xxx.json</pre>

</body>
</html>"""
    return html_doc


# ============================ 仿真数据 ====================================

def make_demo(outdir):
    os.makedirs(outdir, exist_ok=True)
    rng = random.Random(42)
    rows = []
    valid_cnt = 0
    alpha_milli = 300
    zero_cm = 3.0
    t_global = 0

    def add_frame(seg, nominal, raw, status, t_ms):
        nonlocal valid_cnt
        if status == 0 and in_range(raw):
            valid_cnt += 1
        rows.append({
            "t_ms": t_ms,
            "seg": seg,
            "nominal_cm": nominal,
            "raw_mm": int(round(raw)),
            "filt_mm": 0,  # 稍后统一滤波
            "pos_cm_x10": 0,
            "status": status,
            "valid_cnt": valid_cnt,
            "alpha_milli": alpha_milli,
            "hold": 0,
            "fw_seg": seg,
        })

    # 1cm 梯度段：3..25cm（避开 Short 模式最小量程）
    for seg, pos in enumerate(range(3, 26), start=1):
        nominal = pos - zero_cm
        for _ in range(300):
            raw = pos * 10.0 + rng.gauss(0, 2.5)
            status = 0
            if rng.random() < 0.04:
                status = rng.choice([5, 7])
            elif rng.random() < 0.015:
                raw += rng.choice([-1, 1]) * rng.uniform(15, 35)
            add_frame(seg, nominal, raw, status, t_global)
            t_global += 50

    # 阶跃段：8cm → 18cm，2s 后开始 0.3s 过渡
    seg = STEP_SEG
    for i in range(600):
        t = i * 50
        if t < 2000:
            pos = 8.0
        elif t < 2300:
            pos = 8.0 + 10.0 * (t - 2000) / 300.0
        else:
            pos = 18.0
        raw = pos * 10.0 + rng.gauss(0, 2.5)
        status = 0 if rng.random() > 0.02 else 5
        add_frame(seg, 18.0 - zero_cm, raw, status, t)

    # 用固件同款 median-5 + EMA 生成 filt/pos
    by_seg = {}
    for r in rows:
        by_seg.setdefault(r["seg"], []).append(r)
    for seg, items in by_seg.items():
        buf = []
        ema = None
        for r in items:
            if r["status"] != 0 or not in_range(r["raw_mm"]):
                continue
            buf.append(float(r["raw_mm"]))
            if len(buf) > 5:
                buf.pop(0)
            med = sorted(buf)[len(buf) // 2]
            if ema is None:
                ema = med
            else:
                ema += (alpha_milli / 1000.0) * (med - ema)
            r["filt_mm"] = int(round(ema))
            r["pos_cm_x10"] = int(round(ema - zero_cm * 10.0))
    # 无效帧沿用上一有效值（与固件输出保持语义一致）
    last_f, last_p = 0, 0
    for r in rows:
        if r["filt_mm"] == 0 and r["status"] != 0:
            r["filt_mm"] = last_f
            r["pos_cm_x10"] = last_p
        last_f, last_p = r["filt_mm"], r["pos_cm_x10"]

    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(outdir, f"demo_tof_{stamp}.csv")
    meta_path = os.path.join(outdir, f"demo_tof_{stamp}.json")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "seg", "nominal_cm", "raw_mm", "filt_mm", "pos_cm_x10",
                    "status", "valid_cnt", "alpha_milli", "hold", "fw_seg"])
        for r in rows:
            w.writerow([r["t_ms"], r["seg"], r["nominal_cm"], r["raw_mm"], r["filt_mm"],
                        r["pos_cm_x10"], r["status"], r["valid_cnt"], r["alpha_milli"],
                        r["hold"], r["fw_seg"]])
    meta = {
        "created": stamp,
        "demo": True,
        "zero_cm": zero_cm,
        "t_min": 3,
        "t_max": 25,
        "step_cm": 1,
        "samples_per_point": 300,
        "alpha_milli": alpha_milli,
        "step_marker_ms": 2000,
        "total_frames": len(rows),
    }
    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)
    return csv_path, meta_path, meta


# ============================ 主流程 ======================================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv")
    ap.add_argument("--meta")
    ap.add_argument("--demo", action="store_true", help="生成仿真数据并评估（仅验证链路）")
    ap.add_argument("--out", default=os.path.join("tools", "data", "tof_filter_report.html"))
    ap.add_argument("--json-out", default=os.path.join("tools", "data", "tof_metrics.json"))
    ap.add_argument("--zero-cm", type=float, default=None)
    ap.add_argument("--t-min", type=int, default=None)
    ap.add_argument("--step-cm", type=int, default=None)
    ap.add_argument("--marker-ms", type=int, default=None)
    args = ap.parse_args()

    if args.demo:
        csv_path, meta_path, meta = make_demo(os.path.join("tools", "data"))
    else:
        if not args.csv:
            ap.error("需要 --csv，或使用 --demo")
        csv_path = args.csv
        meta_path = args.meta
        meta = load_meta(meta_path)

    rows = load_csv(csv_path)
    if not rows:
        print("CSV 无有效数据")
        sys.exit(1)

    zero_cm = args.zero_cm if args.zero_cm is not None else float(meta.get("zero_cm", 0))
    marker_ms = args.marker_ms if args.marker_ms is not None else int(meta.get("step_marker_ms", 2000))

    alt = altitude_metrics(rows, zero_cm)
    if alt is None:
        print("没有可用的静态梯度段")
        sys.exit(1)
    step = step_metrics(rows, marker_ms)
    sweep, best = sweep_alpha(rows, zero_cm, 0, 1, marker_ms)
    charts = make_charts(rows, alt, step, sweep)
    html_doc = build_html(rows, alt, step, sweep, best, charts, meta, os.path.basename(csv_path))

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(html_doc)

    metrics = {
        "source": csv_path,
        "demo": bool(meta.get("demo", False)),
        "valid_pct": alt["valid_pct"],
        "mae_filt_cm": alt["mae_filt"],
        "max_err_filt_mean_cm": alt["max_err_filt"],
        "max_err_filt_sample_cm": alt["max_err_filt_sample"],
        "max_err_raw_sample_cm": alt["max_err_raw_sample"],
        "mae_raw_cm": alt["mae_raw"],
        "std_raw_mm": alt["std_raw"],
        "std_filt_mm": alt["std_filt"],
        "noise_reduction": alt["noise_reduction"],
        "jump_raw": alt["jump_raw"],
        "jump_filt": alt["jump_filt"],
        "smooth_reduction": alt["smooth_reduction"],
        "step": {k: v for k, v in (step or {}).items() if k not in ("t", "raw", "filt")},
        "recommended_alpha_milli": best["alpha_milli"] if best else None,
        "alpha_sweep": sweep,
    }
    with open(args.json_out, "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    print(f"CSV : {csv_path}")
    print(f"HTML: {args.out}")
    print(f"JSON: {args.json_out}")
    print(f"有效占比 {alt['valid_pct']:.1f}% | MAE {alt['mae_filt']:.2f}cm | "
          f"噪声抑制 {alt['noise_reduction']:.2f}x | 附加延迟 "
          f"{(step['delay_added_ms'] if step else float('nan')):.0f}ms | "
          f"建议 alpha {best['alpha_milli'] if best else 'N/A'}")


if __name__ == "__main__":
    main()

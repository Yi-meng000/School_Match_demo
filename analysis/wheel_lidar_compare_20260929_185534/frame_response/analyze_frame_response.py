#!/usr/bin/env python3
"""Compare new chassis commands with the feedback available before the next command."""

import argparse
import json
import sqlite3
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_bag(bag):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    database = next(bag.glob("*.db3"))
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    result = {}
    wanted = {"/cmd_chassis", "/cmd_track", "/wheel_velocity", "/OdometryHighFreq"}
    for topic_id, topic, message_type in connection.execute("SELECT id,name,type FROM topics"):
        if topic not in wanted:
            continue
        cls = get_message(message_type)
        rows = []
        query = "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp"
        for timestamp, raw in connection.execute(query, (topic_id,)):
            message = deserialize_message(raw, cls)
            t = timestamp * 1e-9
            if topic == "/cmd_chassis":
                rows.append((t, message.vx, message.vy, message.vw))
            elif topic == "/cmd_track":
                rows.append((t, message.linear.x, message.linear.y, message.angular.z))
            else:
                stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
                twist = message.twist if topic == "/wheel_velocity" else message.twist.twist
                rows.append((t, stamp, twist.linear.x, twist.linear.y, twist.angular.z))
        result[topic] = np.asarray(rows, dtype=float)
    connection.close()
    return result


def last_before(data, time, inclusive=False):
    side = "right" if inclusive else "left"
    return int(np.searchsorted(data[:, 0], time, side=side) - 1)


def onset(data, start, sign, axis, threshold):
    selected = data[(data[:, 0] >= start) & (data[:, 0] <= start + 1.0)]
    valid = sign * selected[:, 2 + axis] > threshold
    hits = np.flatnonzero(np.convolve(valid.astype(int), np.ones(3, dtype=int), "valid") == 3)
    if len(hits):
        # Report the first of three consecutive qualifying packets, not the third.
        return float((selected[hits[0], 0] - start) * 1000)
    return None


def summary(records):
    if not records:
        return {}
    duration = sum(row["dt_s"] for row in records)
    command_change = sum(row["command_delta_main"] for row in records)
    wheel_change = sum(row["wheel_delta_main"] for row in records)
    eligible = [row for row in records if row["target_main"] >= 0.3]
    error = np.array([row["target_main"] - row["wheel_end_main"] for row in records])
    return {
        "intervals": len(records),
        "duration_s": duration,
        "dt_median_ms": float(np.median([row["dt_s"] for row in records]) * 1000),
        "command_delta_mean": command_change / len(records),
        "command_delta_median": float(np.median([row["command_delta_main"] for row in records])),
        "wheel_delta_mean": wheel_change / len(records),
        "wheel_delta_median": float(np.median([row["wheel_delta_main"] for row in records])),
        "command_rate": command_change / duration,
        "wheel_rate": wheel_change / duration,
        "rate_difference": (command_change - wheel_change) / duration,
        "target_error_median": float(np.median(error)),
        "target_error_q25_q75": np.percentile(error, [25, 75]).tolist(),
        "target_match_count": sum(row["matches_target_at_end"] for row in eligible),
        "target_match_denominator": len(eligible),
        "target_match_percent": 100 * sum(row["matches_target_at_end"] for row in eligible) / len(eligible) if eligible else None,
        "ever_in_tolerance_percent": 100 * sum(row["ever_in_tolerance"] for row in eligible) / len(eligible) if eligible else None,
        "mean_abs_delta_vx": float(np.mean([abs(row["command_delta_vx"]) for row in records])),
        "mean_abs_delta_vy": float(np.mean([abs(row["command_delta_vy"]) for row in records])),
    }


def analyze(data):
    commands = data["/cmd_chassis"]
    track = data["/cmd_track"]
    wheel = data["/wheel_velocity"]
    lidar = data["/OdometryHighFreq"]
    t0 = min(value[0, 0] for value in data.values())
    active = np.flatnonzero(np.hypot(commands[:, 1], commands[:, 2]) > 0.15)
    spans = []
    first = previous = active[0]
    for index in active[1:]:
        if commands[index, 0] - commands[previous, 0] > 1.0:
            spans.append((commands[first, 0], commands[previous, 0]))
            first = index
        previous = index
    spans.append((commands[first, 0], commands[previous, 0]))

    changed = np.r_[True, np.any(commands[1:, 1:4] != commands[:-1, 1:4], axis=1)]
    runs = []
    records = []
    for motion_number, (begin, end) in enumerate(spans, 1):
        tracking = track[(track[:, 0] >= begin - 0.7) & (track[:, 0] <= end + 1.0)]
        if not np.any(np.hypot(tracking[:, 1], tracking[:, 2]) > 0.01):
            continue
        current = commands[(commands[:, 0] >= begin - 0.7) & (commands[:, 0] <= end + 1.0) & changed]
        fast = np.hypot(current[:, 1], current[:, 2]) > 2.0
        mean_velocity = np.mean(current[fast, 1:3], axis=0)
        axis = int(np.argmax(np.abs(mean_velocity)))
        sign = float(np.sign(mean_velocity[axis]))
        main = sign * current[:, 1 + axis]
        start = np.flatnonzero(main > 0.01)[0]
        peak = start + int(np.argmax(main[start:]))
        nonpositive = np.flatnonzero(main[peak:] <= 0.01)
        stop = peak + int(nonpositive[0]) if len(nonpositive) else len(main) - 1
        current = current[start:stop + 1]
        main = main[start:stop + 1]
        plateau = np.flatnonzero(main >= 0.95 * max(main))
        acceleration_end = int(plateau[0])
        deceleration_start = int(plateau[-1])
        number = len(runs) + 1
        run_records = []
        for k in range(len(current) - 1):
            t_start, t_end = current[k, 0], current[k + 1, 0]
            i_start = last_before(wheel, t_start, inclusive=True)
            i_end = last_before(wheel, t_end)
            phase = "acceleration" if k < acceleration_end else ("deceleration" if k >= deceleration_start else "cruise")
            if np.any(np.hypot(wheel[i_start:i_end + 1, 2], wheel[i_start:i_end + 1, 3]) > 6.0):
                continue
            v_start = float(sign * wheel[i_start, 2 + axis])
            v_end = float(sign * wheel[i_end, 2 + axis])
            feedback = wheel[(wheel[:, 0] >= t_start) & (wheel[:, 0] < t_end)]
            target = float(main[k])
            row = {
                "run": number,
                "interval": k + 1,
                "phase": phase,
                "t_start_since_bag_s": float(t_start - t0),
                "t_end_since_bag_s": float(t_end - t0),
                "t_start_since_run_s": float(t_start - current[0, 0]),
                "dt_s": float(t_end - t_start),
                "target_vx": float(current[k, 1]),
                "target_vy": float(current[k, 2]),
                "target_main": target,
                "next_target_main": float(main[k + 1]),
                "command_delta_vx": float(current[k + 1, 1] - current[k, 1]),
                "command_delta_vy": float(current[k + 1, 2] - current[k, 2]),
                "command_delta_main": float(main[k + 1] - main[k]),
                "applied_command_step_main": float(main[k] - main[k - 1]) if k else target,
                "wheel_start_vx": float(wheel[i_start, 2]),
                "wheel_start_vy": float(wheel[i_start, 3]),
                "wheel_end_vx": float(wheel[i_end, 2]),
                "wheel_end_vy": float(wheel[i_end, 3]),
                "wheel_start_main": v_start,
                "wheel_end_main": v_end,
                "wheel_delta_main": v_end - v_start,
                "wheel_start_sample_age_ms": float((t_start - wheel[i_start, 0]) * 1000),
                "wheel_end_sample_age_ms": float((t_end - wheel[i_end, 0]) * 1000),
                "matches_target_at_end": bool(abs(v_end - target) <= 0.10),
                "matches_vector_target_at_end": bool(np.linalg.norm(wheel[i_end, 2:4] - current[k, 1:3]) <= 0.10),
                "ever_in_tolerance": bool(np.any(np.abs(sign * feedback[:, 2 + axis] - target) <= 0.10)),
                "crosses_zero_command": bool(main[k + 1] < 0.0),
            }
            records.append(row)
            run_records.append(row)

        beginning = current[0, 0]
        first_next = current[1, 0]
        initial_end = last_before(wheel, first_next)
        run = {
            "run": number,
            "original_motion_number": motion_number,
            "start_since_bag_s": float(beginning - t0),
            "start_local": datetime.fromtimestamp(beginning, ZoneInfo("Asia/Shanghai")).strftime("%H:%M:%S.%f")[:-3],
            "main_axis": "xy"[axis],
            "direction_sign": sign,
            "first_target_main": float(main[0]),
            "first_interval_ms": float((first_next - beginning) * 1000),
            "wheel_before_second_command": float(sign * wheel[initial_end, 2 + axis]),
            "wheel_onset_002_ms": onset(wheel, beginning, sign, axis, 0.02),
            "wheel_onset_005_ms": onset(wheel, beginning, sign, axis, 0.05),
            "lidar_onset_002_ms": onset(lidar, beginning, sign, axis, 0.02),
            "lidar_onset_005_ms": onset(lidar, beginning, sign, axis, 0.05),
            "peak_command_main": float(max(main)),
            "acceleration_end_s": float(current[acceleration_end, 0] - beginning),
            "deceleration_start_s": float(current[deceleration_start, 0] - beginning),
            "end_s": float(current[-1, 0] - beginning),
            "acceleration": summary([row for row in run_records if row["phase"] == "acceleration"]),
            "deceleration": summary([row for row in run_records if row["phase"] == "deceleration"]),
        }
        runs.append(run)
    summary_all = {phase: summary([row for row in records if row["phase"] == phase]) for phase in ("acceleration", "deceleration")}
    return {"t0_unix_s": t0, "runs": runs, "summary": summary_all, "intervals": records}


def plot_run(data, result, run, output):
    start = result["t0_unix_s"] + run["start_since_bag_s"]
    end = start + run["end_s"] + 0.7
    axis = "xy".index(run["main_axis"])
    sign = run["direction_sign"]
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for topic, label, color in [("/cmd_chassis", "Command (held between updates)", "black"), ("/wheel_velocity", "Wheel feedback", "#1976d2"), ("/OdometryHighFreq", "Lidar feedback", "#e64a19")]:
        value = data[topic]
        selected = value[(value[:, 0] >= start - 0.1) & (value[:, 0] <= end)]
        velocity = sign * selected[:, (1 if topic == "/cmd_chassis" else 2) + axis]
        if topic == "/wheel_velocity":
            velocity = np.where(np.hypot(selected[:, 2], selected[:, 3]) <= 6.0, velocity, np.nan)
        axes[0].step(selected[:, 0] - start, velocity, where="post", label=label, color=color, linewidth=1.0)
    rows = [row for row in result["intervals"] if row["run"] == run["run"]]
    for row in rows:
        time = row["t_start_since_run_s"]
        axes[0].scatter(time + row["dt_s"], row["wheel_end_main"], s=10, color="#1976d2", zorder=3)
    axes[0].set_ylabel("Main-axis body velocity (m/s)")
    axes[0].legend(loc="upper right")
    phases = {"acceleration": "#81c784", "deceleration": "#ffcc80"}
    for phase, color in phases.items():
        selected = [row for row in rows if row["phase"] == phase]
        for row in selected:
            time = row["t_start_since_run_s"]
            axes[1].plot([time, time + row["dt_s"]], [row["command_delta_main"] / row["dt_s"]] * 2, color="black", linewidth=1.0)
            axes[1].plot([time, time + row["dt_s"]], [row["wheel_delta_main"] / row["dt_s"]] * 2, color="#1976d2", linewidth=1.0)
        if selected:
            left = selected[0]["t_start_since_run_s"]
            right = selected[-1]["t_start_since_run_s"] + selected[-1]["dt_s"]
            for ax in axes:
                ax.axvspan(left, right, color=color, alpha=0.18)
    axes[1].set_ylabel("Per-update average rate (m/s^2)")
    axes[1].set_xlabel("Time from first nonzero command (s)")
    axes[1].set_ylim(-30, 30)
    axes[1].set_title("Feedback-rate spikes include staircase measurement updates; they are not calibrated physical acceleration")
    for ax in axes:
        ax.grid(alpha=0.25)
    fig.suptitle(f"Tracking run {run['run']:02d}, {run['start_local']}, body {run['main_axis']} direction {sign:+.0f}")
    fig.tight_layout()
    fig.savefig(output / f"run_{run['run']:02d}_frame_response.png", dpi=160)
    plt.close(fig)


def write_report(result, bag, output):
    runs = result["runs"]
    lines = ["# 相邻控制指令与轮速响应分析", "", f"数据来源：`{bag.name}`，2026-09-29。前4段为手动运动，下面重新编号的1～13次为寻迹运行。", "", "## 统计定义", "", "- 使用 rosbag 在电脑上的消息接收时间，全部数据按真实时间比较，不对轮速或雷达人为平移。", "- `/cmd_chassis` 每10 ms重发；这里以其 vx/vy/vw 数值发生变化的时刻定义一次新指令，更新间隔中位数约50 ms。", "- 每次往返的主运动轴为车体系x；负向往返统一乘以-1，因此下表的加速为正，制动为负。逐帧JSON仍保留原始vx、vy和其差分。", "- 加速段截止于指令首次达到该次主向峰值的95%；减速段从最后达到95%峰值起，截止于首次零速或反向指令。中间为峰值附近段。末尾零速/反向指令的跨零增量计入制动统计，后续反向纠偏不计入。", "- 对区间 `[t_k,t_{k+1})`，目标是该区间实际保持的 `u_k`。比较用的轮速为两端已经收到的最新反馈；下一帧到来之后的样本不用于判断上一帧是否跟上。", "- 相邻指令差 `Δu=u_(k+1)-u_k`；同区间轮速差 `Δv=v(t_(k+1)^-)-v(t_k)`。`Δu/Δt` 是指令序列的等效变化率，并非保持命令区间内的瞬时物理加速度。", "- 段平均变化率采用 `sum(Δv)/sum(Δt)`，不能把阶梯状反馈的逐帧变化率中位数当成底盘加速度。", "- 达标判定：下一帧发出前的主向反馈速度落在上一帧目标的±0.10 m/s内。此项只统计目标≥0.30 m/s的区间，以免把静止误判为跟上很小的起步目标。", "- 起步判定：连续3个反馈样本主向速度>0.02 m/s，记第一个样本的时间；同时计算0.05 m/s阈值以核对敏感性。轮速观测间隔约10 ms，雷达约5 ms。这是反馈首次显示运动的时间，不能直接证明电机何时收到命令或车体何时开始移动。", "- 4个模长超过6 m/s的明显轮速异常样本所涉及区间不参与统计；未对其它原始反馈做滤波。", "", "## 合并统计", "", "| 阶段 | 区间数 | 平均指令变化 m/s/帧 | 平均轮速变化 m/s/帧 | 指令平均变化率 m/s² | 轮速平均变化率 m/s² | 下一帧前达标 |", "|---|---:|---:|---:|---:|---:|---:|"]
    for phase, label in [("acceleration", "加速"), ("deceleration", "减速")]:
        s = result["summary"][phase]
        lines.append(f"| {label} | {s['intervals']} | {s['command_delta_mean']:.3f} | {s['wheel_delta_mean']:.3f} | {s['command_rate']:.2f} | {s['wheel_rate']:.2f} | {s['target_match_count']}/{s['target_match_denominator']}（{s['target_match_percent']:.1f}%） |")
    lines += ["", "这些是所选加减速时间段内的反馈速度变化率；滞后、旧轮速样本、编码器量化和逆解误差均包含在内，不等于底盘最大物理加减速能力。", "", "## 各次运行起步", "", "| 寻迹次数 | 时间 | 方向 | 第一目标 m/s | 下一帧间隔 ms | 下一帧前轮速 m/s | 轮速起步 ms（0.02/0.05阈值） | 雷达起步 ms（0.02/0.05阈值） |", "|---|---|---|---:|---:|---:|---:|---:|"]
    for r in runs:
        lines.append(f"| {r['run']} | {r['start_local']} | {'+' if r['direction_sign'] > 0 else '-'}{r['main_axis']} | {r['first_target_main']:.3f} | {r['first_interval_ms']:.1f} | {r['wheel_before_second_command']:.3f} | {r['wheel_onset_002_ms']:.1f}/{r['wheel_onset_005_ms']:.1f} | {r['lidar_onset_002_ms']:.1f}/{r['lidar_onset_005_ms']:.1f} |")
    lines += ["", "## 各次运行阶段平均速度变化率", "", "| 次数 | 加速指令 | 加速轮速 | 加速差值 | 减速指令 | 减速轮速 | 减速绝对值差 |", "|---|---:|---:|---:|---:|---:|---:|"]
    for r in runs:
        a = r["acceleration"]
        d = r["deceleration"]
        lines.append(f"| {r['run']} | {a['command_rate']:.2f} | {a['wheel_rate']:.2f} | {a['rate_difference']:.2f} | {d['command_rate']:.2f} | {d['wheel_rate']:.2f} | {abs(d['command_rate']) - abs(d['wheel_rate']):.2f} |")
    lines += ["", "单位全部为m/s²；加速差值为指令减反馈，负值表示该段反馈净增速比指令净增速大。", "", "## 逐帧例子：寻迹第13次", "", "下面每行表示新指令发出后，到下一帧新指令发出前的区间。", "", "| 阶段 | 起始s | 间隔ms | 区间目标u_k | 下一目标 | Δu | 轮速开始 | 轮速结束 | Δv | 结束目标误差u_k-v_end |", "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    example = [row for row in result["intervals"] if row["run"] == 13 and row["phase"] != "cruise"]
    for row in example:
        label = "加速" if row["phase"] == "acceleration" else "减速"
        lines.append(f"| {label} | {row['t_start_since_run_s']:.3f} | {row['dt_s'] * 1000:.1f} | {row['target_main']:.3f} | {row['next_target_main']:.3f} | {row['command_delta_main']:+.3f} | {row['wheel_start_main']:.3f} | {row['wheel_end_main']:.3f} | {row['wheel_delta_main']:+.3f} | {row['target_main'] - row['wheel_end_main']:+.3f} |")
    lines += ["", "## 曲线", ""]
    for number in (1, 5, 13):
        lines += [f"![寻迹第{number}次](run_{number:02d}_frame_response.png)", ""]
    lines += ["完整逐帧数据与统计定义保存在 `frame_response.json`；分析脚本为同目录 `analyze_frame_response.py`。", ""]
    (output / "FRAME_RESPONSE_ANALYSIS.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = load_bag(args.bag)
    result = analyze(data)
    (args.output / "frame_response.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    write_report(result, args.bag, args.output)
    for number in (1, 5, 13):
        plot_run(data, result, result["runs"][number - 1], args.output)
    print(json.dumps(result["summary"], ensure_ascii=False, indent=2))
    print(f"Report: {args.output / 'FRAME_RESPONSE_ANALYSIS.md'}")


if __name__ == "__main__":
    main()

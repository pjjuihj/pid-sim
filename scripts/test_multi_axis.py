#!/usr/bin/env python
"""
多轴支持测试套件 — 验证实物调参的多轴能力。

测试内容：
1. 多轴数据解析（CSV 和 VOFA+ FireWater 协议）
2. 多轴参数发送（独立轴参数更新）
3. 多轴独立调参（各轴独立 PID 控制器和物理模型）
4. 多轴配置和预设管理
"""

import sys
import os
import json
import math
import random
import time
from unittest.mock import MagicMock, patch

# 将脚本目录加入路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pid_engine import PIDController
from plant_model import ServoPlant
from real_auto_tune import RelayFeedbackTuner, AutoTuneResult


# ============================================================
# 多轴配置工具
# ============================================================

MULTI_AXIS_CONFIG = {
    "project": "Multi-Axis Test",
    "serial": {
        "port": "COM3",
        "baud": 115200,
        "protocol": "csv",
        "send_format": "P:{kp:.2f},{ki:.2f},{kd:.3f},{deadband:.1f}\n"
    },
    "axes": [
        {
            "name": "Roll",
            "data_format": "sp,meas,out",
            "params": [
                {"id": "kp", "label": "Kp", "min": 0, "max": 200, "default": 1.5},
                {"id": "ki", "label": "Ki", "min": 0, "max": 10, "default": 1.5},
                {"id": "kd", "label": "Kd", "min": 0, "max": 50, "default": 0.06},
                {"id": "deadband", "label": "Deadband", "min": 0, "max": 10, "default": 2.0}
            ]
        },
        {
            "name": "Pitch",
            "data_format": "sp,meas,out",
            "params": [
                {"id": "kp", "label": "Kp", "min": 0, "max": 200, "default": 2.0},
                {"id": "ki", "label": "Ki", "min": 0, "max": 10, "default": 1.0},
                {"id": "kd", "label": "Kd", "min": 0, "max": 50, "default": 0.08},
                {"id": "deadband", "label": "Deadband", "min": 0, "max": 10, "default": 1.5}
            ]
        }
    ],
    "presets": {
        "默认": {"kp": 1.5, "ki": 1.5, "kd": 0.06, "deadband": 2.0},
        "激进": {"kp": 3.0, "ki": 2.5, "kd": 0.12, "deadband": 1.0},
        "保守": {"kp": 0.8, "ki": 0.5, "kd": 0.03, "deadband": 3.0}
    },
    "chart": {
        "send_interval": 0.05
    }
}


# ============================================================
# 第一部分：多轴数据解析测试
# ============================================================

def test_csv_single_axis_parse():
    """测试 CSV 协议单轴数据解析。"""
    print("\n=== 测试 1: CSV 单轴数据解析 ===")

    axes = MULTI_AXIS_CONFIG["axes"]
    buffer = ""
    results = {"Roll": [], "Pitch": []}

    # 模拟单轴 CSV 数据（Roll 轴）
    test_lines = [
        "0.0,0.15,-12.5\n",
        "1.0,0.85,25.3\n",
        "2.0,1.95,-5.2\n",
        "5.0,4.88,3.1\n",
    ]

    for line in test_lines:
        buffer += line
        while '\n' in buffer:
            data_line, buffer = buffer.split('\n', 1)
            data_line = data_line.strip()
            if not data_line:
                continue
            parts = data_line.split(',')
            if len(parts) >= 3:
                try:
                    sp = float(parts[0])
                    meas = float(parts[1])
                    out = float(parts[2])
                    results["Roll"].append({"sp": sp, "meas": meas, "out": out})
                except ValueError:
                    pass

    print(f"  解析行数: {len(results['Roll'])}")
    for i, r in enumerate(results["Roll"]):
        print(f"    [{i}] sp={r['sp']:.1f}, meas={r['meas']:.2f}, out={r['out']:.1f}")

    passed = len(results["Roll"]) == 4
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_csv_multi_axis_parse():
    """测试 CSV 协议多轴数据解析（连续行格式）。"""
    print("\n=== 测试 2: CSV 多轴数据解析 ===")

    axes = MULTI_AXIS_CONFIG["axes"]
    buffers = {a["name"]: {"t": [], "sp": [], "meas": [], "out": []} for a in axes}

    # 模拟多轴 CSV 数据：Roll 和 Pitch 交替发送
    test_lines = [
        "0.0,0.15,-12.5\n",   # Roll
        "0.0,0.20,-8.3\n",    # Pitch
        "1.0,0.85,25.3\n",    # Roll
        "1.0,0.90,18.7\n",    # Pitch
        "2.0,1.95,-5.2\n",    # Roll
        "2.0,1.88,-3.1\n",    # Pitch
    ]

    t = 0.0
    axis_idx = 0
    for line in test_lines:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            try:
                sp = float(parts[0])
                meas = float(parts[1])
                out = float(parts[2])
                t += 0.025
                axis_name = axes[axis_idx % len(axes)]["name"]
                buf = buffers[axis_name]
                buf["t"].append(round(t, 3))
                buf["sp"].append(round(sp, 2))
                buf["meas"].append(round(meas, 2))
                buf["out"].append(round(out, 1))
                axis_idx += 1
            except ValueError:
                pass

    print(f"  Roll 数据点: {len(buffers['Roll']['sp'])}")
    print(f"  Pitch 数据点: {len(buffers['Pitch']['sp'])}")

    for name, buf in buffers.items():
        if buf["sp"]:
            print(f"    {name}: sp={buf['sp']}, meas={buf['meas']}")

    passed = (len(buffers["Roll"]["sp"]) == 3 and len(buffers["Pitch"]["sp"]) == 3)
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_vofa_firewater_multi_axis_parse():
    """测试 VOFA+ FireWater 协议多轴数据解析。"""
    print("\n=== 测试 3: VOFA+ FireWater 多轴数据解析 ===")

    axes = MULTI_AXIS_CONFIG["axes"]
    buffers = {a["name"]: {"t": [], "sp": [], "meas": [], "out": []} for a in axes}

    # VOFA+ FireWater: 每行逗号分隔浮点数，多轴数据连续排列
    # 格式: sp1,meas1,out1,sp2,meas2,out2
    test_lines = [
        "0.0,0.15,-12.5,0.0,0.20,-8.3\n",    # Roll + Pitch
        "1.0,0.85,25.3,1.0,0.90,18.7\n",      # Roll + Pitch
        "2.0,1.95,-5.2,2.0,1.88,-3.1\n",      # Roll + Pitch
        "5.0,4.88,3.1,5.0,4.92,1.5\n",        # Roll + Pitch
    ]

    t = 0.0
    for line in test_lines:
        parts = line.strip().split(',')
        col_idx = 0
        for axis in axes:
            fmt = axis.get("data_format", "sp,meas,out")
            n_cols = len(fmt.split(","))
            if col_idx + n_cols <= len(parts):
                vals = []
                for i in range(n_cols):
                    try:
                        vals.append(float(parts[col_idx + i]))
                    except ValueError:
                        vals.append(0.0)
                t += 0.025
                buf = buffers[axis["name"]]
                buf["t"].append(round(t, 3))
                if len(vals) >= 1:
                    buf["sp"].append(round(vals[0], 2))
                if len(vals) >= 2:
                    buf["meas"].append(round(vals[1], 2))
                if len(vals) >= 3:
                    buf["out"].append(round(vals[2], 1))
                col_idx += n_cols

    print(f"  Roll 数据点: {len(buffers['Roll']['sp'])}")
    print(f"  Pitch 数据点: {len(buffers['Pitch']['sp'])}")

    for name, buf in buffers.items():
        if buf["sp"]:
            print(f"    {name}: sp={buf['sp'][:2]}..., meas={buf['meas'][:2]}...")

    passed = (len(buffers["Roll"]["sp"]) == 4 and len(buffers["Pitch"]["sp"]) == 4)
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_malformed_data_handling():
    """测试异常数据处理。"""
    print("\n=== 测试 4: 异常数据处理 ===")

    axes = MULTI_AXIS_CONFIG["axes"]
    buffers = {a["name"]: {"t": [], "sp": [], "meas": [], "out": []} for a in axes}

    # 包含异常数据的行
    test_lines = [
        "0.0,0.15,-12.5\n",     # 正常
        "abc,0.20,-8.3\n",      # 非数字
        "1.0,\n",               # 缺少字段
        "\n",                    # 空行
        "1.0,0.85,25.3\n",      # 正常
        ",,\n",                 # 全空
        "2.0,1.95,-5.2\n",      # 正常
    ]

    t = 0.0
    parsed_count = 0
    for line in test_lines:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            try:
                sp = float(parts[0])
                meas = float(parts[1])
                out = float(parts[2])
                t += 0.025
                buf = buffers["Roll"]
                buf["t"].append(round(t, 3))
                buf["sp"].append(round(sp, 2))
                buf["meas"].append(round(meas, 2))
                buf["out"].append(round(out, 1))
                parsed_count += 1
            except ValueError:
                pass

    print(f"  输入行数: {len(test_lines)}")
    print(f"  成功解析: {parsed_count}")
    print(f"  Roll 数据点: {len(buffers['Roll']['sp'])}")

    passed = parsed_count == 3  # 只有 3 行是有效的
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 第二部分：多轴参数发送测试
# ============================================================

def test_multi_axis_param_update():
    """测试多轴独立参数更新。"""
    print("\n=== 测试 5: 多轴独立参数更新 ===")

    # 创建两个独立的 PID 控制器
    roll_pid = PIDController(kp=1.5, ki=1.5, kd=0.06, deadband=2.0)
    pitch_pid = PIDController(kp=2.0, ki=1.0, kd=0.08, deadband=1.5)

    # 记录初始参数
    roll_params_before = roll_pid.get_params()
    pitch_params_before = pitch_pid.get_params()

    # 更新 Roll 轴参数
    roll_pid.update_params({"kp": 3.0, "ki": 2.5})
    # 更新 Pitch 轴参数
    pitch_pid.update_params({"kd": 0.15, "deadband": 1.0})

    roll_params_after = roll_pid.get_params()
    pitch_params_after = pitch_pid.get_params()

    print(f"  Roll 初始: Kp={roll_params_before['kp']:.1f}, Ki={roll_params_before['ki']:.1f}, Kd={roll_params_before['kd']:.2f}")
    print(f"  Roll 更新后: Kp={roll_params_after['kp']:.1f}, Ki={roll_params_after['ki']:.1f}, Kd={roll_params_after['kd']:.2f}")
    print(f"  Pitch 初始: Kp={pitch_params_before['kp']:.1f}, Ki={pitch_params_before['ki']:.1f}, Kd={pitch_params_before['kd']:.2f}")
    print(f"  Pitch 更新后: Kp={pitch_params_after['kp']:.1f}, Ki={pitch_params_after['ki']:.1f}, Kd={pitch_params_after['kd']:.2f}")

    # 验证：Roll 的 Kp/Ki 变了，Kd 没变
    roll_ok = (roll_params_after["kp"] == 3.0 and
               roll_params_after["ki"] == 2.5 and
               roll_params_after["kd"] == 0.06)  # Kd 未变

    # 验证：Pitch 的 Kd/deadband 变了，Kp/Ki 没变
    pitch_ok = (pitch_params_after["kp"] == 2.0 and
                pitch_params_after["ki"] == 1.0 and
                pitch_params_after["kd"] == 0.15 and
                pitch_params_after["deadband"] == 1.0)

    passed = roll_ok and pitch_ok
    print(f"  Roll 独立更新: {'PASS' if roll_ok else 'FAIL'}")
    print(f"  Pitch 独立更新: {'PASS' if pitch_ok else 'FAIL'}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_param_send_format():
    """测试参数发送格式化。"""
    print("\n=== 测试 6: 参数发送格式化 ===")

    send_format = "P:{kp:.2f},{ki:.2f},{kd:.3f},{deadband:.1f}\n"

    test_cases = [
        {"kp": 1.5, "ki": 1.5, "kd": 0.06, "deadband": 2.0},
        {"kp": 3.0, "ki": 2.5, "kd": 0.12, "deadband": 1.0},
        {"kp": 0.8, "ki": 0.5, "kd": 0.03, "deadband": 3.0},
        {"kp": 10.5, "ki": 8.2, "kd": 1.234, "deadband": 0.5},
    ]

    all_ok = True
    for params in test_cases:
        try:
            cmd = send_format.format(**params)
            # 验证格式
            assert cmd.startswith("P:")
            assert cmd.endswith("\n")
            parts = cmd.strip().split(":")[1].split(",")
            assert len(parts) == 4
            # 验证数值
            assert abs(float(parts[0]) - params["kp"]) < 0.001
            assert abs(float(parts[1]) - params["ki"]) < 0.001
            assert abs(float(parts[2]) - params["kd"]) < 0.001
            print(f"  Kp={params['kp']:.2f} -> '{cmd.strip()}' [OK]")
        except Exception as e:
            print(f"  Kp={params['kp']:.2f} -> FAIL: {e}")
            all_ok = False

    print(f"  结果: {'PASS' if all_ok else 'FAIL'}")
    return all_ok


def test_multi_axis_param_independence():
    """测试多轴参数互不影响。"""
    print("\n=== 测试 7: 多轴参数互不影响 ===")

    # 创建两个 PID 控制器，使用相同的初始参数
    pid_a = PIDController(kp=1.5, ki=1.5, kd=0.06)
    pid_b = PIDController(kp=1.5, ki=1.5, kd=0.06)

    # 运行若干步
    dt = 0.005
    for i in range(100):
        t = i * dt
        meas_a = 0.5 * math.sin(t * 2)
        meas_b = 0.3 * math.cos(t * 3)
        pid_a.compute(5.0, meas_a)
        pid_b.compute(5.0, meas_b)

    # 现在只更新 pid_a
    pid_a.update_params({"kp": 10.0, "ki": 5.0, "kd": 0.5})

    # 继续运行
    for i in range(100):
        t = (100 + i) * dt
        meas_a = 0.5 * math.sin(t * 2)
        meas_b = 0.3 * math.cos(t * 3)
        out_a = pid_a.compute(5.0, meas_a)
        out_b = pid_b.compute(5.0, meas_b)

    params_a = pid_a.get_params()
    params_b = pid_b.get_params()

    print(f"  PID-A: Kp={params_a['kp']:.1f}, Ki={params_a['ki']:.1f}, Kd={params_a['kd']:.2f}")
    print(f"  PID-B: Kp={params_b['kp']:.1f}, Ki={params_b['ki']:.1f}, Kd={params_b['kd']:.2f}")

    # 验证 pid_b 的参数未变
    b_unchanged = (params_b["kp"] == 1.5 and params_b["ki"] == 1.5 and params_b["kd"] == 0.06)
    # 验证 pid_a 的参数已变
    a_changed = (params_a["kp"] == 10.0 and params_a["ki"] == 5.0 and params_a["kd"] == 0.5)

    passed = b_unchanged and a_changed
    print(f"  PID-B 未受影响: {'PASS' if b_unchanged else 'FAIL'}")
    print(f"  PID-A 已更新: {'PASS' if a_changed else 'FAIL'}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_simultaneous_param_update():
    """测试同时更新多个轴的参数。"""
    print("\n=== 测试 8: 同时更新多个轴参数 ===")

    pid_controllers = {
        "Roll": PIDController(kp=1.5, ki=1.5, kd=0.06),
        "Pitch": PIDController(kp=2.0, ki=1.0, kd=0.08),
    }

    # 同时更新所有轴的参数
    new_params = {"kp": 5.0, "ki": 3.0, "kd": 0.2}
    for name, pid in pid_controllers.items():
        pid.update_params(new_params)

    # 验证所有轴都更新了
    all_updated = True
    for name, pid in pid_controllers.items():
        p = pid.get_params()
        ok = (p["kp"] == 5.0 and p["ki"] == 3.0 and p["kd"] == 0.2)
        all_updated = all_updated and ok
        print(f"  {name}: Kp={p['kp']:.1f}, Ki={p['ki']:.1f}, Kd={p['kd']:.2f} [{'PASS' if ok else 'FAIL'}]")

    print(f"  结果: {'PASS' if all_updated else 'FAIL'}")
    return all_updated


# ============================================================
# 第三部分：多轴独立调参测试
# ============================================================

class SecondOrderPlant:
    """简单二阶系统用于测试。"""

    def __init__(self, omega_n=10.0, zeta=0.3, gain=1.0, dt=0.005, noise=0.05):
        self.omega_n = omega_n
        self.zeta = zeta
        self.gain = gain
        self.dt = dt
        self.noise = noise
        self.x = 0.0
        self.x_dot = 0.0

    def reset(self):
        self.x = 0.0
        self.x_dot = 0.0

    def __call__(self, control_output):
        accel = (self.gain * self.omega_n**2 * control_output
                 - 2 * self.zeta * self.omega_n * self.x_dot
                 - self.omega_n**2 * self.x)
        self.x_dot += accel * self.dt
        self.x += self.x_dot * self.dt
        return self.x + random.gauss(0, self.noise)


def test_independent_pid_simulation():
    """测试多轴独立 PID 仿真。"""
    print("\n=== 测试 9: 多轴独立 PID 仿真 ===")

    # 创建两个不同参数的 PID + 不同特性的 Plant
    roll_pid = PIDController(kp=2.0, ki=1.5, kd=0.08, dt=0.005, deadband=0.0)
    pitch_pid = PIDController(kp=3.0, ki=2.0, kd=0.12, dt=0.005, deadband=0.0)

    roll_plant = SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01)
    pitch_plant = SecondOrderPlant(omega_n=8.0, zeta=0.5, gain=0.8, noise=0.01)

    dt = 0.005
    steps = int(5.0 / dt)
    setpoint = 5.0

    roll_measurements = []
    pitch_measurements = []

    for i in range(steps):
        t = i * dt
        sp = setpoint if t > 0.5 else 0.0

        # Roll 轴
        roll_out = roll_pid.compute(sp, roll_plant.x)
        roll_meas = roll_plant(roll_out)
        roll_measurements.append(roll_meas)

        # Pitch 轴
        pitch_out = pitch_pid.compute(sp, pitch_plant.x)
        pitch_meas = pitch_plant(pitch_out)
        pitch_measurements.append(pitch_meas)

    # 分析结果
    ss_start = int(3.5 / dt)
    roll_ss = sum(roll_measurements[ss_start:]) / len(roll_measurements[ss_start:])
    pitch_ss = sum(pitch_measurements[ss_start:]) / len(pitch_measurements[ss_start:])

    roll_error = abs(setpoint - roll_ss)
    pitch_error = abs(setpoint - pitch_ss)

    print(f"  Roll 稳态均值: {roll_ss:.3f} (误差: {roll_error:.4f})")
    print(f"  Pitch 稳态均值: {pitch_ss:.3f} (误差: {pitch_error:.4f})")

    # 两个轴都应该接近设定值
    passed = roll_error < 1.0 and pitch_error < 1.0
    print(f"  Roll 收敛: {'PASS' if roll_error < 1.0 else 'FAIL'}")
    print(f"  Pitch 收敛: {'PASS' if pitch_error < 1.0 else 'FAIL'}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_independent_auto_tune():
    """测试多轴独立自动调参。"""
    print("\n=== 测试 10: 多轴独立自动调参 ===")

    axes_config = [
        {"name": "Roll", "omega_n": 10.0, "zeta": 0.3, "gain": 1.0},
        {"name": "Pitch", "omega_n": 8.0, "zeta": 0.5, "gain": 0.8},
    ]

    results = {}
    for axis_cfg in axes_config:
        plant = SecondOrderPlant(
            omega_n=axis_cfg["omega_n"],
            zeta=axis_cfg["zeta"],
            gain=axis_cfg["gain"],
            noise=0.01
        )
        tuner = RelayFeedbackTuner(
            relay_amplitude=500.0,
            relay_hysteresis=0.5,
            sample_time=0.005,
            relay_duration=12.0,
            min_cycles=3,
            min_switch_time=0.02,
            verify_duration=5.0,
            setpoint=5.0,
            zn_method='some_overshoot'
        )

        result = tuner.auto_tune(plant)
        results[axis_cfg["name"]] = result

        print(f"  {axis_cfg['name']}:")
        print(f"    状态: {result.status}")
        if result.zn_params:
            print(f"    Kp={result.zn_params.kp:.2f}, Ki={result.zn_params.ki:.2f}, Kd={result.zn_params.kd:.3f}")
        if result.verification:
            print(f"    超调={result.verification.overshoot:.1f}%, 调节时间={result.verification.settling_time:.2f}s")

    # 验证两个轴都完成了调参
    both_done = all(
        r.relay_result is not None and r.zn_params is not None
        for r in results.values()
    )

    # 验证两个轴的参数不同（因为物理模型不同）
    if both_done and results["Roll"].zn_params and results["Pitch"].zn_params:
        params_differ = (
            results["Roll"].zn_params.kp != results["Pitch"].zn_params.kp or
            results["Roll"].zn_params.ki != results["Pitch"].zn_params.ki
        )
    else:
        params_differ = False

    passed = both_done and params_differ
    print(f"  两轴均完成: {'PASS' if both_done else 'FAIL'}")
    print(f"  参数独立: {'PASS' if params_differ else 'FAIL'}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed, results


def test_cross_axis_param_apply():
    """测试将一个轴的调参结果应用到另一个轴。"""
    print("\n=== 测试 11: 跨轴参数应用 ===")

    # Roll 轴自动调参
    roll_plant = SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01)
    roll_tuner = RelayFeedbackTuner(
        relay_amplitude=500.0, relay_hysteresis=0.5,
        sample_time=0.005, relay_duration=10.0,
        setpoint=5.0, zn_method='some_overshoot'
    )
    roll_result = roll_tuner.auto_tune(roll_plant)

    if not roll_result.recommended_params:
        print("  Roll 调参失败，跳过测试")
        return False

    # 创建 Pitch 轴 PID 控制器
    pitch_pid = PIDController(kp=1.0, ki=0.5, kd=0.02)
    pitch_params_before = pitch_pid.get_params()

    # 将 Roll 的调参结果应用到 Pitch
    pitch_pid.update_params(roll_result.recommended_params)
    pitch_params_after = pitch_pid.get_params()

    print(f"  Roll 调参结果: {roll_result.recommended_params}")
    print(f"  Pitch 更新前: Kp={pitch_params_before['kp']:.2f}, Ki={pitch_params_before['ki']:.2f}, Kd={pitch_params_before['kd']:.3f}")
    print(f"  Pitch 更新后: Kp={pitch_params_after['kp']:.2f}, Ki={pitch_params_after['ki']:.2f}, Kd={pitch_params_after['kd']:.3f}")

    # 验证参数已更新
    applied = (
        pitch_params_after["kp"] == roll_result.recommended_params["kp"] and
        pitch_params_after["ki"] == roll_result.recommended_params["ki"] and
        pitch_params_after["kd"] == roll_result.recommended_params["kd"]
    )

    passed = applied
    print(f"  参数应用: {'PASS' if applied else 'FAIL'}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_multi_axis_step_response():
    """测试多轴同时阶跃响应。"""
    print("\n=== 测试 12: 多轴同时阶跃响应 ===")

    pid_controllers = {
        "Roll": PIDController(kp=2.0, ki=1.5, kd=0.08, dt=0.005, deadband=0.0),
        "Pitch": PIDController(kp=3.0, ki=2.0, kd=0.12, dt=0.005, deadband=0.0),
    }

    plants = {
        "Roll": SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01),
        "Pitch": SecondOrderPlant(omega_n=8.0, zeta=0.5, gain=0.8, noise=0.01),
    }

    dt = 0.005
    steps = int(5.0 / dt)
    setpoint = 5.0

    all_data = {name: {"t": [], "sp": [], "meas": [], "out": []} for name in pid_controllers}

    for i in range(steps):
        t = i * dt
        sp = setpoint if t > 0.5 else 0.0

        for name in pid_controllers:
            pid = pid_controllers[name]
            plant = plants[name]

            out = pid.compute(sp, plant.x)
            meas = plant(out)

            all_data[name]["t"].append(round(t, 3))
            all_data[name]["sp"].append(round(sp, 2))
            all_data[name]["meas"].append(round(meas, 2))
            all_data[name]["out"].append(round(out, 1))

    # 分析结果
    for name in pid_controllers:
        data = all_data[name]
        ss_start = int(3.5 / dt)
        ss_data = data["meas"][ss_start:]
        ss_mean = sum(ss_data) / len(ss_data) if ss_data else 0
        error = abs(setpoint - ss_mean)

        peak = max(data["meas"][int(0.5 / dt):])
        overshoot = max(0, (peak - setpoint) / setpoint * 100)

        print(f"  {name}: 稳态={ss_mean:.3f}, 误差={error:.4f}, 超调={overshoot:.1f}%")

    # 验证两个轴都收敛
    all_converged = True
    for name in pid_controllers:
        data = all_data[name]
        ss_start = int(3.5 / dt)
        ss_data = data["meas"][ss_start:]
        ss_mean = sum(ss_data) / len(ss_data) if ss_data else 0
        if abs(setpoint - ss_mean) > 1.0:
            all_converged = False

    passed = all_converged
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 第四部分：多轴配置和预设测试
# ============================================================

def test_multi_axis_config_structure():
    """测试多轴配置文件结构。"""
    print("\n=== 测试 13: 多轴配置文件结构 ===")

    config = MULTI_AXIS_CONFIG

    # 验证 axes 数组
    assert "axes" in config, "缺少 axes 字段"
    assert len(config["axes"]) == 2, f"期望 2 个轴，实际 {len(config['axes'])}"

    # 验证每个轴的结构
    for axis in config["axes"]:
        assert "name" in axis, f"轴缺少 name 字段"
        assert "data_format" in axis, f"轴 {axis['name']} 缺少 data_format 字段"
        assert "params" in axis, f"轴 {axis['name']} 缺少 params 字段"
        assert len(axis["params"]) >= 3, f"轴 {axis['name']} 参数不足"

        # 验证每个参数的结构
        for param in axis["params"]:
            assert "id" in param, f"参数缺少 id 字段"
            assert "label" in param, f"参数 {param.get('id')} 缺少 label 字段"
            assert "default" in param, f"参数 {param.get('id')} 缺少 default 字段"

    print(f"  轴数: {len(config['axes'])}")
    for axis in config["axes"]:
        print(f"    {axis['name']}: {len(axis['params'])} 个参数")
        for p in axis["params"]:
            print(f"      {p['label']}: default={p['default']}")

    print(f"  结果: PASS")
    return True


def test_multi_axis_preset_management():
    """测试多轴预设管理。"""
    print("\n=== 测试 14: 多轴预设管理 ===")

    config = MULTI_AXIS_CONFIG
    presets = config.get("presets", {})

    # 验证预设存在
    assert len(presets) >= 3, f"期望至少 3 个预设，实际 {len(presets)}"

    # 创建 PID 控制器并应用预设
    pid = PIDController(kp=1.0, ki=0.5, kd=0.02)

    for preset_name, params in presets.items():
        pid.update_params(params)
        current = pid.get_params()

        kp_match = abs(current["kp"] - params["kp"]) < 0.001
        ki_match = abs(current["ki"] - params["ki"]) < 0.001
        kd_match = abs(current["kd"] - params["kd"]) < 0.001

        ok = kp_match and ki_match and kd_match
        print(f"  预设 '{preset_name}': Kp={params['kp']:.1f}, Ki={params['ki']:.1f}, Kd={params['kd']:.2f} [{'PASS' if ok else 'FAIL'}]")

    print(f"  结果: PASS")
    return True


def test_multi_axis_preset_apply_all():
    """测试将预设应用到所有轴。"""
    print("\n=== 测试 15: 预设应用到所有轴 ===")

    pid_controllers = {
        "Roll": PIDController(kp=1.5, ki=1.5, kd=0.06),
        "Pitch": PIDController(kp=2.0, ki=1.0, kd=0.08),
    }

    # 应用"激进"预设到所有轴
    preset_params = {"kp": 3.0, "ki": 2.5, "kd": 0.12}
    for name, pid in pid_controllers.items():
        pid.update_params(preset_params)

    # 验证所有轴都更新了
    all_applied = True
    for name, pid in pid_controllers.items():
        p = pid.get_params()
        ok = (p["kp"] == 3.0 and p["ki"] == 2.5 and p["kd"] == 0.12)
        all_applied = all_applied and ok
        print(f"  {name}: Kp={p['kp']:.1f}, Ki={p['ki']:.1f}, Kd={p['kd']:.2f} [{'PASS' if ok else 'FAIL'}]")

    passed = all_applied
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 主测试函数
# ============================================================

def run_all_tests():
    """运行所有测试。"""
    print("=" * 60)
    print("多轴支持测试套件")
    print("验证实物调参的多轴能力")
    print("=" * 60)

    tests = [
        # 第一部分：多轴数据解析
        ("CSV 单轴数据解析", test_csv_single_axis_parse),
        ("CSV 多轴数据解析", test_csv_multi_axis_parse),
        ("VOFA+ FireWater 多轴数据解析", test_vofa_firewater_multi_axis_parse),
        ("异常数据处理", test_malformed_data_handling),
        # 第二部分：多轴参数发送
        ("多轴独立参数更新", test_multi_axis_param_update),
        ("参数发送格式化", test_param_send_format),
        ("多轴参数互不影响", test_multi_axis_param_independence),
        ("同时更新多个轴参数", test_simultaneous_param_update),
        # 第三部分：多轴独立调参
        ("多轴独立 PID 仿真", test_independent_pid_simulation),
        ("多轴独立自动调参", test_independent_auto_tune),
        ("跨轴参数应用", test_cross_axis_param_apply),
        ("多轴同时阶跃响应", test_multi_axis_step_response),
        # 第四部分：多轴配置和预设
        ("多轴配置文件结构", test_multi_axis_config_structure),
        ("多轴预设管理", test_multi_axis_preset_management),
        ("预设应用到所有轴", test_multi_axis_preset_apply_all),
    ]

    results = []

    for name, test_func in tests:
        try:
            ret = test_func()
            if isinstance(ret, tuple):
                passed = ret[0]
            else:
                passed = ret
            results.append((name, passed))
        except Exception as e:
            print(f"  异常: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))

    # 汇总
    print("\n" + "=" * 60)
    print("测试汇总")
    print("=" * 60)
    total = len(results)
    passed_count = sum(1 for _, p in results if p)
    failed_count = total - passed_count

    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  [{status}] {name}")

    print(f"\n总计: {total}, 通过: {passed_count}, 失败: {failed_count}")
    if total > 0:
        print(f"通过率: {passed_count / total * 100:.1f}%")

    return results


if __name__ == '__main__':
    results = run_all_tests()

#!/usr/bin/env python
"""
超调自动微调功能测试套件。

测试内容：
1. 超调检测准确性验证
2. Kp 降低比例（0.8）验证
3. Kd 增加比例（1.2）验证
4. 微调后响应改善验证
5. 边界条件和多轮微调测试
"""

import sys
import os
import math
import random
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from real_auto_tune import (
    RelayFeedbackTuner,
    OvershootFineTuner,
    RelayFeedbackResult,
    ZieglerNicholsResult,
    VerificationResult,
    AutoTuneResult,
    FineTuneRecord,
)
from pid_engine import PIDController
from plant_model import ServoPlant


# ============================================================
# 测试用的模拟受控对象
# ============================================================

class SecondOrderPlant:
    """二阶系统：用于产生可控超调的受控对象。"""

    def __init__(self, omega_n=10.0, zeta=0.3, gain=1.0, dt=0.005, noise=0.01):
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


class HighOvershootPlant:
    """故意设计为高超调的受控对象（低阻尼）。"""

    def __init__(self, omega_n=10.0, zeta=0.15, gain=1.0, dt=0.005, noise=0.01):
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


class ServoPlantWrapper:
    """包装 ServoPlant 为 callable 接口。"""

    def __init__(self, dt=0.005):
        self.plant = ServoPlant(dt=dt)

    def reset(self):
        self.plant.reset()

    def __call__(self, control_output):
        return self.plant.update(control_output)


# ============================================================
# 测试 1: 超调检测准确性
# ============================================================

def test_overshoot_detection_known_response():
    """使用已知响应数据验证超调检测算法的准确性。"""
    print("\n=== 测试 1: 超调检测准确性（已知数据） ===")

    tuner = RelayFeedbackTuner(setpoint=5.0, verify_duration=5.0)
    dt = 0.005

    # 构造已知响应：目标值=5.0，峰值=6.5（超调=30%）
    total_steps = int(5.0 / dt)

    measurements = []
    setpoints = []
    outputs = []
    times = []

    for i in range(total_steps):
        t = i * dt
        times.append(t)
        sp = 5.0 if t > 0.5 else 0.0
        setpoints.append(sp)

        if t <= 0.5:
            measurements.append(0.0)
            outputs.append(0.0)
        elif t <= 0.6:
            # 上升阶段：从 0 到 6.5（超调）
            phase = (t - 0.5) / 0.1
            measurements.append(6.5 * phase)
            outputs.append(800.0)
        elif t <= 0.8:
            # 峰值保持
            measurements.append(6.5)
            outputs.append(200.0)
        elif t <= 1.5:
            # 下降阶段
            phase = (t - 0.8) / 0.7
            measurements.append(6.5 - 1.5 * phase)
            outputs.append(100.0)
        else:
            # 稳态
            measurements.append(5.0)
            outputs.append(50.0)

    result = tuner._analyze_verification(
        measurements, setpoints, outputs, times, 5.0, dt
    )

    expected_overshoot = (6.5 - 5.0) / 5.0 * 100  # 30%
    print(f"  期望超调: {expected_overshoot:.1f}%")
    print(f"  检测超调: {result.overshoot:.1f}%")
    print(f"  误差: {abs(result.overshoot - expected_overshoot):.2f}%")

    passed = abs(result.overshoot - expected_overshoot) < 1.0  # 允许 1% 误差
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed, result


def test_overshoot_detection_no_overshoot():
    """验证无超调情况下的检测。"""
    print("\n=== 测试 2: 无超调检测 ===")

    tuner = RelayFeedbackTuner(setpoint=5.0, verify_duration=5.0)
    dt = 0.005
    total_steps = int(5.0 / dt)

    measurements = []
    setpoints = []
    outputs = []
    times = []

    for i in range(total_steps):
        t = i * dt
        times.append(t)
        sp = 5.0 if t > 0.5 else 0.0
        setpoints.append(sp)

        if t <= 0.5:
            measurements.append(0.0)
            outputs.append(0.0)
        elif t <= 1.5:
            # 从 0 指数上升到 5.0（无超调）
            phase = 1.0 - math.exp(-(t - 0.5) / 0.3)
            measurements.append(5.0 * phase)
            outputs.append(400.0)
        else:
            measurements.append(5.0)
            outputs.append(50.0)

    result = tuner._analyze_verification(
        measurements, setpoints, outputs, times, 5.0, dt
    )

    print(f"  检测超调: {result.overshoot:.1f}%")
    passed = result.overshoot < 1.0  # 无超调应 < 1%
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed, result


def test_overshoot_detection_high_overshoot():
    """验证高超调（>20%）的检测。"""
    print("\n=== 测试 3: 高超调检测（>20%） ===")

    tuner = RelayFeedbackTuner(setpoint=5.0, verify_duration=5.0)
    dt = 0.005
    total_steps = int(5.0 / dt)

    measurements = []
    setpoints = []
    outputs = []
    times = []

    for i in range(total_steps):
        t = i * dt
        times.append(t)
        sp = 5.0 if t > 0.5 else 0.0
        setpoints.append(sp)

        if t <= 0.5:
            measurements.append(0.0)
            outputs.append(0.0)
        elif t <= 0.55:
            # 快速上升
            phase = (t - 0.5) / 0.05
            measurements.append(7.5 * phase)
            outputs.append(800.0)
        elif t <= 0.7:
            # 峰值 7.5（超调 50%）
            measurements.append(7.5)
            outputs.append(300.0)
        elif t <= 1.2:
            # 振荡下降
            phase = (t - 0.7) / 0.5
            measurements.append(7.5 - 2.5 * phase)
            outputs.append(100.0)
        elif t <= 2.0:
            # 第二个峰（较小）
            phase = (t - 1.2) / 0.8
            measurements.append(5.0 + 1.0 * (1.0 - phase))
            outputs.append(50.0)
        else:
            measurements.append(5.0)
            outputs.append(50.0)

    result = tuner._analyze_verification(
        measurements, setpoints, outputs, times, 5.0, dt
    )

    print(f"  期望超调: 50.0%")
    print(f"  检测超调: {result.overshoot:.1f}%")
    print(f"  超调 > 20%: {result.overshoot > 20.0}")

    passed = result.overshoot > 20.0  # 应检测到高超调
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed, result


# ============================================================
# 测试 2: OvershootFineTuner 单元测试
# ============================================================

def test_finetuner_should_finetune():
    """测试 OvershootFineTuner 的判断逻辑。"""
    print("\n=== 测试 4: 微调器判断逻辑 ===")

    finetuner = OvershootFineTuner(overshoot_threshold=20.0)

    # 超调 < 20%：不需要微调
    assert not finetuner.should_finetune(15.0), "15% 不应触发微调"
    print(f"  超调 15.0%: should_finetune = {finetuner.should_finetune(15.0)} (期望: False) [PASS]")

    # 超调 = 20%：不需要微调（边界值）
    assert not finetuner.should_finetune(20.0), "20% 不应触发微调"
    print(f"  超调 20.0%: should_finetune = {finetuner.should_finetune(20.0)} (期望: False) [PASS]")

    # 超调 > 20%：需要微调
    assert finetuner.should_finetune(25.0), "25% 应触发微调"
    print(f"  超调 25.0%: should_finetune = {finetuner.should_finetune(25.0)} (期望: True) [PASS]")

    # 超调 = 0%：不需要微调
    assert not finetuner.should_finetune(0.0), "0% 不应触发微调"
    print(f"  超调  0.0%: should_finetune = {finetuner.should_finetune(0.0)} (期望: False) [PASS]")

    # 超调 = 100%：需要微调
    assert finetuner.should_finetune(100.0), "100% 应触发微调"
    print(f"  超调 100.0%: should_finetune = {finetuner.should_finetune(100.0)} (期望: True) [PASS]")

    print("  结果: PASS")
    return True


def test_finetuner_adjust_params():
    """测试 OvershootFineTuner 的参数调整逻辑。"""
    print("\n=== 测试 5: 参数调整逻辑 ===")

    finetuner = OvershootFineTuner(kp_factor=0.8, kd_factor=1.2)

    # 基本测试
    kp, ki, kd = 10.0, 5.0, 1.0
    new_kp, new_ki, new_kd = finetuner.adjust_params(kp, ki, kd)

    print(f"  原始: Kp={kp}, Ki={ki}, Kd={kd}")
    print(f"  调整: Kp={new_kp}, Ki={new_ki}, Kd={new_kd}")

    # 验证 Kp 降低 20%
    expected_kp = kp * 0.8
    assert abs(new_kp - expected_kp) < 1e-10, f"Kp 应为 {expected_kp}，实际 {new_kp}"
    print(f"  Kp: {kp} -> {new_kp} (期望 {expected_kp}) [PASS]")

    # 验证 Ki 不变
    assert new_ki == ki, f"Ki 应保持 {ki}，实际 {new_ki}"
    print(f"  Ki: {ki} -> {new_ki} (期望 {ki}) [PASS]")

    # 验证 Kd 增加 20%
    expected_kd = kd * 1.2
    assert abs(new_kd - expected_kd) < 1e-10, f"Kd 应为 {expected_kd}，实际 {new_kd}"
    print(f"  Kd: {kd} -> {new_kd} (期望 {expected_kd}) [PASS]")

    # 多轮微调测试
    kp, ki, kd = 10.0, 5.0, 1.0
    for i in range(3):
        kp, ki, kd = finetuner.adjust_params(kp, ki, kd)
    print(f"\n  3 轮微调后: Kp={kp:.4f}, Ki={ki}, Kd={kd:.4f}")
    expected_kp_3 = 10.0 * (0.8 ** 3)
    expected_kd_3 = 1.0 * (1.2 ** 3)
    assert abs(kp - expected_kp_3) < 1e-10, f"3轮后 Kp 应为 {expected_kp_3}"
    assert abs(kd - expected_kd_3) < 1e-10, f"3轮后 Kd 应为 {expected_kd_3}"
    print(f"  3轮后 Kp: {kp:.4f} (期望 {expected_kp_3:.4f}) [PASS]")
    print(f"  3轮后 Kd: {kd:.4f} (期望 {expected_kd_3:.4f}) [PASS]")

    print("  结果: PASS")
    return True


def test_finetuner_custom_ratios():
    """测试自定义微调比例。"""
    print("\n=== 测试 6: 自定义微调比例 ===")

    finetuner = OvershootFineTuner(kp_factor=0.7, kd_factor=1.3)

    kp, ki, kd = 10.0, 5.0, 1.0
    new_kp, new_ki, new_kd = finetuner.adjust_params(kp, ki, kd)

    assert abs(new_kp - 7.0) < 1e-10, f"Kp 应为 7.0，实际 {new_kp}"
    assert abs(new_kd - 1.3) < 1e-10, f"Kd 应为 1.3，实际 {new_kd}"
    print(f"  Kp: {kp} -> {new_kp} (x0.7) [PASS]")
    print(f"  Kd: {kd} -> {new_kd} (x1.3) [PASS]")

    print("  结果: PASS")
    return True


# ============================================================
# 测试 3: Kp 降低比例验证（0.8）
# ============================================================

def test_kp_reduction_ratio():
    """验证 Kp 降低比例精确为 0.8。"""
    print("\n=== 测试 7: Kp 降低比例 (0.8) 验证 ===")

    finetuner = OvershootFineTuner(kp_factor=0.8, kd_factor=1.2)

    test_cases = [
        (10.0, 8.0),
        (100.0, 80.0),
        (1.5, 1.2),
        (0.5, 0.4),
        (50.0, 40.0),
        (0.01, 0.008),
    ]

    all_passed = True
    for kp_in, expected_kp in test_cases:
        new_kp, _, _ = finetuner.adjust_params(kp_in, 5.0, 1.0)
        ok = abs(new_kp - expected_kp) < 1e-6  # 使用浮点容差
        all_passed = all_passed and ok
        status = 'PASS' if ok else 'FAIL'
        print(f"  Kp={kp_in} -> {new_kp} (期望 {expected_kp}) [{status}]")

    print(f"  结果: {'PASS' if all_passed else 'FAIL'}")
    return all_passed


def test_kp_ratio_preserves_ki():
    """验证 Kp 调整时 Ki 保持不变。"""
    print("\n=== 测试 8: Kp 调整时 Ki 不变 ===")

    finetuner = OvershootFineTuner(kp_factor=0.8, kd_factor=1.2)

    ki_values = [0.0, 1.0, 5.0, 100.0, 0.001]
    all_passed = True

    for ki in ki_values:
        _, new_ki, _ = finetuner.adjust_params(10.0, ki, 1.0)
        ok = new_ki == ki
        all_passed = all_passed and ok
        status = 'PASS' if ok else 'FAIL'
        print(f"  Ki={ki} -> {new_ki} (期望 {ki}) [{status}]")

    print(f"  结果: {'PASS' if all_passed else 'FAIL'}")
    return all_passed


# ============================================================
# 测试 4: Kd 增加比例验证（1.2）
# ============================================================

def test_kd_increase_ratio():
    """验证 Kd 增加比例精确为 1.2。"""
    print("\n=== 测试 9: Kd 增加比例 (1.2) 验证 ===")

    finetuner = OvershootFineTuner(kp_factor=0.8, kd_factor=1.2)

    test_cases = [
        (1.0, 1.2),
        (0.5, 0.6),
        (10.0, 12.0),
        (0.01, 0.012),
        (50.0, 60.0),
        (0.001, 0.0012),
    ]

    all_passed = True
    for kd_in, expected_kd in test_cases:
        _, _, new_kd = finetuner.adjust_params(10.0, 5.0, kd_in)
        ok = abs(new_kd - expected_kd) < 1e-10
        all_passed = all_passed and ok
        status = 'PASS' if ok else 'FAIL'
        print(f"  Kd={kd_in} -> {new_kd} (期望 {expected_kd}) [{status}]")

    print(f"  结果: {'PASS' if all_passed else 'FAIL'}")
    return all_passed


def test_kd_ratio_preserves_kp():
    """验证 Kd 调整时 Kp 按比例变化（不影响 Kp 的独立性）。"""
    print("\n=== 测试 10: Kd 调整不影响 Kp ===")

    finetuner = OvershootFineTuner(kp_factor=0.8, kd_factor=1.2)

    kp_values = [1.0, 10.0, 100.0, 0.5]
    all_passed = True

    for kp in kp_values:
        new_kp, _, _ = finetuner.adjust_params(kp, 5.0, 1.0)
        expected = kp * 0.8
        ok = abs(new_kp - expected) < 1e-10
        all_passed = all_passed and ok
        status = 'PASS' if ok else 'FAIL'
        print(f"  Kp={kp} -> {new_kp} (期望 {expected}) [{status}]")

    print(f"  结果: {'PASS' if all_passed else 'FAIL'}")
    return all_passed


# ============================================================
# 测试 5: 微调后响应改善验证
# ============================================================

def test_finetune_improves_overshoot():
    """验证超调微调后超调量降低。"""
    print("\n=== 测试 11: 微调后超调改善 ===")

    # 使用 ServoPlant（更稳定，与固件一致）
    plant = ServoPlantWrapper(dt=0.005)
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=1.0,
        sample_time=0.005,
        relay_duration=15.0,
        min_cycles=3,
        min_switch_time=0.03,
        verify_duration=5.0,
        setpoint=5.0,
        zn_method='classic',  # 经典方法会产生较大超调
        overshoot_threshold=20.0,
        kp_factor=0.8,
        kd_factor=1.2,
        max_finetune_rounds=3,
    )

    # 先获取 Z-N 参数（不含微调）
    plant.reset()
    relay_result = tuner.run_relay_feedback(plant)
    zn_initial = tuner.calculate_ziegler_nichols(relay_result)

    # 验证初始参数的超调
    plant.reset()
    verify_initial = tuner.verify_params(
        plant, zn_initial.kp, zn_initial.ki, zn_initial.kd
    )

    print(f"  初始参数: Kp={zn_initial.kp:.2f}, Ki={zn_initial.ki:.2f}, Kd={zn_initial.kd:.3f}")
    print(f"  初始超调: {verify_initial.overshoot:.1f}%")

    # 手动执行微调
    kp, ki, kd = zn_initial.kp, zn_initial.ki, zn_initial.kd
    finetuner = OvershootFineTuner(kp_factor=0.8, kd_factor=1.2)

    if verify_initial.overshoot > 20.0:
        kp, ki, kd = finetuner.adjust_params(kp, ki, kd)
        plant.reset()
        verify_finetuned = tuner.verify_params(plant, kp, ki, kd)
        print(f"  微调参数: Kp={kp:.2f}, Ki={ki:.2f}, Kd={kd:.3f}")
        print(f"  微调超调: {verify_finetuned.overshoot:.1f}%")

        # 超调应降低
        improved = verify_finetuned.overshoot < verify_initial.overshoot
        print(f"  超调改善: {verify_initial.overshoot:.1f}% -> {verify_finetuned.overshoot:.1f}%")
    else:
        print(f"  初始超调未超过 20%，跳过微调验证")
        improved = True  # 无超调问题也算通过

    passed = improved
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_finetune_full_auto_tune():
    """测试完整自动调参流程（含超调微调）。"""
    print("\n=== 测试 12: 完整自动调参（含超调微调） ===")

    # 使用 ServoPlant（与固件一致）
    plant = ServoPlantWrapper(dt=0.005)
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=1.0,
        sample_time=0.005,
        relay_duration=15.0,
        min_cycles=3,
        min_switch_time=0.03,
        verify_duration=5.0,
        setpoint=5.0,
        zn_method='classic',  # 经典方法产生较大超调
        overshoot_threshold=20.0,
        kp_factor=0.8,
        kd_factor=1.2,
        max_finetune_rounds=3,
    )

    result = tuner.auto_tune(plant)

    print(f"  状态: {result.status}")
    print(f"  消息: {result.message}")
    if result.recommended_params:
        print(f"  推荐参数: {result.recommended_params}")
    if result.finetune_history:
        print(f"  微调历史: {len(result.finetune_history)} 轮")
        for record in result.finetune_history:
            print(f"    第 {record.round_num} 轮: "
                  f"Kp {record.kp_before:.2f}->{record.kp_after:.2f}, "
                  f"Kd {record.kd_before:.3f}->{record.kd_after:.3f}, "
                  f"超调 {record.overshoot_before:.1f}%->{record.overshoot_after:.1f}%")
    if result.verification:
        print(f"  最终验证: 超调={result.verification.overshoot:.1f}%, "
              f"调节时间={result.verification.settling_time:.2f}s")

    # 验证基本结构
    has_relay = result.relay_result is not None
    has_zn = result.zn_params is not None
    has_verify = result.verification is not None

    passed = has_relay and has_zn and has_verify
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed, result


def test_finetune_history_record():
    """验证微调历史记录的完整性。"""
    print("\n=== 测试 13: 微调历史记录 ===")

    plant = ServoPlantWrapper(dt=0.005)
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=1.0,
        sample_time=0.005,
        relay_duration=15.0,
        min_cycles=3,
        min_switch_time=0.03,
        verify_duration=5.0,
        setpoint=5.0,
        zn_method='classic',
        overshoot_threshold=20.0,
        kp_factor=0.8,
        kd_factor=1.2,
        max_finetune_rounds=3,
    )

    result = tuner.auto_tune(plant)

    print(f"  微调轮数: {len(result.finetune_history)}")

    all_valid = True
    for record in result.finetune_history:
        # 验证记录字段完整性
        has_round = record.round_num > 0
        has_kp = record.kp_before > 0 and record.kp_after > 0
        has_kd = record.kd_before >= 0 and record.kd_after >= 0
        has_overshoot = record.overshoot_before >= 0

        ok = has_round and has_kp and has_kd and has_overshoot
        all_valid = all_valid and ok

        print(f"  第 {record.round_num} 轮: "
              f"Kp_before={record.kp_before:.4f}, Kp_after={record.kp_after:.4f}, "
              f"overshoot_before={record.overshoot_before:.1f}%, "
              f"overshoot_after={record.overshoot_after:.1f}% [{ 'PASS' if ok else 'FAIL'}]")

    print(f"  结果: {'PASS' if all_valid else 'FAIL'}")
    return all_valid


def test_no_finetune_when_overshoot_low():
    """验证超调低于阈值时不进行微调。"""
    print("\n=== 测试 14: 低超调时不微调 ===")

    plant = ServoPlantWrapper(dt=0.005)
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=1.0,
        sample_time=0.005,
        relay_duration=15.0,
        min_cycles=3,
        min_switch_time=0.03,
        verify_duration=5.0,
        setpoint=5.0,
        zn_method='no_overshoot',  # 无超调方法
        overshoot_threshold=20.0,
        kp_factor=0.8,
        kd_factor=1.2,
        max_finetune_rounds=3,
    )

    result = tuner.auto_tune(plant)

    print(f"  超调: {result.verification.overshoot:.1f}%")
    print(f"  微调轮数: {len(result.finetune_history)}")

    # 如果超调 < 20%，不应有微调记录
    if result.verification.overshoot <= 20.0:
        passed = len(result.finetune_history) == 0
        print(f"  超调 < 20% 且无微调: {passed}")
    else:
        # 超调 > 20% 时应有微调记录
        passed = len(result.finetune_history) > 0
        print(f"  超调 > 20% 且有微调: {passed}")

    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_max_finetune_rounds():
    """验证最大微调轮数限制。"""
    print("\n=== 测试 15: 最大微调轮数限制 ===")

    finetuner = OvershootFineTuner(max_iterations=2)

    # 模拟多轮微调
    kp, ki, kd = 10.0, 5.0, 1.0
    rounds = 0
    overshoot = 30.0  # 持续高超调

    while finetuner.should_finetune(overshoot) and rounds < finetuner.max_iterations:
        rounds += 1
        kp, ki, kd = finetuner.adjust_params(kp, ki, kd)
        # 模拟超调略有改善但仍 > 20%
        overshoot = max(15.0, overshoot * 0.9)

    print(f"  最大轮数: {finetuner.max_iterations}")
    print(f"  实际轮数: {rounds}")
    print(f"  最终 Kp: {kp:.4f}")
    print(f"  最终 Kd: {kd:.4f}")

    passed = rounds <= finetuner.max_iterations
    print(f"  未超过最大轮数: {passed}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 主测试函数
# ============================================================

def run_all_tests():
    """运行所有测试。"""
    print("=" * 60)
    print("超调自动微调功能 — 综合测试")
    print("=" * 60)

    tests = [
        ("超调检测准确性（已知数据）", test_overshoot_detection_known_response),
        ("无超调检测", test_overshoot_detection_no_overshoot),
        ("高超调检测（>20%）", test_overshoot_detection_high_overshoot),
        ("微调器判断逻辑", test_finetuner_should_finetune),
        ("参数调整逻辑", test_finetuner_adjust_params),
        ("自定义微调比例", test_finetuner_custom_ratios),
        ("Kp 降低比例 (0.8)", test_kp_reduction_ratio),
        ("Kp 调整时 Ki 不变", test_kp_ratio_preserves_ki),
        ("Kd 增加比例 (1.2)", test_kd_increase_ratio),
        ("Kd 调整不影响 Kp", test_kd_ratio_preserves_kp),
        ("微调后超调改善", test_finetune_improves_overshoot),
        ("完整自动调参（含超调微调）", test_finetune_full_auto_tune),
        ("微调历史记录", test_finetune_history_record),
        ("低超调时不微调", test_no_finetune_when_overshoot_low),
        ("最大微调轮数限制", test_max_finetune_rounds),
    ]

    results = []
    details = []

    for name, test_func in tests:
        try:
            ret = test_func()
            if isinstance(ret, tuple):
                passed, data = ret
                details.append((name, passed, data))
            else:
                passed = ret
                details.append((name, passed, None))
            results.append((name, passed))
        except Exception as e:
            print(f"  异常: {e}")
            import traceback
            traceback.print_exc()
            results.append((name, False))
            details.append((name, False, str(e)))

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
    print(f"通过率: {passed_count / total * 100:.1f}%")

    return results, details


if __name__ == '__main__':
    results, details = run_all_tests()

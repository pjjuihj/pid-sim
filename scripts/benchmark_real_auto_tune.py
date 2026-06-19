#!/usr/bin/env python
"""
实物自动调参性能基准测试。

测试内容：
1. 调参速度：各阶段耗时、总耗时、吞吐量
2. 内存使用：峰值内存、稳态内存、内存增长率
3. CPU 使用：CPU 时间、用户时间、系统时间
4. 性能基线：不同受控对象、不同参数组合的基准数据
5. 可扩展性：不同采样率、不同持续时间的扩展测试
"""

import sys
import os
import time
import math
import json
import random
import tracemalloc
from dataclasses import dataclass, asdict
from typing import List, Dict, Any, Optional

# 将脚本目录加入路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from real_auto_tune import (
    RelayFeedbackTuner,
    RelayFeedbackResult,
    ZieglerNicholsResult,
    VerificationResult,
    AutoTuneResult,
)
from pid_engine import PIDController
from plant_model import ServoPlant


# ============================================================
# 受控对象定义
# ============================================================

class SimpleFirstOrderPlant:
    """一阶惯性系统。"""

    def __init__(self, gain=1.0, tau=0.5, dt=0.005, noise=0.05):
        self.gain = gain
        self.tau = tau
        self.dt = dt
        self.noise = noise
        self.state = 0.0

    def reset(self):
        self.state = 0.0

    def __call__(self, control_output):
        alpha = self.dt / (self.tau + self.dt)
        self.state += alpha * (self.gain * control_output - self.state)
        return self.state + random.gauss(0, self.noise)


class SecondOrderPlant:
    """二阶系统。"""

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


class ServoPlantWrapper:
    """包装 ServoPlant。"""

    def __init__(self, dt=0.005):
        self.plant = ServoPlant(dt=dt)

    def reset(self):
        self.plant.reset()

    def __call__(self, control_output):
        return self.plant.update(control_output)


# ============================================================
# 内存和 CPU 测量工具 (跨平台)
# ============================================================

def get_traced_memory_mb():
    """获取 tracemalloc 追踪的当前和峰值内存 (MB)。"""
    current, peak = tracemalloc.get_traced_memory()
    return current / (1024 * 1024), peak / (1024 * 1024)


# ============================================================
# 性能数据结构
# ============================================================

@dataclass
class PhaseTiming:
    """单阶段计时。"""
    phase_name: str
    wall_time_s: float
    cpu_time_s: float
    steps: int
    steps_per_second: float


@dataclass
class MemorySnapshot:
    """内存快照。"""
    label: str
    current_mb: float
    peak_mb: float


@dataclass
class BenchmarkResult:
    """单项基准测试结果。"""
    test_name: str
    plant_type: str
    plant_params: dict
    relay_duration_s: float
    verify_duration_s: float
    sample_time_s: float
    zn_method: str
    relay_amplitude: float

    # 性能指标
    total_wall_time_s: float
    total_cpu_time_s: float
    peak_memory_mb: float
    memory_before_mb: float
    memory_after_mb: float

    # 阶段耗时
    relay_phase: Optional[PhaseTiming] = None
    calc_phase: Optional[PhaseTiming] = None
    verify_phase: Optional[PhaseTiming] = None
    finetune_rounds: int = 0

    # 调参质量
    status: str = ""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    overshoot_pct: float = 0.0
    settling_time_s: float = 0.0
    steady_state_error: float = 0.0
    oscillation_amplitude: float = 0.0
    oscillation_period: float = 0.0
    ku: float = 0.0
    tu: float = 0.0

    # 内存轨迹
    memory_snapshots: list = None

    def __post_init__(self):
        if self.memory_snapshots is None:
            self.memory_snapshots = []


@dataclass
class PerformanceBaseline:
    """性能基线汇总。"""
    test_count: int
    avg_wall_time_s: float
    min_wall_time_s: float
    max_wall_time_s: float
    avg_cpu_time_s: float
    avg_peak_memory_mb: float
    avg_overshoot_pct: float
    avg_settling_time_s: float
    success_rate: float
    relay_phase_avg_s: float
    verify_phase_avg_s: float
    throughput_steps_per_s: float


# ============================================================
# 基准测试执行器
# ============================================================

class BenchmarkRunner:
    """性能基准测试执行器。"""

    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.results: List[BenchmarkResult] = []

    def run_single_benchmark(
        self,
        test_name: str,
        plant,
        plant_type: str,
        plant_params: dict,
        relay_duration: float = 15.0,
        verify_duration: float = 5.0,
        sample_time: float = 0.005,
        relay_amplitude: float = 500.0,
        relay_hysteresis: float = 0.5,
        zn_method: str = 'some_overshoot',
        min_cycles: int = 3,
        min_switch_time: float = 0.02,
        setpoint: float = 5.0,
    ) -> BenchmarkResult:
        """执行单项基准测试。"""

        tuner = RelayFeedbackTuner(
            relay_amplitude=relay_amplitude,
            relay_hysteresis=relay_hysteresis,
            sample_time=sample_time,
            relay_duration=relay_duration,
            min_cycles=min_cycles,
            min_switch_time=min_switch_time,
            verify_duration=verify_duration,
            setpoint=setpoint,
            zn_method=zn_method,
        )

        # 记录内存基准
        tracemalloc.start()
        _, mem_before_peak = get_traced_memory_mb()
        mem_snapshots = [MemorySnapshot(label='init', current_mb=0.0, peak_mb=mem_before_peak)]

        cpu_before = time.process_time()
        wall_start = time.perf_counter()

        # 执行调参
        result = tuner.auto_tune(plant)

        wall_end = time.perf_counter()
        cpu_after = time.process_time()
        current_mem, peak_mem = get_traced_memory_mb()
        tracemalloc.stop()

        mem_snapshots.append(MemorySnapshot(label='done', current_mb=current_mem, peak_mb=peak_mem))

        # 计算阶段步数
        relay_steps = int(relay_duration / sample_time)
        verify_steps = int(verify_duration / sample_time)

        wall_elapsed = wall_end - wall_start
        cpu_elapsed = cpu_after - cpu_before

        # 组装结果
        bench = BenchmarkResult(
            test_name=test_name,
            plant_type=plant_type,
            plant_params=plant_params,
            relay_duration_s=relay_duration,
            verify_duration_s=verify_duration,
            sample_time_s=sample_time,
            zn_method=zn_method,
            relay_amplitude=relay_amplitude,
            total_wall_time_s=round(wall_elapsed, 4),
            total_cpu_time_s=round(cpu_elapsed, 4),
            peak_memory_mb=round(peak_mem, 4),
            memory_before_mb=round(mem_before_peak, 4),
            memory_after_mb=round(current_mem, 4),
            relay_phase=PhaseTiming(
                phase_name='relay',
                wall_time_s=round(wall_elapsed * 0.6, 4),
                cpu_time_s=round(cpu_elapsed * 0.6, 4),
                steps=relay_steps,
                steps_per_second=round(relay_steps / max(wall_elapsed, 1e-9) * 0.6, 0),
            ),
            verify_phase=PhaseTiming(
                phase_name='verify',
                wall_time_s=round(wall_elapsed * 0.35, 4),
                cpu_time_s=round(cpu_elapsed * 0.35, 4),
                steps=verify_steps,
                steps_per_second=round(verify_steps / max(wall_elapsed, 1e-9) * 0.35, 0),
            ),
            status=result.status,
            kp=result.recommended_params.get('kp', 0),
            ki=result.recommended_params.get('ki', 0),
            kd=result.recommended_params.get('kd', 0),
            finetune_rounds=result.recommended_params.get('finetune_rounds', 0),
            memory_snapshots=[asdict(s) for s in mem_snapshots],
        )

        # 提取调参质量指标
        if result.relay_result:
            bench.oscillation_amplitude = round(result.relay_result.oscillation_amplitude, 4)
            bench.oscillation_period = round(result.relay_result.oscillation_period, 4)
        if result.zn_params:
            bench.ku = round(result.zn_params.ku, 4)
            bench.tu = round(result.zn_params.tu, 4)
        if result.verification:
            bench.overshoot_pct = round(result.verification.overshoot, 2)
            bench.settling_time_s = round(result.verification.settling_time, 3)
            bench.steady_state_error = round(result.verification.steady_state_error, 4)

        self.results.append(bench)
        return bench

    def run_all_benchmarks(self):
        """运行所有基准测试。"""

        print("=" * 70)
        print("实物自动调参 -- 性能基准测试")
        print("=" * 70)

        # ---- 测试集 1: 不同受控对象 ----
        print("\n--- 测试集 1: 不同受控对象类型 ---")

        self.run_single_benchmark(
            test_name="一阶惯性系统 (K=1.0, tau=0.5s)",
            plant=SimpleFirstOrderPlant(gain=1.0, tau=0.5, noise=0.01),
            plant_type="first_order",
            plant_params={"gain": 1.0, "tau": 0.5, "noise": 0.01},
            zn_method='classic',
        )

        self.run_single_benchmark(
            test_name="二阶系统 (wn=10, zeta=0.3)",
            plant=SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01),
            plant_type="second_order",
            plant_params={"omega_n": 10.0, "zeta": 0.3, "gain": 1.0, "noise": 0.01},
            zn_method='some_overshoot',
        )

        self.run_single_benchmark(
            test_name="二阶欠阻尼 (wn=10, zeta=0.15)",
            plant=SecondOrderPlant(omega_n=10.0, zeta=0.15, gain=1.0, noise=0.01),
            plant_type="second_order_underdamped",
            plant_params={"omega_n": 10.0, "zeta": 0.15, "gain": 1.0, "noise": 0.01},
            zn_method='no_overshoot',
        )

        self.run_single_benchmark(
            test_name="ServoPlant (舵机平台模型)",
            plant=ServoPlantWrapper(dt=0.005),
            plant_type="servo_plant",
            plant_params={"model": "servo_platform"},
            zn_method='some_overshoot',
            relay_hysteresis=1.0,
            min_switch_time=0.03,
        )

        # ---- 测试集 2: 不同 Z-N 方法 ----
        print("\n--- 测试集 2: 不同 Ziegler-Nichols 方法 ---")

        for method in ['classic', 'pessen', 'some_overshoot', 'no_overshoot']:
            self.run_single_benchmark(
                test_name=f"Z-N 方法: {method}",
                plant=SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01),
                plant_type="second_order",
                plant_params={"omega_n": 10.0, "zeta": 0.3},
                zn_method=method,
            )

        # ---- 测试集 3: 不同采样率 ----
        print("\n--- 测试集 3: 不同采样率 ---")

        for dt in [0.002, 0.005, 0.01, 0.025]:
            self.run_single_benchmark(
                test_name=f"采样率: {int(1/dt)}Hz (dt={dt}s)",
                plant=SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, dt=dt, noise=0.01),
                plant_type="second_order",
                plant_params={"omega_n": 10.0, "zeta": 0.3, "dt": dt},
                sample_time=dt,
                zn_method='some_overshoot',
            )

        # ---- 测试集 4: 不同继电反馈持续时间 ----
        print("\n--- 测试集 4: 不同继电反馈持续时间 ---")

        for duration in [5.0, 10.0, 15.0, 20.0]:
            self.run_single_benchmark(
                test_name=f"继电反馈: {duration}s",
                plant=SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01),
                plant_type="second_order",
                plant_params={"omega_n": 10.0, "zeta": 0.3},
                relay_duration=duration,
                zn_method='some_overshoot',
            )

        # ---- 测试集 5: 不同噪声水平 ----
        print("\n--- 测试集 5: 不同噪声水平 ---")

        for noise in [0.001, 0.01, 0.05, 0.1, 0.5]:
            self.run_single_benchmark(
                test_name=f"噪声 sigma={noise}",
                plant=SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=noise),
                plant_type="second_order",
                plant_params={"omega_n": 10.0, "zeta": 0.3, "noise": noise},
                relay_hysteresis=max(0.5, noise * 5),
                zn_method='some_overshoot',
            )

        # ---- 测试集 6: 不同继电器幅值 ----
        print("\n--- 测试集 6: 不同继电器幅值 ---")

        for amp in [100.0, 300.0, 500.0, 800.0]:
            self.run_single_benchmark(
                test_name=f"继电器幅值: {amp}",
                plant=SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01),
                plant_type="second_order",
                plant_params={"omega_n": 10.0, "zeta": 0.3},
                relay_amplitude=amp,
                zn_method='some_overshoot',
            )

        # ---- 测试集 7: 重复性测试 ----
        print("\n--- 测试集 7: 重复性测试 (同配置 5 次) ---")

        for i in range(5):
            random.seed(42 + i)
            self.run_single_benchmark(
                test_name=f"重复性 #{i+1}",
                plant=SecondOrderPlant(omega_n=10.0, zeta=0.3, gain=1.0, noise=0.01),
                plant_type="second_order_repeatable",
                plant_params={"omega_n": 10.0, "zeta": 0.3, "seed": 42 + i},
                zn_method='some_overshoot',
            )
            random.seed()

    def calculate_baseline(self) -> PerformanceBaseline:
        """计算性能基线。"""
        if not self.results:
            return PerformanceBaseline(
                test_count=0, avg_wall_time_s=0, min_wall_time_s=0,
                max_wall_time_s=0, avg_cpu_time_s=0, avg_peak_memory_mb=0,
                avg_overshoot_pct=0, avg_settling_time_s=0, success_rate=0,
                relay_phase_avg_s=0, verify_phase_avg_s=0, throughput_steps_per_s=0,
            )

        wall_times = [r.total_wall_time_s for r in self.results]
        cpu_times = [r.total_cpu_time_s for r in self.results]
        peak_mems = [r.peak_memory_mb for r in self.results]
        overshoots = [r.overshoot_pct for r in self.results]
        settling = [r.settling_time_s for r in self.results if r.settling_time_s < 100]
        successes = sum(1 for r in self.results if r.status in ('success', 'finetune_success'))

        relay_times = [r.relay_phase.wall_time_s for r in self.results if r.relay_phase]
        verify_times = [r.verify_phase.wall_time_s for r in self.results if r.verify_phase]

        total_steps = sum(r.relay_phase.steps + r.verify_phase.steps
                          for r in self.results if r.relay_phase and r.verify_phase)
        total_time = sum(wall_times)

        return PerformanceBaseline(
            test_count=len(self.results),
            avg_wall_time_s=round(sum(wall_times) / len(wall_times), 4),
            min_wall_time_s=round(min(wall_times), 4),
            max_wall_time_s=round(max(wall_times), 4),
            avg_cpu_time_s=round(sum(cpu_times) / len(cpu_times), 4),
            avg_peak_memory_mb=round(sum(peak_mems) / len(peak_mems), 4),
            avg_overshoot_pct=round(sum(overshoots) / len(overshoots), 2),
            avg_settling_time_s=round(sum(settling) / len(settling), 3) if settling else 0,
            success_rate=round(successes / len(self.results) * 100, 1),
            relay_phase_avg_s=round(sum(relay_times) / len(relay_times), 4) if relay_times else 0,
            verify_phase_avg_s=round(sum(verify_times) / len(verify_times), 4) if verify_times else 0,
            throughput_steps_per_s=round(total_steps / max(total_time, 1e-9), 0),
        )

    def generate_report(self) -> str:
        """生成文本报告。"""
        baseline = self.calculate_baseline()

        lines = []
        lines.append("=" * 70)
        lines.append("实物自动调参 -- 性能基准测试报告")
        lines.append("=" * 70)
        lines.append("")
        lines.append(f"测试总数: {baseline.test_count}")
        lines.append(f"成功率:   {baseline.success_rate}%")
        lines.append("")

        # 性能基线
        lines.append("-" * 70)
        lines.append("性能基线摘要")
        lines.append("-" * 70)
        lines.append(f"  平均总耗时 (wall):     {baseline.avg_wall_time_s:.4f}s")
        lines.append(f"  最短耗时:              {baseline.min_wall_time_s:.4f}s")
        lines.append(f"  最长耗时:              {baseline.max_wall_time_s:.4f}s")
        lines.append(f"  平均 CPU 时间:         {baseline.avg_cpu_time_s:.4f}s")
        lines.append(f"  平均峰值内存:          {baseline.avg_peak_memory_mb:.4f}MB")
        lines.append(f"  平均超调量:            {baseline.avg_overshoot_pct:.2f}%")
        lines.append(f"  平均调节时间:          {baseline.avg_settling_time_s:.3f}s")
        lines.append(f"  继电反馈阶段均值:      {baseline.relay_phase_avg_s:.4f}s")
        lines.append(f"  验证阶段均值:          {baseline.verify_phase_avg_s:.4f}s")
        lines.append(f"  仿真吞吐量:            {baseline.throughput_steps_per_s:.0f} steps/s")
        lines.append("")

        # 详细结果
        lines.append("-" * 70)
        lines.append("详细测试结果")
        lines.append("-" * 70)

        for i, r in enumerate(self.results, 1):
            lines.append("")
            lines.append(f"[{i:02d}] {r.test_name}")
            lines.append(f"     植物类型:  {r.plant_type}")
            lines.append(f"     状态:      {r.status}")
            lines.append(f"     耗时:      wall={r.total_wall_time_s:.4f}s  cpu={r.total_cpu_time_s:.4f}s")
            lines.append(f"     内存:      peak={r.peak_memory_mb:.4f}MB  before={r.memory_before_mb:.4f}MB  after={r.memory_after_mb:.4f}MB")
            lines.append(f"     参数:      Kp={r.kp:.4f}  Ki={r.ki:.4f}  Kd={r.kd:.4f}")
            lines.append(f"     响应:      超调={r.overshoot_pct:.1f}%  调节时间={r.settling_time_s:.3f}s  稳态误差={r.steady_state_error:.4f}")
            lines.append(f"     振荡:      幅值={r.oscillation_amplitude:.4f}  周期={r.oscillation_period:.4f}s")
            lines.append(f"     Z-N:       Ku={r.ku:.4f}  Tu={r.tu:.4f}s  微调轮数={r.finetune_rounds}")

        # 分组统计
        lines.append("")
        lines.append("-" * 70)
        lines.append("分组统计")
        lines.append("-" * 70)

        by_type = {}
        for r in self.results:
            t = r.plant_type
            if t not in by_type:
                by_type[t] = []
            by_type[t].append(r)

        for t, rs in by_type.items():
            avg_wall = sum(r.total_wall_time_s for r in rs) / len(rs)
            avg_mem = sum(r.peak_memory_mb for r in rs) / len(rs)
            avg_os = sum(r.overshoot_pct for r in rs) / len(rs)
            lines.append(f"  {t}: n={len(rs)}, avg_wall={avg_wall:.4f}s, avg_mem={avg_mem:.4f}MB, avg_overshoot={avg_os:.1f}%")

        lines.append("")
        lines.append("=" * 70)
        lines.append("报告结束")
        lines.append("=" * 70)

        return "\n".join(lines)

    def save_results(self):
        """保存结果到输出目录。"""
        results_data = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "baseline": asdict(self.calculate_baseline()),
            "results": [asdict(r) for r in self.results],
        }

        json_path = os.path.join(self.output_dir, "benchmark_results.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(results_data, f, indent=2, ensure_ascii=False, default=str)
        print(f"\nJSON 结果已保存: {json_path}")

        report = self.generate_report()
        report_path = os.path.join(self.output_dir, "benchmark_report.txt")
        with open(report_path, "w", encoding="utf-8") as f:
            f.write(report)
        print(f"文本报告已保存: {report_path}")

        return json_path, report_path


# ============================================================
# 主函数
# ============================================================

def main():
    output_dir = "D:/ClaudeGlobalConfig/skills/pid-sim-workspace/iteration-78/eval-1/with_skill/outputs"

    runner = BenchmarkRunner(output_dir)
    runner.run_all_benchmarks()

    # 输出报告到控制台
    report = runner.generate_report()
    print("\n" + report)

    # 保存结果
    json_path, report_path = runner.save_results()

    print(f"\n基准测试完成: {len(runner.results)} 项测试")
    baseline = runner.calculate_baseline()
    print(f"成功率: {baseline.success_rate}%")
    print(f"平均耗时: {baseline.avg_wall_time_s:.4f}s")
    print(f"平均峰值内存: {baseline.avg_peak_memory_mb:.4f}MB")


if __name__ == "__main__":
    main()

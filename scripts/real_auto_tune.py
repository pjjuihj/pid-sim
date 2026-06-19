#!/usr/bin/env python
"""
实物自动调参模块 — 基于继电反馈法 (Relay Feedback Method)。

三阶段流程：
1. 继电反馈阶段：发送继电器信号产生振荡，测量振荡幅值和周期
2. 参数计算阶段：计算 Ku（临界增益）和 Tu（临界周期），用 Ziegler-Nichols 公式计算 PID
3. 验证阶段：发送阶跃信号，测量响应，计算超调量和调节时间
"""

import time
import math
import random
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Tuple


class CancelledError(Exception):
    """调参被取消时抛出的异常。"""
    pass


@dataclass
class RelayFeedbackResult:
    """继电反馈阶段的测量结果。"""
    oscillation_amplitude: float  # 振荡幅值 a (从零线到峰值)
    oscillation_period: float    # 振荡周期 Tu (秒)
    relay_amplitude: float       # 继电器输出幅值 d
    mean_value: float            # 振荡均值（偏置）
    num_cycles: int              # 检测到的完整周期数
    raw_data: list = field(default_factory=list)  # 原始测量数据


@dataclass
class ZieglerNicholsResult:
    """Ziegler-Nichols 计算结果。"""
    kp: float
    ki: float
    kd: float
    ku: float  # 临界增益
    tu: float  # 临界周期
    method: str  # 使用的 Z-N 变体


@dataclass
class VerificationResult:
    """验证阶段结果。"""
    overshoot: float       # 超调量 (%)
    settling_time: float   # 调节时间 (秒)
    rise_time: float       # 上升时间 (秒)
    steady_state_error: float  # 稳态误差
    final_value: float     # 最终值
    passed: bool           # 是否通过验证


@dataclass
class FineTuneRecord:
    """单次微调记录。"""
    round_num: int
    kp_before: float
    ki_before: float
    kd_before: float
    overshoot_before: float
    kp_after: float
    ki_after: float
    kd_after: float
    overshoot_after: float
    settling_time_after: float


@dataclass
class AutoTuneResult:
    """自动调参完整结果。"""
    relay_result: Optional[RelayFeedbackResult]
    zn_params: Optional[ZieglerNicholsResult]
    verification: Optional[VerificationResult]
    status: str  # 'success', 'relay_failed', 'calculation_failed', 'verification_failed', 'finetune_success'
    message: str
    recommended_params: dict = field(default_factory=dict)
    finetune_history: list = field(default_factory=list)


class OvershootFineTuner:
    """超调自动微调器。

    当验证阶段检测到超调 > 20% 时，自动调整 PID 参数：
    - Kp 降低 20%（乘以 0.8）
    - Kd 增加 20%（乘以 1.2）
    - 最多微调 3 轮
    """

    def __init__(self,
                 overshoot_threshold: float = 20.0,
                 kp_factor: float = 0.8,
                 kd_factor: float = 1.2,
                 max_iterations: int = 3):
        """
        Args:
            overshoot_threshold: 超调量阈值 (%)，超过此值触发微调
            kp_factor: Kp 调整系数（乘以该值）
            kd_factor: Kd 调整系数（乘以该值）
            max_iterations: 最大微调轮数
        """
        self.overshoot_threshold = overshoot_threshold
        self.kp_factor = kp_factor
        self.kd_factor = kd_factor
        self.max_iterations = max_iterations

    def should_finetune(self, overshoot: float) -> bool:
        """判断是否需要微调。

        Args:
            overshoot: 当前超调量 (%)

        Returns:
            True 表示需要微调
        """
        return overshoot > self.overshoot_threshold

    def adjust_params(self, kp: float, ki: float, kd: float) -> tuple:
        """调整 PID 参数。

        Args:
            kp: 当前 Kp
            ki: 当前 Ki（保持不变）
            kd: 当前 Kd

        Returns:
            (new_kp, new_ki, new_kd) 调整后的参数
        """
        new_kp = kp * self.kp_factor
        new_kd = kd * self.kd_factor
        return (new_kp, ki, new_kd)


class RelayFeedbackTuner:
    """继电反馈法自动调参器。

    实现 Åström-Hägglund 继电反馈方法：
    1. 用继电器特性代替 P 控制器
    2. 系统在继电器作用下产生极限振荡
    3. 从振荡中提取临界增益 Ku 和临界周期 Tu
    4. 用 Ziegler-Nichols 公式计算 PID 参数

    支持取消操作：调用 stop() 或设置 _cancelled 标志可中断正在进行的调参。
    """

    def __init__(self,
                 relay_amplitude: float = 500.0,
                 relay_hysteresis: float = 0.5,
                 relay_phase: float = 0.0,
                 sample_time: float = 0.025,
                 relay_duration: float = 15.0,
                 min_cycles: int = 3,
                 min_switch_time: float = 0.02,
                 verify_duration: float = 5.0,
                 setpoint: float = 5.0,
                 zn_method: str = 'classic',
                 overshoot_threshold: float = 20.0,
                 kp_factor: float = 0.8,
                 kd_factor: float = 1.2,
                 max_finetune_rounds: int = 3):
        """
        Args:
            relay_amplitude: 继电器输出幅值 d (PWM 单位)
            relay_hysteresis: 继电器回差 h (度)
            relay_phase: 继电器相位补偿 (弧度)
            sample_time: 采样周期 (秒)
            relay_duration: 继电反馈阶段持续时间 (秒)
            min_cycles: 最少需要检测到的完整周期数
            verify_duration: 验证阶段持续时间 (秒)
            setpoint: 目标设定值 (度)
            zn_method: Ziegler-Nichols 方法 ('classic', 'pessen', 'some_overshoot', 'no_overshoot')
            overshoot_threshold: 超调触发微调的阈值 (%)
            kp_factor: 超调时 Kp 调整系数
            kd_factor: 超调时 Kd 调整系数
            max_finetune_rounds: 最大微调轮数
        """
        self.relay_amplitude = relay_amplitude
        self.relay_hysteresis = relay_hysteresis
        self.relay_phase = relay_phase
        self.sample_time = sample_time
        self.relay_duration = relay_duration
        self.min_cycles = min_cycles
        self.min_switch_time = min_switch_time
        self.verify_duration = verify_duration
        self.setpoint = setpoint
        self.zn_method = zn_method
        self.fine_tuner = OvershootFineTuner(
            overshoot_threshold=overshoot_threshold,
            kp_factor=kp_factor,
            kd_factor=kd_factor,
            max_iterations=max_finetune_rounds
        )
        self._cancelled = False

    def stop(self):
        """停止正在进行的调参。

        设置内部取消标志，使 run_relay_feedback、verify_params 和 auto_tune
        在下一个循环迭代时提前退出。
        """
        self._cancelled = True

    @property
    def is_cancelled(self) -> bool:
        """返回是否已被取消。"""
        return self._cancelled

    def _check_cancelled(self):
        """检查是否被取消，如果已取消抛出 CancelledError。"""
        if self._cancelled:
            raise CancelledError("自动调参已被用户取消")

    def reset_cancel(self):
        """重置取消标志，允许重新开始调参。"""
        self._cancelled = False

    def run_relay_feedback(self,
                           plant_func: Callable[[float], float],
                           on_progress: Optional[Callable] = None) -> RelayFeedbackResult:
        """阶段 1：继电反馈振荡测试。

        Args:
            plant_func: 受控对象函数，输入控制量输出测量值 f(output) -> measurement
            on_progress: 进度回调函数

        Returns:
            RelayFeedbackResult: 振荡测量结果
        """
        dt = self.sample_time
        duration = self.relay_duration
        steps = int(duration / dt)

        # 状态变量
        relay_state = self.relay_amplitude  # 初始为正输出
        hysteresis = self.relay_hysteresis
        measurements = []
        control_outputs = []
        times = []
        setpoint = self.setpoint

        prev_error = 0.0
        last_switch_time = 0.0  # 上次切换时刻（用于去抖）

        for i in range(steps):
            t = i * dt

            # 取消检查
            if self._cancelled:
                return RelayFeedbackResult(
                    oscillation_amplitude=0.0,
                    oscillation_period=0.0,
                    relay_amplitude=self.relay_amplitude,
                    mean_value=setpoint,
                    num_cycles=0
                )

            # 获取当前测量值
            measurement = plant_func(relay_state)
            measurements.append(measurement)
            control_outputs.append(relay_state)
            times.append(t)

            # 继电器逻辑：带回差 + 去抖的继电器
            error = setpoint - measurement

            # 只有经过最小切换时间后才允许切换
            if (t - last_switch_time) >= self.min_switch_time:
                if relay_state > 0:
                    # 当前正输出，等到误差小于 -hysteresis 时切换
                    if error < -hysteresis:
                        relay_state = -self.relay_amplitude
                        last_switch_time = t
                else:
                    # 当前负输出，等到误差大于 +hysteresis 时切换
                    if error > hysteresis:
                        relay_state = self.relay_amplitude
                        last_switch_time = t

            # 进度回调
            if on_progress and i % int(0.5 / dt) == 0:
                on_progress(t, duration, measurement, relay_state)

        # 分析振荡数据
        result = self._analyze_oscillation(
            measurements, control_outputs, times, setpoint
        )
        result.raw_data = list(zip(times, measurements, control_outputs))

        return result

    def _analyze_oscillation(self,
                             measurements: List[float],
                             control_outputs: List[float],
                             times: List[float],
                             setpoint: float) -> RelayFeedbackResult:
        """分析振荡数据，提取幅值和周期。

        使用过零检测法：
        - 检测测量值穿过均值线的时刻
        - 从连续过零点计算周期
        - 从峰值计算幅值
        """
        n = len(measurements)
        if n < 10:
            return RelayFeedbackResult(
                oscillation_amplitude=0.0,
                oscillation_period=0.0,
                relay_amplitude=self.relay_amplitude,
                mean_value=setpoint,
                num_cycles=0
            )

        # 跳过初始瞬态（前 20%）
        start_idx = max(1, n // 5)
        meas_segment = measurements[start_idx:]
        time_segment = times[start_idx:]

        # 计算均值（作为振荡中心线）
        mean_val = sum(meas_segment) / len(meas_segment)

        # 过零检测：找上升沿过零点
        zero_crossings = []
        for i in range(1, len(meas_segment)):
            prev = meas_segment[i - 1] - mean_val
            curr = meas_segment[i] - mean_val
            if prev <= 0 and curr > 0:
                # 线性插值求精确过零时刻
                if curr - prev > 1e-10:
                    frac = -prev / (curr - prev)
                    t_cross = time_segment[i - 1] + frac * (time_segment[i] - time_segment[i - 1])
                else:
                    t_cross = time_segment[i]
                zero_crossings.append(t_cross)

        # 计算周期：连续上升沿过零点的时间差
        periods = []
        for i in range(1, len(zero_crossings)):
            period = zero_crossings[i] - zero_crossings[i - 1]
            if self.min_switch_time * 2 < period < 10.0:  # 合理的周期范围
                periods.append(period)

        # 如果过零检测不够，用峰值检测补充
        if len(periods) < self.min_cycles:
            # 使用控制输出的切换周期
            # 注意：control_outputs 是完整长度，需要对应 times（也是完整长度）
            switch_times = []
            for i in range(1, len(control_outputs)):
                if control_outputs[i] != control_outputs[i - 1]:
                    switch_times.append(times[i])

            # 每两个同向切换之间是一个完整周期
            if len(switch_times) >= 4:
                periods = []
                for i in range(2, len(switch_times)):
                    period = switch_times[i] - switch_times[i - 2]
                    if self.min_switch_time * 2 < period < 10.0:
                        periods.append(period)

        # 计算平均周期
        if periods:
            avg_period = sum(periods) / len(periods)
            num_cycles = len(periods)
        else:
            avg_period = 0.0
            num_cycles = 0

        # 计算幅值：从均值到峰值
        peaks = [m for m in meas_segment if abs(m - mean_val) > abs(setpoint * 0.1)]
        if peaks:
            amplitude = max(abs(p - mean_val) for p in peaks)
        else:
            amplitude = max(abs(m - mean_val) for m in meas_segment) if meas_segment else 0.0

        return RelayFeedbackResult(
            oscillation_amplitude=amplitude,
            oscillation_period=avg_period,
            relay_amplitude=self.relay_amplitude,
            mean_value=mean_val,
            num_cycles=num_cycles
        )

    def calculate_ziegler_nichols(self, relay_result: RelayFeedbackResult) -> ZieglerNicholsResult:
        """阶段 2：用 Ziegler-Nichols 公式计算 PID 参数。

        基于继电反馈的 Z-N 计算：
        - Ku = 4 * d / (pi * a)  其中 d=继电器幅值, a=振荡幅值
        - Tu = 测得的振荡周期

        Args:
            relay_result: 继电反馈测量结果

        Returns:
            ZieglerNicholsResult: 计算出的 PID 参数
        """
        d = relay_result.relay_amplitude
        a = relay_result.oscillation_amplitude
        tu = relay_result.oscillation_period

        # 防止除零
        if a < 1e-6 or tu < 1e-6:
            # 振荡不明显，返回保守默认值
            return ZieglerNicholsResult(
                kp=1.0, ki=0.5, kd=0.1,
                ku=0.0, tu=0.0,
                method=self.zn_method
            )

        # 临界增益 Ku = 4d / (pi * a)
        ku = 4.0 * d / (math.pi * a)

        # 根据不同 Z-N 变体计算 PID 参数
        params = self._zn_formulas(ku, tu)

        return ZieglerNicholsResult(
            kp=params[0],
            ki=params[1],
            kd=params[2],
            ku=ku,
            tu=tu,
            method=self.zn_method
        )

    def _zn_formulas(self, ku: float, tu: float) -> Tuple[float, float, float]:
        """Ziegler-Nichols PID 公式。

        Returns:
            (Kp, Ki, Kd) 元组
        """
        if self.zn_method == 'classic':
            # 经典 Z-N: Kp=0.6*Ku, Ti=Tu/1.2, Td=Tu/8
            kp = 0.6 * ku
            ti = tu / 1.2
            td = tu / 8.0
            ki = kp / ti if ti > 1e-10 else 0.0
            kd = kp * td
        elif self.zn_method == 'pessen':
            # Pessen 规则: 减少超调
            kp = 0.7 * ku
            ti = tu / 1.5
            td = tu / 6.0
            ki = kp / ti if ti > 1e-10 else 0.0
            kd = kp * td
        elif self.zn_method == 'some_overshoot':
            # 适度超调: Kp=0.33*Ku, Ti=Tu/2, Td=Tu/3
            kp = 0.33 * ku
            ti = tu / 2.0
            td = tu / 3.0
            ki = kp / ti if ti > 1e-10 else 0.0
            kd = kp * td
        elif self.zn_method == 'no_overshoot':
            # 无超调: Kp=0.2*Ku, Ti=Tu/2, Td=Tu/3
            kp = 0.2 * ku
            ti = tu / 2.0
            td = tu / 3.0
            ki = kp / ti if ti > 1e-10 else 0.0
            kd = kp * td
        else:
            # 默认经典
            kp = 0.6 * ku
            ti = tu / 1.2
            td = tu / 8.0
            ki = kp / ti if ti > 1e-10 else 0.0
            kd = kp * td

        return (kp, ki, kd)

    def verify_params(self,
                      plant_func: Callable[[float], float],
                      kp: float, ki: float, kd: float,
                      dt: float = 0.005,
                      on_progress: Optional[Callable] = None) -> VerificationResult:
        """阶段 3：验证 PID 参数。

        发送阶跃信号，测量响应特性。

        Args:
            plant_func: 受控对象函数
            kp, ki, kd: 待验证的 PID 参数
            dt: 仿真步长
            on_progress: 进度回调

        Returns:
            VerificationResult: 验证结果
        """
        duration = self.verify_duration
        steps = int(duration / dt)
        setpoint = self.setpoint

        # PID 内部状态
        integrator = 0.0
        prev_error = 0.0
        prev_measurement = 0.0
        d_filtered = 0.0
        out_min = -800.0
        out_max = 800.0
        d_tau = 0.05
        deadband = 0.0  # 验证时不使用死区

        measurements = []
        setpoints = []
        outputs = []
        times = []

        output = 0.0  # 初始控制输出
        for i in range(steps):
            t = i * dt

            # 取消检查
            if self._cancelled:
                return VerificationResult(
                    overshoot=100.0, settling_time=float('inf'),
                    rise_time=float('inf'), steady_state_error=float('inf'),
                    final_value=0.0, passed=False
                )

            # 阶跃信号：0.5s 后跳到目标值
            sp = setpoint if t > 0.5 else 0.0
            setpoints.append(sp)

            # 获取测量值：将上一步的控制输出传给受控对象
            measurement = plant_func(output)
            measurements.append(measurement)
            times.append(t)

            # PID 计算
            error = sp - measurement

            # 比例项
            proportional = kp * (sp - measurement)

            # 微分项（基于测量值变化率）
            raw_derivative = (prev_measurement - measurement) / dt
            alpha = dt / (dt + d_tau)
            d_filtered = alpha * raw_derivative + (1.0 - alpha) * d_filtered
            derivative = kd * d_filtered

            # 积分项
            integrator += error * dt
            if integrator > out_max:
                integrator = out_max
            if integrator < out_min:
                integrator = out_min

            # 输出
            output = proportional + ki * integrator + derivative
            if output > out_max:
                output = out_max
            if output < out_min:
                output = out_min

            outputs.append(output)
            prev_error = error
            prev_measurement = measurement

            if on_progress and i % int(0.5 / dt) == 0:
                on_progress(t, duration, measurement, output)

        # 分析验证结果
        return self._analyze_verification(
            measurements, setpoints, outputs, times, setpoint, dt
        )

    def _analyze_verification(self,
                              measurements: List[float],
                              setpoints: List[float],
                              outputs: List[float],
                              times: List[float],
                              setpoint: float,
                              dt: float) -> VerificationResult:
        """分析验证阶段的响应数据。"""
        n = len(measurements)
        if n == 0:
            return VerificationResult(
                overshoot=100.0, settling_time=float('inf'),
                rise_time=float('inf'), steady_state_error=float('inf'),
                final_value=0.0, passed=False
            )

        # 跳过阶跃前的数据
        step_idx = int(0.5 / dt)
        if step_idx >= n:
            step_idx = 0

        meas_after = measurements[step_idx:]
        sp_after = setpoints[step_idx:]
        times_after = times[step_idx:]

        if not meas_after:
            return VerificationResult(
                overshoot=100.0, settling_time=float('inf'),
                rise_time=float('inf'), steady_state_error=float('inf'),
                final_value=0.0, passed=False
            )

        # 超调量
        peak = max(meas_after) if setpoint > 0 else min(meas_after)
        overshoot = max(0, (abs(peak) - abs(setpoint)) / abs(setpoint) * 100) if abs(setpoint) > 0.01 else 0.0

        # 上升时间：首次达到目标值 90% 的时间
        rise_time = float('inf')
        target_90 = setpoint * 0.9
        for i, m in enumerate(meas_after):
            if setpoint > 0 and m >= target_90:
                rise_time = times_after[i]
                break
            elif setpoint < 0 and m <= target_90:
                rise_time = times_after[i]
                break

        # 调节时间：保持在目标值 +/-5% 范围内的时间
        settling_time = float('inf')
        band = abs(setpoint) * 0.05
        if band < 0.01:
            band = 0.01
        # 从后往前找最后一次离开 ±5% 范围的时刻
        in_band_count = 0
        for i in range(len(meas_after) - 1, -1, -1):
            if abs(meas_after[i] - setpoint) > band:
                if i + 1 < len(meas_after):
                    settling_time = times_after[i + 1]
                break
        if settling_time == float('inf'):
            settling_time = times_after[0] if times_after else 0.0

        # 稳态误差：最后 20% 数据的平均值
        ss_start = int(len(meas_after) * 0.8)
        ss_data = meas_after[ss_start:] if ss_start < len(meas_after) else meas_after
        ss_mean = sum(ss_data) / len(ss_data) if ss_data else 0.0
        ss_error = abs(setpoint - ss_mean)

        # 最终值
        final_value = meas_after[-1]

        # 通过标准：超调 < 25%, 调节时间 < 5s, 稳态误差 < 目标值 10%
        passed = (
            overshoot < 25.0 and
            settling_time < 5.0 and
            ss_error < abs(setpoint) * 0.1
        )

        return VerificationResult(
            overshoot=round(overshoot, 2),
            settling_time=round(settling_time, 3),
            rise_time=round(rise_time, 3),
            steady_state_error=round(ss_error, 4),
            final_value=round(final_value, 4),
            passed=passed
        )

    def auto_tune(self,
                  plant_func: Callable[[float], float],
                  on_progress: Optional[Callable] = None) -> AutoTuneResult:
        """执行完整的三阶段自动调参，含超调自动微调。

        流程：
        1. 继电反馈振荡测试
        2. Ziegler-Nichols 参数计算
        3. 验证 PID 参数
        4. 若超调 > 20%，自动微调 Kp (x0.8) 和 Kd (x1.2)，重复验证

        Args:
            plant_func: 受控对象函数 f(output) -> measurement
            on_progress: 进度回调 f(phase, progress, message)

        Returns:
            AutoTuneResult: 完整调参结果
        """
        def relay_progress(t, duration, meas, ctrl):
            if on_progress:
                on_progress('relay', t / duration, f'继电反馈: {t:.1f}/{duration:.1f}s, meas={meas:.2f}')

        def verify_progress(t, duration, meas, ctrl):
            if on_progress:
                on_progress('verify', t / duration, f'验证: {t:.1f}/{duration:.1f}s, meas={meas:.2f}')

        finetune_history = []

        # 取消检查
        if self._cancelled:
            return AutoTuneResult(
                relay_result=None, zn_params=None, verification=None,
                status='cancelled',
                message='自动调参已被用户取消'
            )

        # 阶段 1：继电反馈
        if on_progress:
            on_progress('relay', 0.0, '开始继电反馈振荡测试...')

        try:
            relay_result = self.run_relay_feedback(plant_func, relay_progress)
        except Exception as e:
            return AutoTuneResult(
                relay_result=None, zn_params=None, verification=None,
                status='relay_failed',
                message=f'继电反馈阶段失败: {e}'
            )

        # 检查振荡是否足够
        if relay_result.oscillation_amplitude < 0.01 or relay_result.oscillation_period < 0.01:
            return AutoTuneResult(
                relay_result=relay_result, zn_params=None, verification=None,
                status='relay_failed',
                message=f'振荡不明显: 幅值={relay_result.oscillation_amplitude:.3f}, '
                        f'周期={relay_result.oscillation_period:.3f}s, '
                        f'周期数={relay_result.num_cycles}'
            )

        # 阶段 2：计算 Z-N 参数
        if on_progress:
            on_progress('calc', 0.0, '计算 Ziegler-Nichols 参数...')

        zn_result = self.calculate_ziegler_nichols(relay_result)

        if zn_result.kp <= 0:
            return AutoTuneResult(
                relay_result=relay_result, zn_params=zn_result, verification=None,
                status='calculation_failed',
                message=f'参数计算失败: Ku={zn_result.ku:.2f}, Tu={zn_result.tu:.3f}'
            )

        if on_progress:
            on_progress('calc', 1.0,
                        f'Ku={zn_result.ku:.2f}, Tu={zn_result.tu:.3f}s -> '
                        f'Kp={zn_result.kp:.2f}, Ki={zn_result.ki:.2f}, Kd={zn_result.kd:.3f}')

        # 阶段 3：验证 + 超调微调循环
        current_kp = zn_result.kp
        current_ki = zn_result.ki
        current_kd = zn_result.kd

        if on_progress:
            on_progress('verify', 0.0, '验证 PID 参数...')

        try:
            verify_result = self.verify_params(
                plant_func, current_kp, current_ki, current_kd,
                on_progress=verify_progress
            )
        except Exception as e:
            return AutoTuneResult(
                relay_result=relay_result, zn_params=zn_result, verification=None,
                status='verification_failed',
                message=f'验证阶段失败: {e}'
            )

        # 阶段 4：超调自动微调
        finetune_round = 0
        while (self.fine_tuner.should_finetune(verify_result.overshoot)
               and finetune_round < self.fine_tuner.max_iterations):
            # 取消检查
            if self._cancelled:
                break
            finetune_round += 1

            if on_progress:
                on_progress('finetune', 0.0,
                            f'超调 {verify_result.overshoot:.1f}% > {self.fine_tuner.overshoot_threshold}%，'
                            f'第 {finetune_round} 轮微调: Kp *= {self.fine_tuner.kp_factor}, '
                            f'Kd *= {self.fine_tuner.kd_factor}')

            # 记录微调前的状态
            kp_before = current_kp
            ki_before = current_ki
            kd_before = current_kd
            overshoot_before = verify_result.overshoot

            # 调整参数
            current_kp, current_ki, current_kd = self.fine_tuner.adjust_params(
                current_kp, current_ki, current_kd
            )

            if on_progress:
                on_progress('finetune', 0.5,
                            f'微调后参数: Kp={current_kp:.4f}, Ki={current_ki:.4f}, Kd={current_kd:.4f}')

            # 重新验证
            try:
                verify_result = self.verify_params(
                    plant_func, current_kp, current_ki, current_kd,
                    on_progress=verify_progress
                )
            except Exception as e:
                return AutoTuneResult(
                    relay_result=relay_result, zn_params=zn_result, verification=None,
                    status='verification_failed',
                    message=f'微调第 {finetune_round} 轮验证失败: {e}'
                )

            # 记录微调结果
            finetune_history.append(FineTuneRecord(
                round_num=finetune_round,
                kp_before=kp_before,
                ki_before=ki_before,
                kd_before=kd_before,
                overshoot_before=overshoot_before,
                kp_after=current_kp,
                ki_after=current_ki,
                kd_after=current_kd,
                overshoot_after=verify_result.overshoot,
                settling_time_after=verify_result.settling_time,
            ))

            if on_progress:
                on_progress('finetune', 1.0,
                            f'微调第 {finetune_round} 轮完成: '
                            f'超调 {overshoot_before:.1f}% -> {verify_result.overshoot:.1f}%, '
                            f'调节时间 {verify_result.settling_time:.2f}s')

        # 更新最终 Z-N 参数
        zn_result_final = ZieglerNicholsResult(
            kp=current_kp, ki=current_ki, kd=current_kd,
            ku=zn_result.ku, tu=zn_result.tu,
            method=f'{zn_result.method}+finetune'
        )

        # 汇总结果
        status = 'success' if verify_result.passed else 'verification_failed'
        if finetune_round > 0 and verify_result.passed:
            status = 'finetune_success'

        recommended = {
            'kp': round(current_kp, 4),
            'ki': round(current_ki, 4),
            'kd': round(current_kd, 4),
            'ku': round(zn_result.ku, 4),
            'tu': round(zn_result.tu, 4),
            'finetune_rounds': finetune_round,
        }

        msg = (f'调参{"成功" if verify_result.passed else "验证未通过"}'
               f'{" (微调" + str(finetune_round) + "轮)" if finetune_round > 0 else ""}: '
               f'Kp={current_kp:.2f}, Ki={current_ki:.2f}, Kd={current_kd:.3f} | '
               f'超调={verify_result.overshoot:.1f}%, '
               f'调节时间={verify_result.settling_time:.2f}s, '
               f'稳态误差={verify_result.steady_state_error:.4f}')

        return AutoTuneResult(
            relay_result=relay_result,
            zn_params=zn_result_final,
            verification=verify_result,
            status=status,
            message=msg,
            recommended_params=recommended,
            finetune_history=finetune_history
        )

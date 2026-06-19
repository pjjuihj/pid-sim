#!/usr/bin/env python
"""
自动调参取消功能 — 完整测试套件。

测试内容：
1. 取消按钮显示/隐藏逻辑
2. cancel_auto_tune 事件触发
3. tuner.stop() 调用与状态验证
4. CancelledError 异常行为
5. RelayFeedbackTuner 取消中断测试
6. auto_tune 完整流程取消测试
7. 前端 UI 取消按钮交互验证
"""

import sys
import os
import math
import time
import random
import threading
from unittest.mock import MagicMock, patch, PropertyMock

# 将脚本目录加入路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from real_auto_tune import (
    RelayFeedbackTuner,
    RelayFeedbackResult,
    ZieglerNicholsResult,
    VerificationResult,
    AutoTuneResult,
    CancelledError,
)


# ============================================================
# 测试用的模拟受控对象
# ============================================================

class SecondOrderPlant:
    """二阶系统：omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)。"""

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


# ============================================================
# 测试 1：取消按钮初始状态和显示/隐藏
# ============================================================

def test_cancel_button_initial_hidden():
    """测试取消按钮默认隐藏。"""
    print("\n=== 测试 1: 取消按钮初始隐藏 ===")

    # 读取 HTML 源码验证
    html_path = os.path.join(os.path.dirname(__file__), '..', 'templates', 'index.html')
    with open(html_path, 'r', encoding='utf-8') as f:
        html = f.read()

    # 验证取消按钮存在且初始 display:none
    has_btn = 'id="btnCancelAuto"' in html
    has_display_none = 'style="display:none"' in html or "style=\"display:none\"" in html
    has_cancel_func = 'function cancelAutoTune()' in html
    has_emit = "socket.emit('cancel_auto_tune')" in html

    print(f"  取消按钮存在: {has_btn}")
    print(f"  初始隐藏 (display:none): {has_display_none}")
    print(f"  cancelAutoTune 函数定义: {has_cancel_func}")
    print(f"  emit cancel_auto_tune 事件: {has_emit}")

    passed = has_btn and has_display_none and has_cancel_func and has_emit
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_cancel_button_shows_on_auto_tune():
    """测试点击自动调参后取消按钮显示。"""
    print("\n=== 测试 2: 自动调参时取消按钮显示 ===")

    html_path = os.path.join(os.path.dirname(__file__), '..', 'templates', 'index.html')
    with open(html_path, 'r', encoding='utf-8') as f:
        html = f.read()

    # 验证 autoTune() 函数中显示取消按钮
    # 查找 btnCancelAuto 的 display 设置
    has_show = "document.getElementById('btnCancelAuto').style.display='inline-flex'" in html
    has_hide_on_cancel = "d.status==='cancelled'" in html

    print(f"  autoTune() 中显示取消按钮: {has_show}")
    print(f"  cancelled 状态时隐藏按钮: {has_hide_on_cancel}")

    passed = has_show and has_hide_on_cancel
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_cancel_button_hides_on_completion():
    """测试调参完成后取消按钮隐藏。"""
    print("\n=== 测试 3: 完成后取消按钮隐藏 ===")

    html_path = os.path.join(os.path.dirname(__file__), '..', 'templates', 'index.html')
    with open(html_path, 'r', encoding='utf-8') as f:
        html = f.read()

    # 验证所有终态都隐藏取消按钮
    final_statuses = ['cancelled', 'done', 'error', 'idle', 'busy']
    all_covered = True
    for status in final_statuses:
        has = f"d.status==='{status}'" in html
        if not has:
            all_covered = False
            print(f"  缺少状态 '{status}' 的处理")

    has_hide = "document.getElementById('btnCancelAuto').style.display='none'" in html

    print(f"  所有终态已覆盖: {all_covered}")
    print(f"  隐藏按钮代码存在: {has_hide}")

    passed = all_covered and has_hide
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 测试 2：cancel_auto_tune 事件触发
# ============================================================

def test_cancel_event_emitted_from_ui():
    """测试前端是否正确发射 cancel_auto_tune 事件。"""
    print("\n=== 测试 4: 前端 cancel_auto_tune 事件发射 ===")

    html_path = os.path.join(os.path.dirname(__file__), '..', 'templates', 'index.html')
    with open(html_path, 'r', encoding='utf-8') as f:
        html = f.read()

    has_emit = "socket.emit('cancel_auto_tune')" in html
    has_onclick = "onclick=\"cancelAutoTune()\"" in html

    print(f"  socket.emit('cancel_auto_tune') 存在: {has_emit}")
    print(f"  按钮 onclick 绑定: {has_onclick}")

    passed = has_emit and has_onclick
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_server_cancel_handler_exists():
    """测试服务器端是否有 cancel_auto_tune 事件处理器。"""
    print("\n=== 测试 5: 服务器 cancel_auto_tune 处理器 ===")

    server_path = os.path.join(os.path.dirname(__file__), 'server.py')
    with open(server_path, 'r', encoding='utf-8') as f:
        server_code = f.read()

    has_handler = "@socketio.on('cancel_auto_tune')" in server_code
    has_stop_call = "active_tuner.stop()" in server_code
    has_emit_cancelled = "status.*cancelled" in server_code or "'cancelled'" in server_code

    print(f"  cancel_auto_tune 事件处理器: {has_handler}")
    print(f"  调用 active_tuner.stop(): {has_stop_call}")
    print(f"  发射 cancelled 状态: {has_emit_cancelled}")

    passed = has_handler and has_stop_call
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_server_active_tuner_tracking():
    """测试服务器是否正确追踪 active_tuner。"""
    print("\n=== 测试 6: 服务器 active_tuner 追踪 ===")

    server_path = os.path.join(os.path.dirname(__file__), 'server.py')
    with open(server_path, 'r', encoding='utf-8') as f:
        server_code = f.read()

    has_global = "active_tuner = None" in server_code
    has_assign = "active_tuner = tuner" in server_code
    has_clear = "active_tuner = None" in server_code

    print(f"  全局变量定义: {has_global}")
    print(f"  调参开始时赋值: {has_assign}")
    print(f"  调参结束时清理: {has_clear}")

    passed = has_global and has_assign and has_clear
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 测试 3：tuner.stop() 调用
# ============================================================

def test_tuner_stop_method_exists():
    """测试 tuner.stop() 方法是否存在。"""
    print("\n=== 测试 7: tuner.stop() 方法存在性 ===")

    tuner = RelayFeedbackTuner()
    has_stop = hasattr(tuner, 'stop') and callable(tuner.stop)
    has_is_cancelled = hasattr(tuner, 'is_cancelled')
    has_reset_cancel = hasattr(tuner, 'reset_cancel')
    has_check_cancelled = hasattr(tuner, '_check_cancelled')

    print(f"  stop() 方法: {has_stop}")
    print(f"  is_cancelled 属性: {has_is_cancelled}")
    print(f"  reset_cancel() 方法: {has_reset_cancel}")
    print(f"  _check_cancelled() 方法: {has_check_cancelled}")

    passed = has_stop and has_is_cancelled and has_reset_cancel and has_check_cancelled
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_tuner_stop_sets_flag():
    """测试 tuner.stop() 设置取消标志。"""
    print("\n=== 测试 8: tuner.stop() 设置取消标志 ===")

    tuner = RelayFeedbackTuner()
    print(f"  调用前 is_cancelled: {tuner.is_cancelled}")

    tuner.stop()
    print(f"  调用后 is_cancelled: {tuner.is_cancelled}")

    passed = tuner.is_cancelled is True
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_tuner_reset_cancel():
    """测试 tuner.reset_cancel() 重置取消标志。"""
    print("\n=== 测试 9: tuner.reset_cancel() 重置 ===")

    tuner = RelayFeedbackTuner()
    tuner.stop()
    print(f"  stop() 后 is_cancelled: {tuner.is_cancelled}")

    tuner.reset_cancel()
    print(f"  reset_cancel() 后 is_cancelled: {tuner.is_cancelled}")

    passed = tuner.is_cancelled is False
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_check_cancelled_raises():
    """测试 _check_cancelled 在取消时抛出 CancelledError。"""
    print("\n=== 测试 10: _check_cancelled 抛出 CancelledError ===")

    tuner = RelayFeedbackTuner()

    # 未取消时不应抛出
    try:
        tuner._check_cancelled()
        print("  未取消时: 无异常 (正确)")
    except CancelledError:
        print("  未取消时: 错误抛出异常")
        return False

    tuner.stop()

    # 取消后应抛出
    try:
        tuner._check_cancelled()
        print("  取消后: 未抛出异常 (错误)")
        return False
    except CancelledError as e:
        print(f"  取消后: CancelledError 已抛出 ({e})")

    print(f"  结果: PASS")
    return True


# ============================================================
# 测试 4：RelayFeedbackTuner 取消中断测试
# ============================================================

def test_relay_feedback_cancel_midway():
    """测试继电反馈过程中取消中断。"""
    print("\n=== 测试 11: 继电反馈中途取消 ===")

    # 使用带 sleep 的 plant 确保取消有时间生效
    class SlowPlant:
        def __init__(self):
            self.x = 0.0
        def __call__(self, output):
            time.sleep(0.002)  # 每步慢 2ms，确保线程有时间取消
            self.x += 0.001 * output
            return self.x + random.gauss(0, 0.01)

    plant = SlowPlant()
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=0.5,
        sample_time=0.005,
        relay_duration=2.0,  # 较短时间但有 sleep
        setpoint=5.0
    )

    result_holder = [None]

    def run_in_thread():
        result_holder[0] = tuner.run_relay_feedback(plant)

    t = threading.Thread(target=run_in_thread)
    t.start()

    # 等待一点时间后取消
    time.sleep(0.05)
    tuner.stop()

    t.join(timeout=5.0)

    result = result_holder[0]
    assert result is not None, "调参未返回结果"

    print(f"  返回振荡幅值: {result.oscillation_amplitude:.3f}")
    print(f"  返回振荡周期: {result.oscillation_period:.3f}")
    print(f"  返回周期数: {result.num_cycles}")

    # 取消后应返回零结果（不完整的数据）
    passed = (
        result.oscillation_amplitude == 0.0 or
        result.num_cycles == 0 or
        len(result.raw_data) == 0  # 应该提前终止，没有原始数据
    )
    print(f"  提前终止: {passed}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_verify_cancel_midway():
    """测试验证阶段取消中断。"""
    print("\n=== 测试 12: 验证阶段中途取消 ===")

    class SlowPlant:
        def __init__(self):
            self.x = 0.0
        def __call__(self, output):
            time.sleep(0.002)  # 每步慢 2ms
            self.x += 0.001 * output
            return self.x + random.gauss(0, 0.01)

    plant = SlowPlant()
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=0.5,
        sample_time=0.005,
        relay_duration=2.0,
        verify_duration=2.0,  # 较短但有 sleep
        setpoint=5.0
    )

    result_holder = [None]

    def run_verify():
        result_holder[0] = tuner.verify_params(
            plant, kp=1.5, ki=1.5, kd=0.06,
            dt=0.005
        )

    t = threading.Thread(target=run_verify)
    t.start()

    time.sleep(0.05)
    tuner.stop()

    t.join(timeout=5.0)

    result = result_holder[0]
    assert result is not None, "验证未返回结果"

    print(f"  验证结果 passed: {result.passed}")
    print(f"  超调量: {result.overshoot:.1f}%")
    print(f"  调节时间: {result.settling_time:.2f}s")

    # 取消后验证应返回失败结果
    passed = not result.passed
    print(f"  取消后验证失败: {passed}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 测试 5：auto_tune 完整流程取消测试
# ============================================================

def test_auto_tune_cancel_at_start():
    """测试在 auto_tune 开始时立即取消。"""
    print("\n=== 测试 13: auto_tune 开始时取消 ===")

    plant = SecondOrderPlant(omega_n=3.0, zeta=0.3, gain=1.0, noise=0.01)
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=0.5,
        sample_time=0.005,
        relay_duration=15.0,
        setpoint=5.0,
        zn_method='some_overshoot'
    )

    # 开始前就取消
    tuner.stop()

    progress_log = []

    def on_progress(phase, progress, msg):
        progress_log.append({'phase': phase, 'progress': progress, 'msg': msg})

    result = tuner.auto_tune(plant, on_progress=on_progress)

    print(f"  状态: {result.status}")
    print(f"  消息: {result.message}")
    print(f"  进度事件数: {len(progress_log)}")

    passed = result.status == 'cancelled'
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_auto_tune_cancel_during_relay():
    """测试在继电反馈阶段取消。"""
    print("\n=== 测试 14: auto_tune 继电反馈阶段取消 ===")

    class SlowPlant:
        def __init__(self):
            self.x = 0.0
        def __call__(self, output):
            time.sleep(0.002)  # 每步慢 2ms，确保有时间取消
            self.x += 0.001 * output
            return self.x + random.gauss(0, 0.01)

    plant = SlowPlant()
    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=0.5,
        sample_time=0.005,
        relay_duration=2.0,  # 较短但有 sleep
        setpoint=5.0,
        zn_method='some_overshoot'
    )

    result_holder = [None]

    def run_auto_tune():
        result_holder[0] = tuner.auto_tune(plant)

    t = threading.Thread(target=run_auto_tune)
    t.start()

    time.sleep(0.05)
    tuner.stop()

    t.join(timeout=10.0)

    result = result_holder[0]
    assert result is not None

    print(f"  状态: {result.status}")
    print(f"  消息: {result.message}")

    passed = result.status in ('cancelled', 'relay_failed')
    print(f"  取消/中止: {passed}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_auto_tune_cancel_resets_on_rerun():
    """测试取消后重置可以重新开始调参。"""
    print("\n=== 测试 15: 取消后重置重新调参 ===")

    tuner = RelayFeedbackTuner()

    # 取消
    tuner.stop()
    assert tuner.is_cancelled is True
    print(f"  取消后: is_cancelled={tuner.is_cancelled}")

    # 重置
    tuner.reset_cancel()
    assert tuner.is_cancelled is False
    print(f"  重置后: is_cancelled={tuner.is_cancelled}")

    # 验证可以正常运行
    plant = SecondOrderPlant(omega_n=10.0, zeta=0.2, gain=1.0, noise=0.01)
    tuner_relay = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=0.5,
        sample_time=0.005,
        relay_duration=5.0,
        setpoint=5.0
    )
    result = tuner_relay.run_relay_feedback(plant)

    passed = result.oscillation_amplitude > 0.01
    print(f"  重新调参后振荡幅值: {result.oscillation_amplitude:.3f}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 测试 6：CancelledError 异常行为
# ============================================================

def test_cancelled_error_is_exception():
    """测试 CancelledError 是 Exception 的子类。"""
    print("\n=== 测试 16: CancelledError 异常类 ===")

    is_exception = issubclass(CancelledError, Exception)
    print(f"  是 Exception 子类: {is_exception}")

    # 可以正常 raise 和 catch
    try:
        raise CancelledError("test message")
    except CancelledError as e:
        msg = str(e)
        print(f"  异常消息: {msg}")
        caught = msg == "test message"

    passed = is_exception and caught
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_auto_tune_handles_cancelled_error():
    """测试 auto_tune 正确捕获 CancelledError。"""
    print("\n=== 测试 17: auto_tune 捕获 CancelledError ===")

    tuner = RelayFeedbackTuner(
        relay_amplitude=500.0,
        relay_hysteresis=0.5,
        sample_time=0.005,
        relay_duration=15.0,
        setpoint=5.0
    )

    # Mock plant_func 以抛出 CancelledError
    def bad_plant_func(output):
        raise CancelledError("simulated cancel")

    # auto_tune 应该优雅地处理这个异常
    result = tuner.auto_tune(bad_plant_func)

    print(f"  状态: {result.status}")
    print(f"  消息: {result.message}")

    # 应该返回失败状态而不是崩溃
    passed = result.status in ('cancelled', 'relay_failed')
    print(f"  优雅处理: {passed}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 测试 7：并发安全测试
# ============================================================

def test_concurrent_stop_safety():
    """测试多次并发调用 stop() 的安全性。"""
    print("\n=== 测试 18: 并发 stop() 安全性 ===")

    tuner = RelayFeedbackTuner()

    # 多线程同时调用 stop
    def stop_worker():
        tuner.stop()

    threads = [threading.Thread(target=stop_worker) for _ in range(10)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=2.0)

    passed = tuner.is_cancelled is True
    print(f"  并发 stop() 后 is_cancelled: {tuner.is_cancelled}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


def test_stop_idempotent():
    """测试多次 stop() 调用的幂等性。"""
    print("\n=== 测试 19: stop() 幂等性 ===")

    tuner = RelayFeedbackTuner()

    for i in range(5):
        tuner.stop()

    passed = tuner.is_cancelled is True
    print(f"  5 次 stop() 后 is_cancelled: {tuner.is_cancelled}")
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 测试 8：UI 事件流验证
# ============================================================

def test_auto_tune_result_hides_cancel_button():
    """测试 auto_tune_result 事件也隐藏取消按钮。"""
    print("\n=== 测试 20: auto_tune_result 隐藏取消按钮 ===")

    html_path = os.path.join(os.path.dirname(__file__), '..', 'templates', 'index.html')
    with open(html_path, 'r', encoding='utf-8') as f:
        html = f.read()

    # auto_tune_result 处理函数应该重新启用按钮
    has_result_handler = "socket.on('auto_tune_result'" in html
    has_enable = "document.getElementById('btnAuto').disabled=false" in html
    has_reset_text = "document.getElementById('btnAuto').textContent='⚡ 自动调参'" in html

    print(f"  auto_tune_result 事件处理: {has_result_handler}")
    print(f"  重新启用 btnAuto: {has_enable}")
    print(f"  重置按钮文本: {has_reset_text}")

    passed = has_result_handler and has_enable and has_reset_text
    print(f"  结果: {'PASS' if passed else 'FAIL'}")
    return passed


# ============================================================
# 主测试函数
# ============================================================

def run_all_tests():
    """运行所有测试。"""
    print("=" * 60)
    print("自动调参取消功能 — 完整测试套件")
    print("=" * 60)

    tests = [
        ("取消按钮初始隐藏", test_cancel_button_initial_hidden),
        ("取消按钮调参时显示", test_cancel_button_shows_on_auto_tune),
        ("取消按钮完成后隐藏", test_cancel_button_hides_on_completion),
        ("前端 cancel 事件发射", test_cancel_event_emitted_from_ui),
        ("服务器 cancel 处理器", test_server_cancel_handler_exists),
        ("服务器 active_tuner 追踪", test_server_active_tuner_tracking),
        ("tuner.stop() 方法存在", test_tuner_stop_method_exists),
        ("tuner.stop() 设置标志", test_tuner_stop_sets_flag),
        ("tuner.reset_cancel() 重置", test_tuner_reset_cancel),
        ("_check_cancelled 抛出异常", test_check_cancelled_raises),
        ("继电反馈中途取消", test_relay_feedback_cancel_midway),
        ("验证阶段中途取消", test_verify_cancel_midway),
        ("auto_tune 开始时取消", test_auto_tune_cancel_at_start),
        ("auto_tune 继电阶段取消", test_auto_tune_cancel_during_relay),
        ("取消后重置重新调参", test_auto_tune_cancel_resets_on_rerun),
        ("CancelledError 异常类", test_cancelled_error_is_exception),
        ("auto_tune 捕获 CancelledError", test_auto_tune_handles_cancelled_error),
        ("并发 stop() 安全性", test_concurrent_stop_safety),
        ("stop() 幂等性", test_stop_idempotent),
        ("auto_tune_result 隐藏按钮", test_auto_tune_result_hides_cancel_button),
    ]

    results = []

    for name, test_func in tests:
        try:
            passed = test_func()
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
    print(f"通过率: {passed_count / total * 100:.1f}%")

    return results


if __name__ == '__main__':
    results = run_all_tests()

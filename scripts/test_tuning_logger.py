#!/usr/bin/env python
"""
PID 调参日志记录模块 — 综合测试套件。

测试内容：
1. 日志格式（时间戳、级别、阶段、消息结构）
2. 日志级别（DEBUG, INFO, WARNING, ERROR, CRITICAL）
3. 日志内容完整性（继电反馈、Z-N 计算、验证、微调各阶段）
4. 文件输出（JSON Lines 和纯文本格式）
5. 查询和过滤功能
6. 与 RelayFeedbackTuner 集成测试
"""

import sys
import os
import re
import json
import time
import math
import tempfile
import random

# 将脚本目录加入路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from tuning_logger import TuningLogger, LogLevel, LogEntry
from real_auto_tune import (
    RelayFeedbackTuner,
    RelayFeedbackResult,
    ZieglerNicholsResult,
    VerificationResult,
    AutoTuneResult,
    OvershootFineTuner,
)
from plant_model import ServoPlant


# ============================================================
# 辅助类：模拟受控对象
# ============================================================

class SimpleSecondOrderPlant:
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


class ServoPlantWrapper:
    """包装 ServoPlant 为 callable 接口。"""

    def __init__(self, dt=0.005):
        self.plant = ServoPlant(dt=dt)

    def reset(self):
        self.plant.reset()

    def __call__(self, control_output):
        return self.plant.update(control_output)


# ============================================================
# 测试 1：日志格式
# ============================================================

def test_log_format_timestamp():
    """测试日志时间戳格式。"""
    print("\n=== 测试 1.1: 日志时间戳格式 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    logger.info('general', '测试消息')
    logger.close()

    entries = logger.get_entries()
    assert len(entries) >= 1

    # 时间戳格式: 2026-06-17 10:30:45.123
    ts_pattern = r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}$'
    for entry in entries:
        match = re.match(ts_pattern, entry.timestamp)
        assert match, f"时间戳格式错误: {entry.timestamp}"

    print(f"  日志条目数: {len(entries)}")
    print(f"  时间戳格式: YYYY-MM-DD HH:MM:SS.mmm")
    print(f"  示例: {entries[0].timestamp}")
    print("  结果: PASS")
    return True


def test_log_format_level():
    """测试日志级别格式。"""
    print("\n=== 测试 1.2: 日志级别格式 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    logger.debug('general', 'debug')
    logger.info('general', 'info')
    logger.warning('general', 'warning')
    logger.error('general', 'error')
    logger.critical('general', 'critical')
    logger.close()

    entries = logger.get_entries()
    # start_session 创建一条 INFO 日志，所以跳过第一条
    expected_levels = ['DEBUG', 'INFO', 'WARNING', 'ERROR', 'CRITICAL']
    actual_levels = [e.level for e in entries[1:]]  # 跳过 start_session

    for actual, expected in zip(actual_levels, expected_levels):
        assert actual == expected, f"级别错误: {actual} != {expected}"

    print(f"  总级别数: {len(entries)} (含 start_session)")
    for entry in entries:
        print(f"    {entry.level}")
    print("  结果: PASS")
    return True


def test_log_format_phase():
    """测试日志阶段标识。"""
    print("\n=== 测试 1.3: 日志阶段标识 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    phases = ['relay', 'calc', 'verify', 'finetune', 'serial', 'general']
    for phase in phases:
        logger.info(phase, f'阶段 {phase} 测试')

    logger.close()
    entries = logger.get_entries()

    # 去掉 start_session 的条目
    phase_entries = [e for e in entries if e.message.startswith('阶段')]

    for entry, expected in zip(phase_entries, phases):
        assert entry.phase == expected, f"阶段错误: {entry.phase} != {expected}"

    print(f"  阶段数: {len(phase_entries)}")
    for entry in phase_entries:
        print(f"    [{entry.phase}] {entry.message}")
    print("  结果: PASS")
    return True


def test_log_format_console_output():
    """测试控制台输出格式。"""
    print("\n=== 测试 1.4: 控制台输出格式 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    logger.info('relay', '测试控制台格式', {'key': 'value'})
    logger.close()

    entries = logger.get_entries()
    # entries[0] 是 start_session，entries[1] 是我们的日志
    target_entry = entries[1]
    console_line = target_entry.format_console()

    # 验证格式: timestamp [LEVEL] [phase] message {data}
    assert target_entry.timestamp in console_line
    assert '[INFO]' in console_line
    assert '[relay]' in console_line
    assert '测试控制台格式' in console_line
    assert '{"key": "value"}' in console_line

    print(f"  控制台输出: {console_line}")
    print("  结果: PASS")
    return True


def test_log_format_json_output():
    """测试 JSON 输出格式。"""
    print("\n=== 测试 1.5: JSON 输出格式 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    logger.info('relay', 'JSON 测试', {'amplitude': 3.5, 'period': 1.2})
    logger.close()

    entries = logger.get_entries()
    # entries[0] 是 start_session，entries[1] 是带 data 的日志
    target_entry = entries[1]
    json_str = target_entry.to_json()
    parsed = json.loads(json_str)

    assert 'timestamp' in parsed
    assert 'level' in parsed
    assert 'phase' in parsed
    assert 'message' in parsed
    assert 'data' in parsed
    assert parsed['data']['amplitude'] == 3.5
    assert parsed['data']['period'] == 1.2

    print(f"  JSON 输出: {json_str[:100]}...")
    print(f"  字段: {list(parsed.keys())}")
    print("  结果: PASS")
    return True


def test_log_format_elapsed_time():
    """测试经过时间记录。"""
    print("\n=== 测试 1.6: 经过时间记录 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    time.sleep(0.05)
    logger.info('general', '第一条日志')
    time.sleep(0.05)
    logger.info('general', '第二条日志')
    logger.close()

    entries = logger.get_entries()
    session_entries = [e for e in entries if e.elapsed is not None]

    assert len(session_entries) >= 2
    assert session_entries[1].elapsed > session_entries[0].elapsed

    print(f"  第一条经过时间: {session_entries[0].elapsed:.4f}s")
    print(f"  第二条经过时间: {session_entries[1].elapsed:.4f}s")
    print("  结果: PASS")
    return True


# ============================================================
# 测试 2：日志级别
# ============================================================

def test_log_level_filtering():
    """测试日志级别过滤。"""
    print("\n=== 测试 2.1: 日志级别过滤 ===")

    # 只记录 INFO 及以上
    logger = TuningLogger(console_output=False, min_level=LogLevel.INFO)
    logger.start_session()
    logger.debug('general', '这条不应该出现')
    logger.info('general', '这条应该出现')
    logger.warning('general', '这条也应该出现')
    logger.close()

    entries = logger.get_entries()
    debug_entries = [e for e in entries if e.level == 'DEBUG']

    # start_session 产生的 INFO 条目 + 手动的 INFO 和 WARNING
    info_plus = [e for e in entries if e.level in ('INFO', 'WARNING', 'ERROR', 'CRITICAL')]

    assert len(debug_entries) == 0, "DEBUG 条目不应出现"
    assert len(info_plus) >= 3, f"INFO+ 条目不足: {len(info_plus)}"

    print(f"  总条目: {len(entries)}")
    print(f"  DEBUG 条目: {len(debug_entries)} (应为 0)")
    print(f"  INFO+ 条目: {len(info_plus)} (应 >= 3)")
    print("  结果: PASS")
    return True


def test_log_level_all_levels():
    """测试所有日志级别。"""
    print("\n=== 测试 2.2: 所有日志级别 ===")

    logger = TuningLogger(console_output=False, min_level=LogLevel.DEBUG)
    logger.start_session()
    logger.debug('general', 'debug 消息')
    logger.info('general', 'info 消息')
    logger.warning('general', 'warning 消息')
    logger.error('general', 'error 消息')
    logger.critical('general', 'critical 消息')
    logger.close()

    level_counts = logger.get_level_counts()

    assert 'DEBUG' in level_counts
    assert 'INFO' in level_counts
    assert 'WARNING' in level_counts
    assert 'ERROR' in level_counts
    assert 'CRITICAL' in level_counts

    # 排除 start_session 的条目
    debug_count = level_counts.get('DEBUG', 0)
    info_count = level_counts.get('INFO', 0) - 1  # 减去 start_session

    assert debug_count >= 1
    assert info_count >= 1
    assert level_counts.get('WARNING', 0) >= 1
    assert level_counts.get('ERROR', 0) >= 1
    assert level_counts.get('CRITICAL', 0) >= 1

    print(f"  级别分布: {level_counts}")
    print("  结果: PASS")
    return True


def test_log_level_numeric_values():
    """测试日志级别数值。"""
    print("\n=== 测试 2.3: 日志级别数值 ===")

    assert LogLevel.DEBUG == 10
    assert LogLevel.INFO == 20
    assert LogLevel.WARNING == 30
    assert LogLevel.ERROR == 40
    assert LogLevel.CRITICAL == 50

    print("  DEBUG=10, INFO=20, WARNING=30, ERROR=40, CRITICAL=50")
    print("  结果: PASS")
    return True


# ============================================================
# 测试 3：日志内容完整性
# ============================================================

def test_relay_phase_logging():
    """测试继电反馈阶段日志记录。"""
    print("\n=== 测试 3.1: 继电反馈阶段日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    # 模拟继电反馈流程
    logger.log_relay_start(amplitude=500.0, hysteresis=1.0, duration=15.0)

    # 模拟结果
    relay_result = RelayFeedbackResult(
        oscillation_amplitude=3.21,
        oscillation_period=1.45,
        relay_amplitude=500.0,
        mean_value=4.89,
        num_cycles=5,
        raw_data=[(0.0, 1.0, 500.0)] * 100
    )
    logger.log_relay_result(relay_result)
    logger.close()

    # 验证
    relay_entries = logger.get_entries_by_phase('relay')
    start_entries = logger.get_entries_by_event('relay_start')
    complete_entries = logger.get_entries_by_event('relay_complete')

    # relay 阶段有 2 条日志（start + complete），start_session 是 general 阶段
    assert len(relay_entries) >= 2
    assert len(start_entries) >= 1
    assert len(complete_entries) >= 1

    # 验证 start 事件数据
    start_data = start_entries[0].data
    assert start_data['relay_amplitude'] == 500.0
    assert start_data['relay_hysteresis'] == 1.0
    assert start_data['relay_duration'] == 15.0

    # 验证 complete 事件数据
    complete_data = complete_entries[0].data
    assert complete_data['oscillation_amplitude'] == 3.21
    assert complete_data['oscillation_period'] == 1.45
    assert complete_data['num_cycles'] == 5
    assert complete_data['raw_data_points'] == 100

    print(f"  继电反馈日志条目: {len(relay_entries)}")
    print(f"  start 事件: {len(start_entries)}")
    print(f"  complete 事件: {len(complete_entries)}")
    print(f"  start 数据: {start_data}")
    print(f"  complete 数据: {complete_data}")
    print("  结果: PASS")
    return True


def test_zn_calculation_logging():
    """测试 Ziegler-Nichols 计算阶段日志。"""
    print("\n=== 测试 3.2: Ziegler-Nichols 计算日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    zn_result = ZieglerNicholsResult(
        kp=5.23, ki=3.14, kd=0.89,
        ku=12.5, tu=1.8,
        method='some_overshoot'
    )
    logger.log_zn_calculation(zn_result)
    logger.close()

    calc_entries = logger.get_entries_by_phase('calc')
    calc_events = logger.get_entries_by_event('zn_calculated')

    assert len(calc_events) >= 1

    data = calc_events[0].data
    assert data['kp'] == 5.23
    assert data['ki'] == 3.14
    assert data['kd'] == 0.89
    assert data['ku'] == 12.5
    assert data['tu'] == 1.8
    assert data['method'] == 'some_overshoot'

    print(f"  calc 日志条目: {len(calc_entries)}")
    print(f"  zn_calculated 事件: {len(calc_events)}")
    print(f"  数据: Kp={data['kp']}, Ki={data['ki']}, Kd={data['kd']}")
    print("  结果: PASS")
    return True


def test_verification_phase_logging():
    """测试验证阶段日志。"""
    print("\n=== 测试 3.3: 验证阶段日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    # 验证开始
    logger.log_verify_start(kp=5.23, ki=3.14, kd=0.89)

    # 验证结果（通过）
    verify_result_pass = VerificationResult(
        overshoot=12.5, settling_time=2.3,
        rise_time=0.8, steady_state_error=0.05,
        final_value=4.95, passed=True
    )
    logger.log_verification(verify_result_pass)

    # 验证结果（未通过）
    verify_result_fail = VerificationResult(
        overshoot=35.0, settling_time=8.5,
        rise_time=0.5, steady_state_error=0.8,
        final_value=4.2, passed=False
    )
    logger.log_verification(verify_result_fail)

    logger.close()

    verify_start_events = logger.get_entries_by_event('verify_start')
    verify_complete_events = logger.get_entries_by_event('verify_complete')

    assert len(verify_start_events) >= 1
    assert len(verify_complete_events) >= 2

    # 验证通过的日志级别应为 INFO
    pass_entry = verify_complete_events[0]
    assert pass_entry.level == 'INFO'
    assert pass_entry.data['passed'] is True

    # 验证未通过的日志级别应为 WARNING
    fail_entry = verify_complete_events[1]
    assert fail_entry.level == 'WARNING'
    assert fail_entry.data['passed'] is False

    print(f"  verify_start 事件: {len(verify_start_events)}")
    print(f"  verify_complete 事件: {len(verify_complete_events)}")
    print(f"  通过验证级别: {pass_entry.level} (应为 INFO)")
    print(f"  未通过验证级别: {fail_entry.level} (应为 WARNING)")
    print("  结果: PASS")
    return True


def test_finetune_phase_logging():
    """测试超调微调阶段日志。"""
    print("\n=== 测试 3.4: 超调微调阶段日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    # 微调触发
    logger.log_finetune_trigger(overshoot=28.5, threshold=20.0, round_num=1)

    # 参数调整
    logger.log_finetune_adjust(
        kp_before=5.23, ki_before=3.14, kd_before=0.89,
        kp_after=4.18, ki_after=3.14, kd_after=1.07,
        kp_factor=0.8, kd_factor=1.2
    )

    # 微调结果
    logger.log_finetune_result(
        round_num=1,
        overshoot_before=28.5,
        overshoot_after=15.2,
        settling_time=2.1
    )

    logger.close()

    trigger_events = logger.get_entries_by_event('finetune_trigger')
    adjust_events = logger.get_entries_by_event('finetune_adjust')
    complete_events = logger.get_entries_by_event('finetune_complete')

    assert len(trigger_events) >= 1
    assert len(adjust_events) >= 1
    assert len(complete_events) >= 1

    # 验证触发日志
    trigger_data = trigger_events[0].data
    assert trigger_data['overshoot'] == 28.5
    assert trigger_data['threshold'] == 20.0
    assert trigger_data['round'] == 1

    # 验证调整日志
    adjust_data = adjust_events[0].data
    assert adjust_data['kp_before'] == 5.23
    assert adjust_data['kp_after'] == 4.18
    assert adjust_data['kp_factor'] == 0.8
    assert adjust_data['kd_factor'] == 1.2

    # 验证结果日志
    complete_data = complete_events[0].data
    assert complete_data['overshoot_before'] == 28.5
    assert complete_data['overshoot_after'] == 15.2
    assert complete_data['settling_time_after'] == 2.1

    print(f"  finetune_trigger 事件: {len(trigger_events)}")
    print(f"  finetune_adjust 事件: {len(adjust_events)}")
    print(f"  finetune_complete 事件: {len(complete_events)}")
    print("  结果: PASS")
    return True


def test_serial_phase_logging():
    """测试串口阶段日志。"""
    print("\n=== 测试 3.5: 串口阶段日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    logger.log_serial_connect('COM3', 115200)
    logger.log_serial_error('设备未响应')
    logger.log_serial_disconnect()
    logger.close()

    serial_entries = logger.get_entries_by_phase('serial')
    connect_events = logger.get_entries_by_event('serial_connect')
    error_events = logger.get_entries_by_event('serial_error')
    disconnect_events = logger.get_entries_by_event('serial_disconnect')

    assert len(connect_events) >= 1
    assert len(error_events) >= 1
    assert len(disconnect_events) >= 1

    # 验证连接数据
    connect_data = connect_events[0].data
    assert connect_data['port'] == 'COM3'
    assert connect_data['baud'] == 115200

    # 验证错误日志级别
    assert error_events[0].level == 'ERROR'

    print(f"  serial 日志条目: {len(serial_entries)}")
    print(f"  connect 事件: {len(connect_events)}")
    print(f"  error 事件: {len(error_events)}")
    print(f"  disconnect 事件: {len(disconnect_events)}")
    print("  结果: PASS")
    return True


def test_session_summary_logging():
    """测试会话总结日志。"""
    print("\n=== 测试 3.6: 会话总结日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    # 模拟完整调参结果
    relay_result = RelayFeedbackResult(
        oscillation_amplitude=3.21,
        oscillation_period=1.45,
        relay_amplitude=500.0,
        mean_value=4.89,
        num_cycles=5
    )
    zn_result = ZieglerNicholsResult(
        kp=5.23, ki=3.14, kd=0.89,
        ku=12.5, tu=1.8,
        method='some_overshoot'
    )
    verify_result = VerificationResult(
        overshoot=12.5, settling_time=2.3,
        rise_time=0.8, steady_state_error=0.05,
        final_value=4.95, passed=True
    )
    auto_tune_result = AutoTuneResult(
        relay_result=relay_result,
        zn_params=zn_result,
        verification=verify_result,
        status='success',
        message='调参成功: Kp=5.23, Ki=3.14, Kd=0.89',
        recommended_params={'kp': 5.23, 'ki': 3.14, 'kd': 0.89},
        finetune_history=[]
    )

    logger.log_session_summary(auto_tune_result)
    logger.close()

    summary_events = logger.get_entries_by_event('session_end')
    assert len(summary_events) >= 1

    data = summary_events[0].data
    assert data['status'] == 'success'
    assert data['recommended_params']['kp'] == 5.23
    assert data['finetune_rounds'] == 0

    print(f"  session_end 事件: {len(summary_events)}")
    print(f"  数据: status={data['status']}, params={data['recommended_params']}")
    print("  结果: PASS")
    return True


# ============================================================
# 测试 4：文件输出
# ============================================================

def test_file_output_plaintext():
    """测试纯文本文件输出。"""
    print("\n=== 测试 4.1: 纯文本文件输出 ===")

    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False) as f:
        log_path = f.name

    try:
        logger = TuningLogger(log_file=log_path, console_output=False)
        logger.start_session()
        logger.info('relay', '继电反馈测试')
        logger.warning('verify', '超调过高')
        logger.close()

        with open(log_path, 'r', encoding='utf-8') as f:
            content = f.read()

        lines = [l for l in content.strip().split('\n') if l]
        assert len(lines) >= 3  # start_session + info + warning

        # 验证每行格式
        for line in lines:
            assert re.match(r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3} \[\w+\] \[\w+\]', line)

        print(f"  文件路径: {log_path}")
        print(f"  行数: {len(lines)}")
        print(f"  示例行: {lines[0][:80]}...")
        print("  结果: PASS")
        return True
    finally:
        if os.path.exists(log_path):
            os.remove(log_path)


def test_file_output_json():
    """测试 JSON 文件输出。"""
    print("\n=== 测试 4.2: JSON 文件输出 ===")

    with tempfile.NamedTemporaryFile(mode='w', suffix='.jsonl', delete=False) as f:
        log_path = f.name

    try:
        logger = TuningLogger(log_file=log_path, console_output=False, json_format=True)
        logger.start_session()
        logger.info('relay', 'JSON 测试', {'amplitude': 3.5})
        logger.close()

        with open(log_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        valid_json = [l for l in lines if l.strip()]
        assert len(valid_json) >= 2

        for line in valid_json:
            parsed = json.loads(line)
            assert 'timestamp' in parsed
            assert 'level' in parsed
            assert 'phase' in parsed
            assert 'message' in parsed

        print(f"  文件路径: {log_path}")
        print(f"  JSON 行数: {len(valid_json)}")
        parsed_first = json.loads(valid_json[0])
        print(f"  示例: {json.dumps(parsed_first, ensure_ascii=False)[:80]}...")
        print("  结果: PASS")
        return True
    finally:
        if os.path.exists(log_path):
            os.remove(log_path)


# ============================================================
# 测试 5：查询和过滤
# ============================================================

def test_query_by_level():
    """测试按级别查询日志。"""
    print("\n=== 测试 5.1: 按级别查询日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    logger.debug('general', 'd1')
    logger.info('general', 'i1')
    logger.info('general', 'i2')
    logger.warning('general', 'w1')
    logger.error('general', 'e1')
    logger.close()

    debug_entries = logger.get_entries_by_level(LogLevel.DEBUG)
    info_entries = logger.get_entries_by_level(LogLevel.INFO)
    warning_entries = logger.get_entries_by_level(LogLevel.WARNING)
    error_entries = logger.get_entries_by_level(LogLevel.ERROR)

    assert len(debug_entries) >= 1
    assert len(info_entries) >= 3  # start_session + i1 + i2
    assert len(warning_entries) >= 1
    assert len(error_entries) >= 1

    print(f"  DEBUG: {len(debug_entries)}, INFO: {len(info_entries)}, "
          f"WARNING: {len(warning_entries)}, ERROR: {len(error_entries)}")
    print("  结果: PASS")
    return True


def test_query_by_phase():
    """测试按阶段查询日志。"""
    print("\n=== 测试 5.2: 按阶段查询日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    logger.info('relay', 'r1')
    logger.info('relay', 'r2')
    logger.info('calc', 'c1')
    logger.info('verify', 'v1')
    logger.info('verify', 'v2')
    logger.info('verify', 'v3')
    logger.close()

    relay_entries = logger.get_entries_by_phase('relay')
    calc_entries = logger.get_entries_by_phase('calc')
    verify_entries = logger.get_entries_by_phase('verify')

    # start_session 是 general 阶段，不计入 relay
    assert len(relay_entries) >= 2  # r1 + r2
    assert len(calc_entries) >= 1
    assert len(verify_entries) >= 3

    print(f"  relay: {len(relay_entries)}, calc: {len(calc_entries)}, verify: {len(verify_entries)}")
    print("  结果: PASS")
    return True


def test_query_by_event():
    """测试按事件类型查询日志。"""
    print("\n=== 测试 5.3: 按事件查询日志 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    logger.log_relay_start(500.0, 1.0, 15.0)
    logger.log_relay_result(RelayFeedbackResult(
        oscillation_amplitude=3.0, oscillation_period=1.5,
        relay_amplitude=500.0, mean_value=5.0, num_cycles=5
    ))
    logger.log_zn_calculation(ZieglerNicholsResult(
        kp=5.0, ki=3.0, kd=0.8, ku=12.0, tu=1.5, method='classic'
    ))

    logger.close()

    start_events = logger.get_entries_by_event('relay_start')
    complete_events = logger.get_entries_by_event('relay_complete')
    zn_events = logger.get_entries_by_event('zn_calculated')

    assert len(start_events) >= 1
    assert len(complete_events) >= 1
    assert len(zn_events) >= 1

    print(f"  relay_start: {len(start_events)}")
    print(f"  relay_complete: {len(complete_events)}")
    print(f"  zn_calculated: {len(zn_events)}")
    print("  结果: PASS")
    return True


def test_export_json():
    """测试 JSON 导出功能。"""
    print("\n=== 测试 5.4: JSON 导出 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()
    logger.info('relay', '导出测试', {'key': 'value'})
    logger.close()

    json_str = logger.export_json()
    parsed = json.loads(json_str)

    assert isinstance(parsed, list)
    assert len(parsed) >= 2  # start_session + info

    # 验证每条记录格式
    for record in parsed:
        assert 'timestamp' in record
        assert 'level' in record
        assert 'phase' in record
        assert 'message' in record

    print(f"  导出记录数: {len(parsed)}")
    print(f"  JSON 长度: {len(json_str)} 字符")
    print("  结果: PASS")
    return True


# ============================================================
# 测试 6：与 RelayFeedbackTuner 集成
# ============================================================

def test_integration_with_tuner():
    """测试与 RelayFeedbackTuner 的完整集成。"""
    print("\n=== 测试 6.1: 与 RelayFeedbackTuner 完整集成 ===")

    with tempfile.NamedTemporaryFile(mode='w', suffix='.log', delete=False) as f:
        log_path = f.name

    try:
        logger = TuningLogger(log_file=log_path, console_output=False)
        logger.start_session()

        plant = SimpleSecondOrderPlant(omega_n=3.0, zeta=0.3, gain=1.0, noise=0.01)
        tuner = RelayFeedbackTuner(
            relay_amplitude=500.0,
            relay_hysteresis=0.5,
            sample_time=0.005,
            relay_duration=10.0,
            min_cycles=3,
            min_switch_time=0.02,
            verify_duration=5.0,
            setpoint=5.0,
            zn_method='some_overshoot'
        )

        # 手动记录各阶段
        logger.log_relay_start(500.0, 0.5, 10.0)
        relay_result = tuner.run_relay_feedback(plant)
        logger.log_relay_result(relay_result)

        zn_result = tuner.calculate_ziegler_nichols(relay_result)
        logger.log_zn_calculation(zn_result)

        logger.log_verify_start(zn_result.kp, zn_result.ki, zn_result.kd)
        verify_result = tuner.verify_params(plant, zn_result.kp, zn_result.ki, zn_result.kd)
        logger.log_verification(verify_result)

        # 构建 AutoTuneResult
        auto_result = AutoTuneResult(
            relay_result=relay_result,
            zn_params=zn_result,
            verification=verify_result,
            status='success' if verify_result.passed else 'verification_failed',
            message='测试调参',
            recommended_params={
                'kp': round(zn_result.kp, 4),
                'ki': round(zn_result.ki, 4),
                'kd': round(zn_result.kd, 4),
            }
        )
        logger.log_session_summary(auto_result)
        logger.close()

        # 验证日志完整性
        entries = logger.get_entries()
        phase_counts = logger.get_phase_counts()

        assert 'relay' in phase_counts
        assert 'calc' in phase_counts
        assert 'verify' in phase_counts
        assert 'general' in phase_counts

        assert len(logger.get_entries_by_event('relay_start')) >= 1
        assert len(logger.get_entries_by_event('relay_complete')) >= 1
        assert len(logger.get_entries_by_event('zn_calculated')) >= 1
        assert len(logger.get_entries_by_event('verify_complete')) >= 1
        assert len(logger.get_entries_by_event('session_end')) >= 1

        # 验证文件输出
        with open(log_path, 'r', encoding='utf-8') as f:
            file_content = f.read()
        file_lines = [l for l in file_content.strip().split('\n') if l]
        assert len(file_lines) >= len(entries)

        print(f"  总日志条目: {len(entries)}")
        print(f"  阶段分布: {phase_counts}")
        print(f"  文件行数: {len(file_lines)}")
        print(f"  推荐参数: {auto_result.recommended_params}")
        print("  结果: PASS")
        return True
    finally:
        if os.path.exists(log_path):
            os.remove(log_path)


def test_integration_with_finetune():
    """测试包含超调微调的完整集成。"""
    print("\n=== 测试 6.2: 包含超调微调的集成 ===")

    logger = TuningLogger(console_output=False)
    logger.start_session()

    # 模拟多轮微调
    for round_num in range(1, 4):
        overshoot_before = 30.0 - (round_num - 1) * 8.0
        overshoot_after = overshoot_before - 8.0

        logger.log_finetune_trigger(overshoot_before, 20.0, round_num)
        logger.log_finetune_adjust(
            kp_before=5.0 * (0.8 ** (round_num - 1)),
            ki_before=3.0,
            kd_before=0.8 * (1.2 ** (round_num - 1)),
            kp_after=5.0 * (0.8 ** round_num),
            ki_after=3.0,
            kd_after=0.8 * (1.2 ** round_num),
            kp_factor=0.8,
            kd_factor=1.2
        )
        logger.log_finetune_result(round_num, overshoot_before, overshoot_after, 2.0)

    logger.close()

    trigger_events = logger.get_entries_by_event('finetune_trigger')
    adjust_events = logger.get_entries_by_event('finetune_adjust')
    complete_events = logger.get_entries_by_event('finetune_complete')

    assert len(trigger_events) == 3
    assert len(adjust_events) == 3
    assert len(complete_events) == 3

    print(f"  finetune_trigger: {len(trigger_events)}")
    print(f"  finetune_adjust: {len(adjust_events)}")
    print(f"  finetune_complete: {len(complete_events)}")
    print("  结果: PASS")
    return True


# ============================================================
# 主测试函数
# ============================================================

def run_all_tests():
    """运行所有测试。"""
    print("=" * 70)
    print("PID 调参日志记录模块 — 综合测试")
    print("=" * 70)

    tests = [
        # 日志格式
        ("时间戳格式", test_log_format_timestamp),
        ("级别格式", test_log_format_level),
        ("阶段标识", test_log_format_phase),
        ("控制台输出格式", test_log_format_console_output),
        ("JSON 输出格式", test_log_format_json_output),
        ("经过时间记录", test_log_format_elapsed_time),

        # 日志级别
        ("级别过滤", test_log_level_filtering),
        ("所有级别", test_log_level_all_levels),
        ("级别数值", test_log_level_numeric_values),

        # 日志内容完整性
        ("继电反馈阶段", test_relay_phase_logging),
        ("Ziegler-Nichols 计算", test_zn_calculation_logging),
        ("验证阶段", test_verification_phase_logging),
        ("超调微调阶段", test_finetune_phase_logging),
        ("串口阶段", test_serial_phase_logging),
        ("会话总结", test_session_summary_logging),

        # 文件输出
        ("纯文本文件", test_file_output_plaintext),
        ("JSON 文件", test_file_output_json),

        # 查询和过滤
        ("按级别查询", test_query_by_level),
        ("按阶段查询", test_query_by_phase),
        ("按事件查询", test_query_by_event),
        ("JSON 导出", test_export_json),

        # 集成测试
        ("与 RelayFeedbackTuner 集成", test_integration_with_tuner),
        ("包含超调微调的集成", test_integration_with_finetune),
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
    print("\n" + "=" * 70)
    print("测试汇总")
    print("=" * 70)
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

#!/usr/bin/env python
"""
PID 调参日志记录模块。

为实物自动调参过程提供结构化日志记录，包括：
- 时间戳
- 日志级别（DEBUG, INFO, WARNING, ERROR, CRITICAL）
- 阶段标识（relay, calc, verify, finetune）
- 结构化数据（JSON 格式可选）
- 文件输出和控制台输出

日志格式：
    2026-06-17 10:30:45.123 [INFO] [relay] 继电反馈振荡测试开始
    2026-06-17 10:30:50.456 [DEBUG] [relay] 振荡数据: amplitude=3.21, period=1.45s
    2026-06-17 10:30:50.789 [INFO] [calc] Ziegler-Nichols 参数计算: Kp=5.23, Ki=3.14, Kd=0.89
    2026-06-17 10:30:55.123 [WARNING] [verify] 超调量 28.5% 超过阈值 20%，触发微调
    2026-06-17 10:30:56.000 [ERROR] [serial] 串口连接失败: COM3 不存在
    2026-06-17 10:31:00.123 [CRITICAL] [relay] 继电反馈失败: 振荡不明显
"""

import json
import time
import os
import sys
from datetime import datetime
from enum import IntEnum
from typing import Optional, Dict, Any, List, Callable
from dataclasses import dataclass, field, asdict


class LogLevel(IntEnum):
    """日志级别。"""
    DEBUG = 10
    INFO = 20
    WARNING = 30
    ERROR = 40
    CRITICAL = 50


@dataclass
class LogEntry:
    """单条日志记录。"""
    timestamp: str
    level: str
    phase: str
    message: str
    data: Optional[Dict[str, Any]] = None
    elapsed: Optional[float] = None  # 自调参开始的经过时间（秒）

    def to_dict(self) -> dict:
        """转为字典。"""
        d = {
            'timestamp': self.timestamp,
            'level': self.level,
            'phase': self.phase,
            'message': self.message,
        }
        if self.data is not None:
            d['data'] = self.data
        if self.elapsed is not None:
            d['elapsed'] = round(self.elapsed, 4)
        return d

    def to_json(self) -> str:
        """转为 JSON 字符串。"""
        return json.dumps(self.to_dict(), ensure_ascii=False)

    def format_console(self) -> str:
        """格式化为控制台输出。"""
        parts = [self.timestamp, f'[{self.level}]', f'[{self.phase}]', self.message]
        if self.data:
            parts.append(json.dumps(self.data, ensure_ascii=False))
        return ' '.join(parts)


class TuningLogger:
    """PID 调参过程日志记录器。

    功能：
    - 多级别日志（DEBUG, INFO, WARNING, ERROR, CRITICAL）
    - 阶段标记（relay, calc, verify, finetune, serial, general）
    - 文件输出（JSON Lines 格式）
    - 控制台输出
    - 结构化数据记录
    - 经过时间追踪

    使用方法：
        logger = TuningLogger(log_file='tuning.log')
        logger.info('relay', '继电反馈开始')
        logger.log_relay_result(result)
        logger.log_zn_params(zn_result)
        logger.log_verification(verify_result)
        entries = logger.get_entries()
    """

    # 阶段标识常量
    PHASE_RELAY = 'relay'
    PHASE_CALC = 'calc'
    PHASE_VERIFY = 'verify'
    PHASE_FINETUNE = 'finetune'
    PHASE_SERIAL = 'serial'
    PHASE_GENERAL = 'general'

    def __init__(self,
                 log_file: Optional[str] = None,
                 console_output: bool = True,
                 min_level: LogLevel = LogLevel.DEBUG,
                 json_format: bool = False):
        """
        Args:
            log_file: 日志文件路径（None 表示不写文件）
            console_output: 是否输出到控制台
            min_level: 最低日志级别
            json_format: 文件输出是否使用 JSON 格式
        """
        self.log_file = log_file
        self.console_output = console_output
        self.min_level = min_level
        self.json_format = json_format
        self._entries: List[LogEntry] = []
        self._start_time: Optional[float] = None
        self._file_handle = None

        # 打开日志文件
        if log_file:
            os.makedirs(os.path.dirname(log_file) or '.', exist_ok=True)
            self._file_handle = open(log_file, 'w', encoding='utf-8')

    def start_session(self):
        """开始新的调参会话。"""
        self._start_time = time.time()
        self._entries.clear()
        self.info(self.PHASE_GENERAL, '=== 调参会话开始 ===')

    def _elapsed(self) -> Optional[float]:
        """返回自会话开始的经过时间。"""
        if self._start_time is not None:
            return time.time() - self._start_time
        return None

    def _log(self, level: LogLevel, phase: str, message: str, data: Optional[Dict] = None):
        """记录一条日志。"""
        if level < self.min_level:
            return

        entry = LogEntry(
            timestamp=datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
            level=level.name,
            phase=phase,
            message=message,
            data=data,
            elapsed=self._elapsed()
        )
        self._entries.append(entry)

        # 控制台输出
        if self.console_output:
            print(entry.format_console())

        # 文件输出
        if self._file_handle:
            if self.json_format:
                self._file_handle.write(entry.to_json() + '\n')
            else:
                self._file_handle.write(entry.format_console() + '\n')
            self._file_handle.flush()

    def debug(self, phase: str, message: str, data: Optional[Dict] = None):
        """记录 DEBUG 级别日志。"""
        self._log(LogLevel.DEBUG, phase, message, data)

    def info(self, phase: str, message: str, data: Optional[Dict] = None):
        """记录 INFO 级别日志。"""
        self._log(LogLevel.INFO, phase, message, data)

    def warning(self, phase: str, message: str, data: Optional[Dict] = None):
        """记录 WARNING 级别日志。"""
        self._log(LogLevel.WARNING, phase, message, data)

    def error(self, phase: str, message: str, data: Optional[Dict] = None):
        """记录 ERROR 级别日志。"""
        self._log(LogLevel.ERROR, phase, message, data)

    def critical(self, phase: str, message: str, data: Optional[Dict] = None):
        """记录 CRITICAL 级别日志。"""
        self._log(LogLevel.CRITICAL, phase, message, data)

    # ========== 结构化日志方法 ==========

    def log_relay_start(self, amplitude: float, hysteresis: float, duration: float):
        """记录继电反馈阶段开始。"""
        self.info(self.PHASE_RELAY, '继电反馈振荡测试开始', {
            'relay_amplitude': amplitude,
            'relay_hysteresis': hysteresis,
            'relay_duration': duration,
            'event': 'relay_start'
        })

    def log_relay_result(self, result):
        """记录继电反馈阶段结果。"""
        self.info(self.PHASE_RELAY, '继电反馈振荡测试完成', {
            'oscillation_amplitude': round(result.oscillation_amplitude, 4),
            'oscillation_period': round(result.oscillation_period, 4),
            'relay_amplitude': result.relay_amplitude,
            'mean_value': round(result.mean_value, 4),
            'num_cycles': result.num_cycles,
            'raw_data_points': len(result.raw_data),
            'event': 'relay_complete'
        })

    def log_relay_insufficient(self, result):
        """记录继电反馈振荡不足。"""
        self.warning(self.PHASE_RELAY, '继电反馈振荡不明显', {
            'oscillation_amplitude': round(result.oscillation_amplitude, 4),
            'oscillation_period': round(result.oscillation_period, 4),
            'num_cycles': result.num_cycles,
            'event': 'relay_insufficient'
        })

    def log_relay_failed(self, reason: str):
        """记录继电反馈失败。"""
        self.error(self.PHASE_RELAY, f'继电反馈失败: {reason}', {
            'event': 'relay_failed',
            'reason': reason
        })

    def log_zn_calculation(self, zn_result):
        """记录 Ziegler-Nichols 参数计算结果。"""
        self.info(self.PHASE_CALC, 'Ziegler-Nichols 参数计算完成', {
            'kp': round(zn_result.kp, 4),
            'ki': round(zn_result.ki, 4),
            'kd': round(zn_result.kd, 4),
            'ku': round(zn_result.ku, 4),
            'tu': round(zn_result.tu, 4),
            'method': zn_result.method,
            'event': 'zn_calculated'
        })

    def log_zn_edge_case(self, case: str, zn_result):
        """记录 Z-N 边界情况处理。"""
        self.warning(self.PHASE_CALC, f'Z-N 边界情况: {case}', {
            'kp': round(zn_result.kp, 4),
            'ki': round(zn_result.ki, 4),
            'kd': round(zn_result.kd, 4),
            'case': case,
            'event': 'zn_edge_case'
        })

    def log_verify_start(self, kp: float, ki: float, kd: float):
        """记录验证阶段开始。"""
        self.info(self.PHASE_VERIFY, 'PID 参数验证开始', {
            'kp': round(kp, 4),
            'ki': round(ki, 4),
            'kd': round(kd, 4),
            'event': 'verify_start'
        })

    def log_verification(self, verify_result):
        """记录验证阶段结果。"""
        level = LogLevel.INFO if verify_result.passed else LogLevel.WARNING
        self._log(level, self.PHASE_VERIFY, 'PID 参数验证完成', {
            'overshoot': verify_result.overshoot,
            'settling_time': verify_result.settling_time,
            'rise_time': verify_result.rise_time,
            'steady_state_error': verify_result.steady_state_error,
            'final_value': verify_result.final_value,
            'passed': verify_result.passed,
            'event': 'verify_complete'
        })

    def log_finetune_trigger(self, overshoot: float, threshold: float, round_num: int):
        """记录超调微调触发。"""
        self.warning(self.PHASE_FINETUNE,
                     f'超调 {overshoot:.1f}% 超过阈值 {threshold}%，触发第 {round_num} 轮微调',
                     {
                         'overshoot': overshoot,
                         'threshold': threshold,
                         'round': round_num,
                         'event': 'finetune_trigger'
                     })

    def log_finetune_adjust(self, kp_before: float, ki_before: float, kd_before: float,
                            kp_after: float, ki_after: float, kd_after: float,
                            kp_factor: float, kd_factor: float):
        """记录微调参数调整。"""
        self.info(self.PHASE_FINETUNE, '微调参数调整', {
            'kp_before': round(kp_before, 4),
            'ki_before': round(ki_before, 4),
            'kd_before': round(kd_before, 4),
            'kp_after': round(kp_after, 4),
            'ki_after': round(ki_after, 4),
            'kd_after': round(kd_after, 4),
            'kp_factor': kp_factor,
            'kd_factor': kd_factor,
            'event': 'finetune_adjust'
        })

    def log_finetune_result(self, round_num: int, overshoot_before: float,
                            overshoot_after: float, settling_time: float):
        """记录微调结果。"""
        self.info(self.PHASE_FINETUNE, f'微调第 {round_num} 轮完成', {
            'round': round_num,
            'overshoot_before': overshoot_before,
            'overshoot_after': overshoot_after,
            'settling_time_after': settling_time,
            'event': 'finetune_complete'
        })

    def log_serial_connect(self, port: str, baud: int):
        """记录串口连接。"""
        self.info(self.PHASE_SERIAL, f'串口连接: {port} @ {baud}', {
            'port': port,
            'baud': baud,
            'event': 'serial_connect'
        })

    def log_serial_disconnect(self):
        """记录串口断开。"""
        self.info(self.PHASE_SERIAL, '串口已断开', {
            'event': 'serial_disconnect'
        })

    def log_serial_error(self, error: str):
        """记录串口错误。"""
        self.error(self.PHASE_SERIAL, f'串口错误: {error}', {
            'error': error,
            'event': 'serial_error'
        })

    def log_session_summary(self, result):
        """记录调参会话总结。"""
        self.info(self.PHASE_GENERAL, '=== 调参会话结束 ===', {
            'status': result.status,
            'message': result.message,
            'recommended_params': result.recommended_params,
            'finetune_rounds': len(result.finetune_history),
            'event': 'session_end'
        })

    def log_cancelled(self):
        """记录调参被取消。"""
        self.warning(self.PHASE_GENERAL, '调参已被用户取消', {
            'event': 'cancelled'
        })

    # ========== 查询方法 ==========

    def get_entries(self) -> List[LogEntry]:
        """获取所有日志条目。"""
        return list(self._entries)

    def get_entries_by_level(self, level: LogLevel) -> List[LogEntry]:
        """按级别过滤日志条目。"""
        return [e for e in self._entries if e.level == level.name]

    def get_entries_by_phase(self, phase: str) -> List[LogEntry]:
        """按阶段过滤日志条目。"""
        return [e for e in self._entries if e.phase == phase]

    def get_entries_by_event(self, event: str) -> List[LogEntry]:
        """按事件类型过滤日志条目。"""
        return [e for e in self._entries
                if e.data and e.data.get('event') == event]

    def get_entry_count(self) -> int:
        """获取日志条目总数。"""
        return len(self._entries)

    def get_level_counts(self) -> Dict[str, int]:
        """统计各级别日志数量。"""
        counts = {}
        for entry in self._entries:
            counts[entry.level] = counts.get(entry.level, 0) + 1
        return counts

    def get_phase_counts(self) -> Dict[str, int]:
        """统计各阶段日志数量。"""
        counts = {}
        for entry in self._entries:
            counts[entry.phase] = counts.get(entry.phase, 0) + 1
        return counts

    def export_json(self) -> str:
        """导出所有日志为 JSON 字符串。"""
        return json.dumps([e.to_dict() for e in self._entries],
                          ensure_ascii=False, indent=2)

    def close(self):
        """关闭日志文件。"""
        if self._file_handle:
            self._file_handle.close()
            self._file_handle = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

#!/usr/bin/env python
"""自动检测 STM32 项目中的 PID 配置（通用版本）。"""

import argparse
import json
import os
import re
import sys


def detect_pid_config(project_root):
    """检测项目中的 PID 配置。"""
    config = {
        'project': os.path.basename(project_root),
        'serial': {'port': 'COM3', 'baud': 115200, 'protocol': 'csv'},
        'axes': [],
        'presets': {}
    }

    # 搜索所有 .c/.h 文件（排除 Drivers 和 .git）
    all_files = []
    max_depth = 10  # S7 修复：限制目录深度
    max_file_size = 1024 * 1024  # 限制文件大小 1MB

    for root, dirs, files in os.walk(project_root):
        # 限制深度
        depth = root.replace(project_root, '').count(os.sep)
        if depth > max_depth:
            dirs.clear()
            continue

        dirs[:] = [d for d in dirs if d not in ('Drivers', '.git', 'MDK-ARM', 'Debug', 'Release', 'Listings', 'Objects')]
        for f in files:
            if f.endswith(('.c', '.h')):
                path = os.path.join(root, f)
                try:
                    # 限制文件大小
                    if os.path.getsize(path) > max_file_size:
                        continue
                    with open(path, 'r', encoding='utf-8', errors='ignore') as file:
                        content = file.read()
                    all_files.append((path, content))
                except (IOError, OSError) as e:
                    print(f"读取文件失败 {path}: {e}", file=sys.stderr)

    # ===== 1. 检测 PID 相关代码 =====
    pid_patterns = [
        # 标准命名
        (r'PID_Init\s*\(', 'PIDController', 'PID_Init'),
        (r'PID_Compute\s*\(', 'PIDController', 'PID_Compute'),
        # 自定义命名（如 Motor_PID_Init, ServoPID_Init 等）
        (r'(\w+PID)\w*_Init\s*\(', None, None),
        (r'(\w+_pid)\w*_init\s*\(', None, None),
        (r'(\w+Pid)\w*_Init\s*\(', None, None),
        # 结构体定义
        (r'typedef\s+struct\s*\{[^}]*\}\s*(\w*[Pp][Ii][Dd]\w*)', None, None),
    ]

    pid_structs = set()
    pid_files = []
    for path, content in all_files:
        is_pid = False
        for pattern, struct_name, func_name in pid_patterns:
            matches = re.findall(pattern, content)
            if matches:
                is_pid = True
                if struct_name:
                    pid_structs.add(struct_name)
                for m in matches:
                    if isinstance(m, str) and ('PID' in m.upper() or 'pid' in m.lower()):
                        pid_structs.add(m)
        if is_pid:
            pid_files.append((path, content))

    # 如果没找到 PID，尝试更宽松的搜索
    if not pid_files:
        for path, content in all_files:
            if re.search(r'\b[Kk]p\b.*\b[Kk]i\b.*\b[Kk]d\b', content):
                pid_files.append((path, content))
                pid_structs.add('PIDController')

    config['_detected_structs'] = list(pid_structs)
    config['_pid_files'] = [f for f, _ in pid_files]

    # ===== 2. 提取 PID 参数 =====
    params = []
    for path, content in pid_files:
        # 查找各种 Init 调用模式
        init_patterns = [
            r'(?:PID|pid|Pid)\w*_Init\s*\(([^)]+)\)',
            r'(?:Motor|Servo|Balance|Angle|Position)\w*(?:PID|Pid|pid)\w*_Init\s*\(([^)]+)\)',
        ]
        for pattern in init_patterns:
            for match in re.finditer(pattern, content):
                call_args = match.group(1)
                nums = re.findall(r'[-+]?\d*\.?\d+f?', call_args)
                if len(nums) >= 6:
                    try:
                        kp = float(nums[0].rstrip('f'))
                        ki = float(nums[1].rstrip('f'))
                        kd = float(nums[2].rstrip('f'))
                        dt = float(nums[3].rstrip('f'))
                        out_min = float(nums[4].rstrip('f'))
                        out_max = float(nums[5].rstrip('f'))
                        if 0 < kp < 100 and 0 < ki < 100 and 0 <= kd < 10:
                            params.append({
                                'file': path, 'kp': kp, 'ki': ki, 'kd': kd,
                                'dt': dt, 'out_min': out_min, 'out_max': out_max
                            })
                    except:
                        pass

    # ===== 3. 检测轴数和名称 =====
    axis_names = []
    for path, content in pid_files:
        # 查找多轴模式
        axis_keywords = {
            'Pitch': ['pitch', 'PITCH', '俯仰'],
            'Roll': ['roll', 'ROLL', '横滚'],
            'Yaw': ['yaw', 'YAW', '偏航'],
            'X': ['_x', '_X', 'X轴', 'axis_x'],
            'Y': ['_y', '_Y', 'Y轴', 'axis_y'],
            'Z': ['_z', '_Z', 'Z轴', 'axis_z'],
        }
        found_axes = []
        for name, keywords in axis_keywords.items():
            for kw in keywords:
                if kw in content:
                    found_axes.append(name)
                    break

        if len(found_axes) >= 2:
            axis_names = found_axes[:3]  # 最多 3 轴
            break
        elif len(found_axes) == 1 and not axis_names:
            axis_names = found_axes

    if not axis_names:
        # 查找 pid_pitch, pid_roll 等变量名
        for path, content in pid_files:
            vars = re.findall(r'(\w+)[_\s]*(?:pid|PID|Pid)', content)
            if vars:
                axis_names = list(set(vars))[:3]
                break

    if not axis_names:
        axis_names = ['PID']

    # ===== 4. 检测串口配置 =====
    uart_configs = []
    for path, content in all_files:
        # 检测 USART/UART 实例
        uart_matches = re.findall(r'(USART[1-5]|UART[4-5])', content)
        if uart_matches:
            for uart in set(uart_matches):
                uart_configs.append(uart)

        # 检测波特率
        baud_matches = re.findall(r'(\d{4,6}).*baud|baud.*(\d{4,6})', content, re.IGNORECASE)
        for m in baud_matches:
            baud = int(m[0] or m[1])
            if baud in (9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600):
                config['serial']['baud'] = baud

        # 检测 GPIO 引脚
        if 'PA9' in content and 'PA10' in content:
            config['serial']['_pins'] = 'PA9/PA10 (USART1)'
        elif 'PA2' in content and 'PA3' in content:
            config['serial']['_pins'] = 'PA2/PA3 (USART2)'
        elif 'PB10' in content and 'PB11' in content:
            config['serial']['_pins'] = 'PB10/PB11 (USART3)'

    if uart_configs:
        config['serial']['_instances'] = list(set(uart_configs))

    # ===== 5. 检测 MCU 系列 =====
    mcu_family = 'STM32F1'
    for path, content in all_files:
        if 'STM32F4' in content or 'stm32f4' in content:
            mcu_family = 'STM32F4'
            break
        elif 'STM32H7' in content or 'stm32h7' in content:
            mcu_family = 'STM32H7'
            break
        elif 'STM32G4' in content or 'stm32g4' in content:
            mcu_family = 'STM32G4'
            break
    config['_mcu_family'] = mcu_family

    # ===== 6. 检测数据输出格式 =====
    for path, content in all_files:
        # 查找 printf/snprintf 格式字符串
        fmt_matches = re.findall(r'(?:snprintf|sprintf)\s*\([^,]+,\s*(?:sizeof[^,]+,)?\s*"([^"]+)"', content)
        for fmt in fmt_matches:
            if '%.1f' in fmt or '%.2f' in fmt or '%d' in fmt:
                config['_output_format'] = fmt
                break

        # 查找 HAL_UART_Transmit 调用
        if 'HAL_UART_Transmit' in content:
            config['_has_uart_output'] = True

    # ===== 7. 构建轴配置 =====
    colors = ['#2196F3', '#FF9800', '#4CAF50', '#E91E63', '#9C27B0', '#00BCD4']
    for i, name in enumerate(axis_names):
        p = params[i] if i < len(params) else (params[0] if params else {
            'kp': 1.5, 'ki': 1.5, 'kd': 0.06, 'dt': 0.005,
            'out_min': -800, 'out_max': 800
        })
        data_cols = 'sp,meas,out' if i == 0 else f'sp{i+1},meas{i+1},out{i+1}'
        axis = {
            'name': name,
            'color': colors[i % len(colors)],
            'data_format': data_cols,
            'params': [
                {'id': 'kp', 'label': 'Kp', 'min': 0, 'max': 5, 'step': 0.01,
                 'default': p['kp'], 'send': True},
                {'id': 'ki', 'label': 'Ki', 'min': 0, 'max': 5, 'step': 0.01,
                 'default': p['ki'], 'send': True},
                {'id': 'kd', 'label': 'Kd', 'min': 0, 'max': 0.5, 'step': 0.001,
                 'default': p['kd'], 'send': True},
                {'id': 'deadband', 'label': 'Deadband', 'min': 0, 'max': 10, 'step': 0.1,
                 'default': 2.0, 'send': True},
                {'id': 'd_tau', 'label': 'D τ 滤波', 'min': 0, 'max': 0.2, 'step': 0.001,
                 'default': 0.05, 'send': False},
                {'id': 'sp_weight', 'label': 'SP Weight', 'min': 0, 'max': 1, 'step': 0.01,
                 'default': 1.0, 'send': False},
                {'id': 'ramp_step', 'label': 'Ramp Step', 'min': 0, 'max': 2, 'step': 0.1,
                 'default': 0.5, 'send': False}
            ]
        }
        config['axes'].append(axis)

    # ===== 8. 生成预设 =====
    if params:
        p = params[0]
        config['presets'] = {
            '当前参数': {
                'kp': p['kp'], 'ki': p['ki'], 'kd': p['kd'],
                'deadband': 2.0, 'd_tau': 0.05, 'sp_weight': 1.0, 'ramp_step': 0.5
            },
            '激进': {
                'kp': round(p['kp'] * 2, 2), 'ki': round(p['ki'] * 1.5, 2),
                'kd': round(p['kd'] * 1.5, 3),
                'deadband': 0.0, 'd_tau': 0.03, 'sp_weight': 0.8, 'ramp_step': 1.0
            },
            '保守': {
                'kp': round(p['kp'] * 0.5, 2), 'ki': round(p['ki'] * 0.5, 2),
                'kd': round(p['kd'] * 0.5, 3),
                'deadband': 5.0, 'd_tau': 0.10, 'sp_weight': 1.0, 'ramp_step': 0.3
            }
        }

    # ===== 9. 添加通用字段 =====
    config['serial']['send_format'] = 'P:{kp:.2f},{ki:.2f},{kd:.3f},{deadband:.1f}\\n'
    config['chart'] = {
        'max_points': 1000,
        'send_interval': 0.05,
        'y_range': [-15, 15],
        'y1_range': [-900, 900]
    }

    return config


def print_summary(config):
    """打印检测摘要。"""
    print(f"项目: {config['project']}")
    print(f"MCU: {config.get('_mcu_family', '未知')}")
    print(f"轴数: {len(config['axes'])}")
    for axis in config['axes']:
        print(f"  - {axis['name']}: Kp={axis['params'][0]['default']}, Ki={axis['params'][1]['default']}, Kd={axis['params'][2]['default']}")
    print(f"串口: {config['serial'].get('_instances', ['未知'])}")
    if '_pins' in config['serial']:
        print(f"引脚: {config['serial']['_pins']}")
    print(f"波特率: {config['serial']['baud']}")
    if '_output_format' in config:
        print(f"输出格式: {config['_output_format']}")


def main():
    parser = argparse.ArgumentParser(description='检测 STM32 项目 PID 配置')
    parser.add_argument('--project', required=True, help='项目根目录')
    parser.add_argument('--output', default=None, help='输出配置文件路径')
    parser.add_argument('--summary', action='store_true', help='打印检测摘要')
    args = parser.parse_args()

    config = detect_pid_config(args.project)

    if args.summary:
        print_summary(config)

    output = json.dumps(config, ensure_ascii=False, indent=2)
    if args.output:
        with open(args.output, 'w', encoding='utf-8') as f:
            f.write(output)
        print(f'配置已保存到: {args.output}')
    else:
        print(output)

    return 0


if __name__ == '__main__':
    sys.exit(main())

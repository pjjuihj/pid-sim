#!/usr/bin/env python
"""在项目中创建 PID 仿真环境。"""

import argparse
import json
import os
import shutil
import sys


def setup_sim(project_root, config_path=None):
    """创建仿真环境。"""
    skill_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sim_dir = os.path.join(project_root, 'tools', 'pid_sim')

    # 创建目录
    os.makedirs(os.path.join(sim_dir, 'templates'), exist_ok=True)
    os.makedirs(os.path.join(sim_dir, 'presets'), exist_ok=True)

    # 复制核心文件
    files = {
        'scripts/server.py': 'server.py',
        'scripts/pid_engine.py': 'pid_engine.py',
        'scripts/plant_model.py': 'plant_model.py',
        'scripts/real_auto_tune.py': 'real_auto_tune.py',
        'templates/index.html': 'templates/index.html',
    }
    for src, dst in files.items():
        src_path = os.path.join(skill_dir, src)
        dst_path = os.path.join(sim_dir, dst)
        if os.path.exists(src_path):
            shutil.copy2(src_path, dst_path)
            print(f'  复制: {dst}')

    # 复制配置文件
    if config_path and os.path.exists(config_path):
        shutil.copy2(config_path, os.path.join(sim_dir, 'config.json'))
        print(f'  复制: config.json (从 {config_path})')
    else:
        # 使用默认配置
        default_config = os.path.join(skill_dir, 'references', 'config.json')
        if os.path.exists(default_config):
            shutil.copy2(default_config, os.path.join(sim_dir, 'config.json'))
            print(f'  复制: config.json (默认)')
        else:
            # 创建最小配置
            config = {
                'project': os.path.basename(project_root),
                'serial': {'port': 'COM3', 'baud': 115200, 'protocol': 'csv',
                           'send_format': 'P:{kp:.2f},{ki:.2f},{kd:.3f},{deadband:.1f}\\n'},
                'chart': {'max_points': 1000, 'send_interval': 0.05, 'y_range': [-15, 15], 'y1_range': [-900, 900]},
                'axes': [{
                    'name': 'PID',
                    'color': '#2196F3',
                    'data_format': 'sp,meas,out',
                    'params': [
                        {'id': 'kp', 'label': 'Kp', 'min': 0, 'max': 5, 'step': 0.01, 'default': 1.5, 'send': True},
                        {'id': 'ki', 'label': 'Ki', 'min': 0, 'max': 5, 'step': 0.01, 'default': 1.5, 'send': True},
                        {'id': 'kd', 'label': 'Kd', 'min': 0, 'max': 0.5, 'step': 0.001, 'default': 0.06, 'send': True},
                        {'id': 'deadband', 'label': 'Deadband', 'min': 0, 'max': 10, 'step': 0.1, 'default': 2.0, 'send': True},
                        {'id': 'd_tau', 'label': 'D τ 滤波', 'min': 0, 'max': 0.2, 'step': 0.001, 'default': 0.05, 'send': False},
                        {'id': 'sp_weight', 'label': 'SP Weight', 'min': 0, 'max': 1, 'step': 0.01, 'default': 1.0, 'send': False},
                        {'id': 'ramp_step', 'label': 'Ramp Step', 'min': 0, 'max': 2, 'step': 0.1, 'default': 0.5, 'send': False}
                    ]
                }],
                'presets': {
                    '默认': {'kp': 1.5, 'ki': 1.5, 'kd': 0.06, 'deadband': 2.0, 'd_tau': 0.05, 'sp_weight': 1.0, 'ramp_step': 0.5},
                    '激进': {'kp': 3.0, 'ki': 2.0, 'kd': 0.10, 'deadband': 0.0, 'd_tau': 0.03, 'sp_weight': 0.8, 'ramp_step': 1.0},
                    '保守': {'kp': 0.5, 'ki': 0.5, 'kd': 0.02, 'deadband': 5.0, 'd_tau': 0.10, 'sp_weight': 1.0, 'ramp_step': 0.3}
                }
            }
            with open(os.path.join(sim_dir, 'config.json'), 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
            print(f'  创建: config.json (最小配置)')

    # 创建 requirements.txt
    with open(os.path.join(sim_dir, 'requirements.txt'), 'w') as f:
        f.write('flask>=3.0\nflask-socketio>=5.3\npyserial>=3.5\n')
    print(f'  创建: requirements.txt')

    print(f'\n仿真环境已创建: {sim_dir}')
    print(f'启动命令: cd {sim_dir} && python server.py')
    print(f'浏览器打开: http://localhost:5000')

    return sim_dir


def main():
    parser = argparse.ArgumentParser(description='创建 PID 仿真环境')
    parser.add_argument('--project', required=True, help='项目根目录')
    parser.add_argument('--config', default=None, help='配置文件路径')
    args = parser.parse_args()

    setup_sim(args.project, args.config)
    return 0


if __name__ == '__main__':
    sys.exit(main())

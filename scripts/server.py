"""PID 仿真 Web 服务器 — 通用版本。"""

import json
import os
import threading
import time
import random
from flask import Flask, render_template, request
from flask_socketio import SocketIO

from pid_engine import PIDController
from plant_model import ServoPlant

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'config.json')
config = {}
pid_controllers = {}  # 每个轴一个 PID
plants = {}           # 每个轴一个物理模型

# 串口
serial_running = False
serial_port = None

# 仿真状态
sim_running = False
sim_mode = 'free'
step_target = 5.0
ramp_setpoints = {}  # 每个轴的斜坡设定值

PRESETS_DIR = os.path.join(os.path.dirname(__file__), 'presets')


def load_config():
    """加载配置文件。"""
    global config
    with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
        config = json.load(f)
    # 初始化 PID 控制器和物理模型
    for axis in config.get('axes', []):
        name = axis['name']
        params = {p['id']: p['default'] for p in axis['params']}
        pid_controllers[name] = PIDController(
            kp=params.get('kp', 1.5), ki=params.get('ki', 1.5),
            kd=params.get('kd', 0.06), dt=0.005,
            out_min=-800.0, out_max=800.0,
            d_tau=params.get('d_tau', 0.05),
            sp_weight=params.get('sp_weight', 1.0),
            deadband=params.get('deadband', 2.0)
        )
        plants[name] = ServoPlant(dt=0.005)
        ramp_setpoints[name] = 0.0
    return config


def save_config():
    """保存配置文件。"""
    with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
        json.dump(config, f, ensure_ascii=False, indent=2)


load_config()


@app.route('/')
def index():
    return render_template('index.html')


# ========== WebSocket 事件 ==========

@socketio.on('connect')
def handle_connect():
    import time
    socketio.emit('config', {
        'project': config.get('project', ''),
        'axes': config.get('axes', []),
        'presets': config.get('presets', {}),
        'serial': config.get('serial', {}),
        'chart': config.get('chart', {}),
        '_ts': time.time(),
    })


@socketio.on('update_params')
def handle_update_params(data):
    """更新某个轴的参数。"""
    axis_name = data.get('axis', 'Roll')
    params = data.get('params', {})
    if axis_name in pid_controllers:
        pid_controllers[axis_name].update_params(params)
        # 斜坡步长
        if 'ramp_step' in params:
            ramp_setpoints[axis_name] = params['ramp_step']
    # 同步到串口
    _send_params_to_serial(axis_name, params)
    socketio.emit('params_updated', {'axis': axis_name, 'params': params})


def _send_params_to_serial(axis_name, params):
    """通过串口发送参数到 STM32。"""
    global serial_port, serial_running
    if not serial_running or not serial_port or not serial_port.is_open:
        return
    # 查找 send_format
    serial_cfg = config.get('serial', {})
    send_fmt = serial_cfg.get('send_format', '')
    if not send_fmt:
        return
    # 收集所有参数
    all_params = {}
    if axis_name in pid_controllers:
        all_params = pid_controllers[axis_name].get_params()
    all_params.update(params)
    try:
        cmd = send_fmt.format(**all_params)
        serial_port.write(cmd.encode('ascii'))
    except Exception:
        pass


@socketio.on('apply_preset')
def handle_apply_preset(data):
    """应用预设参数。"""
    preset_name = data.get('name', '')
    presets = config.get('presets', {})
    if preset_name not in presets:
        return
    params = presets[preset_name]
    # 应用到所有轴
    for name, pid in pid_controllers.items():
        pid.update_params(params)
    _send_params_to_serial('Roll', params)
    socketio.emit('apply_params', params)


@socketio.on('save_preset')
def handle_save_preset(data):
    """保存当前参数为预设。"""
    name = data.get('name', '新预设')
    params = data.get('params', {})
    if 'presets' not in config:
        config['presets'] = {}
    config['presets'][name] = params
    save_config()
    socketio.emit('presets_updated', config['presets'])


@socketio.on('delete_preset')
def handle_delete_preset(data):
    """删除预设。"""
    name = data.get('name', '')
    if name in config.get('presets', {}):
        del config['presets'][name]
        save_config()
        socketio.emit('presets_updated', config['presets'])


@socketio.on('update_config')
def handle_update_config(data):
    """更新配置（串口、图表等）。"""
    if 'serial' in data:
        config['serial'].update(data['serial'])
    if 'chart' in data:
        config['chart'].update(data['chart'])
    save_config()
    socketio.emit('config_updated', config)


# ========== 仿真 ==========

@socketio.on('start_sim')
def handle_start_sim(data=None):
    global sim_running, sim_mode
    if sim_running:
        return
    sim_mode = data.get('mode', 'free') if data else 'free'
    for name in plants:
        plants[name].reset()
        pid_controllers[name].reset()
        ramp_setpoints[name] = 0.0
    sim_running = True
    socketio.start_background_task(sim_loop)


@socketio.on('stop_sim')
def handle_stop_sim():
    global sim_running
    sim_running = False


@socketio.on('reset_sim')
def handle_reset_sim():
    global sim_running
    sim_running = False
    time.sleep(0.1)
    for name in plants:
        plants[name].reset()
        pid_controllers[name].reset()
        ramp_setpoints[name] = 0.0


@socketio.on('step_response')
def handle_step_response(data=None):
    global step_target
    step_target = data.get('target', 5.0) if data else 5.0
    handle_reset_sim()
    handle_start_sim({'mode': 'step'})


def sim_loop():
    """仿真主循环。"""
    global sim_running
    step_count = 0
    dt = 0.005
    chart_cfg = config.get('chart', {})
    send_interval = chart_cfg.get('send_interval', 0.05)
    last_send = time.time()

    # 数据缓冲区
    buffers = {}
    for axis in config.get('axes', []):
        buffers[axis['name']] = {'t': [], 'sp': [], 'meas': [], 'out': []}

    while sim_running:
        t = step_count * dt

        for axis in config.get('axes', []):
            name = axis['name']
            pid = pid_controllers[name]
            plant = plants[name]

            # 设定值
            if sim_mode == 'step':
                target = step_target if t > 0.5 else 0.0
            elif sim_mode == 'steady':
                target = step_target
            else:
                target = _generate_ir_setpoint(t)

            # 斜坡回零
            ramp = ramp_setpoints.get(name, 0.5)
            rp = ramp_setpoints.get(f'{name}_val', 0.0)
            if abs(target) > 0.01:
                rp = target
            else:
                if rp > ramp:
                    rp -= ramp
                elif rp < -ramp:
                    rp += ramp
                else:
                    rp = 0.0
            ramp_setpoints[f'{name}_val'] = rp

            # PID 计算
            meas = plant.platform_angle + random.gauss(0, plant.noise_sigma)
            out = pid.compute(rp, meas)
            plant.update(out)

            buf = buffers[name]
            buf['t'].append(round(t, 3))
            buf['sp'].append(round(rp, 2))
            buf['meas'].append(round(meas, 2))
            buf['out'].append(round(out, 1))

        # 定时发送
        now = time.time()
        if now - last_send >= send_interval:
            for name, buf in buffers.items():
                if buf['t']:
                    socketio.emit('sim_data', {'axis': name, **buf})
                    buffers[name] = {'t': [], 'sp': [], 'meas': [], 'out': []}
            last_send = now

        step_count += 1
        time.sleep(dt)


def _generate_ir_setpoint(t):
    cycle = 8.0
    pt = t % cycle
    if pt < 2.0:   return -8.0 + 8.0 * (pt / 2.0)
    elif pt < 4.0: return 8.0
    elif pt < 6.0: return 8.0 - 8.0 * ((pt - 4.0) / 2.0)
    elif pt < 7.0: return -8.0
    else:          return 0.0


# ========== 自动调参 ==========

@socketio.on('auto_tune')
def handle_auto_tune(data=None):
    global sim_running
    if sim_running:
        socketio.emit('auto_tune_progress', {'status': 'busy', 'msg': '请先停止仿真'})
        return
    target = data.get('target', 5.0) if data else 5.0
    socketio.start_background_task(_auto_tune_worker, target)


def _auto_tune_worker(target):
    global sim_running
    dt = 0.005
    kp_range = [0.5, 1.0, 1.5, 2.0, 3.0]
    ki_range = [0.5, 1.0, 1.5, 2.0, 3.0]
    kd_range = [0.02, 0.04, 0.06, 0.08, 0.10]
    db_range = [0.0, 1.0, 2.0, 3.0, 5.0]
    total = len(kp_range) * len(ki_range) * len(kd_range) * len(db_range)
    best_score = float('inf')
    best_params = {}
    best_metrics = {}
    count = 0

    socketio.emit('auto_tune_progress', {'status': 'running', 'progress': 0, 'total': total, 'msg': f'搜索 {total} 种组合...'})

    for kp in kp_range:
        for ki in ki_range:
            for kd in kd_range:
                for db in db_range:
                    count += 1
                    tp = PIDController(kp=kp, ki=ki, kd=kd, dt=dt, out_min=-800, out_max=800, d_tau=0.05, sp_weight=1.0, deadband=db)
                    tp_plant = ServoPlant(dt=dt)
                    steps = int(3.0 / dt)
                    meas_list = []
                    for i in range(steps):
                        tt = i * dt
                        sp = target if tt > 0.5 else 0.0
                        m = tp_plant.platform_angle + random.gauss(0, 0.1)
                        o = tp.compute(sp, m)
                        tp_plant.update(o)
                        meas_list.append(m)
                    ss = meas_list[int(2.0/dt):]
                    n = len(ss)
                    ss_err = sum(abs(target - ss[i]) for i in range(n)) / n
                    ss_mean = sum(ss) / n
                    ss_osc = (sum((m - ss_mean)**2 for m in ss) / n) ** 0.5
                    pk = max(meas_list[int(0.5/dt):]) if target > 0 else min(meas_list[int(0.5/dt):])
                    overshoot = max(0, (abs(pk) - abs(target)) / abs(target) * 100) if abs(target) > 0.1 else 0
                    score = ss_err * 10 + ss_osc * 20 + overshoot * 0.5
                    if score < best_score:
                        best_score = score
                        best_params = {'kp': kp, 'ki': ki, 'kd': kd, 'deadband': db, 'd_tau': 0.05, 'sp_weight': 1.0}
                        best_metrics = {'ss_error': round(ss_err, 3), 'ss_osc': round(ss_osc, 3), 'overshoot': round(overshoot, 1), 'score': round(score, 2)}
                    if count % 10 == 0 or count == total:
                        socketio.emit('auto_tune_progress', {'status': 'running', 'progress': count, 'total': total, 'msg': f'{count}/{total} | 最优: Kp={best_params.get("kp"):.1f} Ki={best_params.get("ki"):.1f} Kd={best_params.get("kd"):.2f} Db={best_params.get("deadband"):.0f}'})
                        socketio.sleep(0.01)

    socketio.emit('auto_tune_result', {'status': 'done', 'params': best_params, 'metrics': best_metrics, 'msg': f'完成！Kp={best_params["kp"]:.2f} Ki={best_params["ki"]:.2f} Kd={best_params["kd"]:.3f} Db={best_params["deadband"]:.1f}'})
    for name in pid_controllers:
        pid_controllers[name].update_params(best_params)
    socketio.emit('apply_params', best_params)
    _send_params_to_serial('Roll', best_params)


# ========== 串口 ==========

@socketio.on('list_ports')
def handle_list_ports():
    if not HAS_SERIAL:
        socketio.emit('port_list', {'ports': []})
        return
    ports = serial.tools.list_ports.comports()
    socketio.emit('port_list', {'ports': [{'device': p.device, 'desc': p.description} for p in ports]})


@socketio.on('start_serial')
def handle_start_serial(data=None):
    global serial_running, serial_port
    if serial_running:
        return
    if not HAS_SERIAL:
        socketio.emit('serial_status', {'status': 'error', 'msg': 'pyserial 未安装'})
        return
    port_name = data.get('port', config.get('serial', {}).get('port', 'COM3'))
    baud = data.get('baud', config.get('serial', {}).get('baud', 115200))
    try:
        serial_port = serial.Serial(port_name, baud, timeout=0.1)
        serial_running = True
        socketio.emit('serial_status', {'status': 'connected', 'msg': f'{port_name} @ {baud}'})
        socketio.start_background_task(_serial_reader)
    except Exception as e:
        socketio.emit('serial_status', {'status': 'error', 'msg': f'失败: {e}'})


@socketio.on('stop_serial')
def handle_stop_serial():
    global serial_running, serial_port
    serial_running = False
    if serial_port and serial_port.is_open:
        serial_port.close()
    socketio.emit('serial_status', {'status': 'disconnected', 'msg': '已断开'})


def _serial_reader():
    """串口读取，支持多轴数据。"""
    global serial_running
    protocol = config.get('serial', {}).get('protocol', 'csv')
    axes = config.get('axes', [])
    buffer = ''
    last_send = time.time()
    send_interval = config.get('chart', {}).get('send_interval', 0.05)
    t = 0.0
    buffers = {a['name']: {'t': [], 'sp': [], 'meas': [], 'out': []} for a in axes}

    while serial_running and serial_port and serial_port.is_open:
        try:
            raw = serial_port.read(512)
            if raw:
                buffer += raw.decode('ascii', errors='ignore')

                if protocol == 'vofa-firewater':
                    # VOFA+ FireWater: 每行逗号分隔浮点数
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        parts = line.split(',')
                        # 根据列数分配到各轴
                        col_idx = 0
                        for axis in axes:
                            fmt = axis.get('data_format', 'sp,meas,out')
                            n_cols = len(fmt.split(','))
                            if col_idx + n_cols <= len(parts):
                                vals = []
                                for i in range(n_cols):
                                    try:
                                        vals.append(float(parts[col_idx + i]))
                                    except ValueError:
                                        vals.append(0.0)
                                t += 0.025
                                buf = buffers[axis['name']]
                                buf['t'].append(round(t, 3))
                                if len(vals) >= 1: buf['sp'].append(round(vals[0], 2))
                                if len(vals) >= 2: buf['meas'].append(round(vals[1], 2))
                                if len(vals) >= 3: buf['out'].append(round(vals[2], 1))
                                col_idx += n_cols
                else:
                    # CSV: 每行逗号分隔，第一轴解析
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if not line:
                            continue
                        parts = line.split(',')
                        if len(parts) >= 3:
                            try:
                                sp = float(parts[0])
                                meas = float(parts[1])
                                out = float(parts[2])
                                t += 0.025
                                buf = buffers[axes[0]['name']]
                                buf['t'].append(round(t, 3))
                                buf['sp'].append(round(sp, 2))
                                buf['meas'].append(round(meas, 2))
                                buf['out'].append(round(out, 1))
                            except ValueError:
                                pass
        except Exception:
            pass

        now = time.time()
        if now - last_send >= send_interval:
            for name, buf in buffers.items():
                if buf['t']:
                    socketio.emit('sim_data', {'axis': name, **buf})
                    buffers[name] = {'t': [], 'sp': [], 'meas': [], 'out': []}
            last_send = now

        socketio.sleep(0.01)


if __name__ == '__main__':
    print(f"PID Tuner 启动: http://localhost:5000")
    print(f"项目: {config.get('project', '')}")
    print(f"轴数: {len(config.get('axes', []))}")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, allow_unsafe_werkzeug=True)

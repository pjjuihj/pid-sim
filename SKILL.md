---
name: pid-sim
description: >
  当用户提到 PID 控制、PID 调参、PID 仿真、稳态振荡、舵机控制、姿态控制、
  平台稳定、卡尔曼滤波调参、PID 参数优化、自动调参，或者用户说"帮我调 PID"、
  "PID 波形"、"看下 PID 响应"、"消除振荡"等类似表述时，使用此技能。
  即使用户没有明确说"仿真"，只要涉及 PID 控制器的参数调整或性能分析，就应触发。
---

# PID 仿真调参工具

为 STM32 项目创建一个基于浏览器的 PID 仿真和实时调参环境。

## 功能

- **离线仿真**：PC 端模拟 PID 控制器 + 物理模型，无需硬件即可调参
- **实时串口**：连接 STM32 硬件，实时显示 PID 数据
- **双向改参**：拖动滑块自动发送参数到 STM32，实时生效
- **自动调参**：网格搜索最优参数组合
- **多轴支持**：同时显示 Pitch 和 Roll（或任意轴）
- **多协议**：CSV、VOFA+ FireWater、JustFloat
- **参数预设**：保存/加载不同场景的参数配置
- **图表交互**：缩放、平移、隐藏/显示曲线

## 工作流程

### 步骤 1：检测项目配置

读取项目中的 PID 相关代码，提取：
- PID 参数（Kp, Ki, Kd, dt, 输出限幅）
- 串口配置（USART 引脚、波特率）
- 轴数（单轴/双轴）
- 数据输出格式

运行配置检测脚本：
```bash
python <skill-path>/scripts/detect_pid_config.py --project <项目根目录>
```

如果检测失败，提示用户手动配置。

### 步骤 2：创建仿真环境

将仿真工具复制到项目的 `tools/pid_sim/` 目录：
```bash
python <skill-path>/scripts/setup_sim.py --project <项目根目录> --config <检测到的配置>
```

这会创建：
- `tools/pid_sim/config.json` — 项目配置
- `tools/pid_sim/server.py` — Flask 后端
- `tools/pid_sim/pid_engine.py` — PID 仿真引擎
- `tools/pid_sim/plant_model.py` — 物理模型
- `tools/pid_sim/templates/index.html` — 前端页面
- `tools/pid_sim/requirements.txt` — Python 依赖

### 步骤 3：安装依赖

```bash
pip install flask flask-socketio pyserial
```

### 步骤 4：启动仿真器

```bash
cd <项目根目录>/tools/pid_sim && python server.py
```

浏览器打开 http://localhost:5000

### 步骤 5：固件适配（可选）

如果需要串口实时改参，需要在固件中添加：
1. UART 初始化（USART1, PA9/PA10, 115200）
2. PID 数据输出（每 25ms 发送 `sp,meas,out\n`）
3. 参数接收解析（接收 `P:kp,ki,kd,deadband\n`）

使用固件适配脚本：
```bash
python <skill-path>/scripts/adapt_firmware.py --project <项目根目录> --uart USART1
```

## 配置文件格式

`config.json` 示例：
```json
{
  "project": "项目名称",
  "serial": {
    "port": "COM3",
    "baud": 115200,
    "protocol": "csv",
    "send_format": "P:{kp:.2f},{ki:.2f},{kd:.3f},{deadband:.1f}\\n"
  },
  "axes": [
    {
      "name": "Roll",
      "data_format": "sp,meas,out",
      "params": [
        {"id": "kp", "label": "Kp", "min": 0, "max": 5, "step": 0.01, "default": 1.5, "send": true},
        {"id": "ki", "label": "Ki", "min": 0, "max": 5, "step": 0.01, "default": 1.5, "send": true},
        {"id": "kd", "label": "Kd", "min": 0, "max": 0.5, "step": 0.001, "default": 0.06, "send": true},
        {"id": "deadband", "label": "Deadband", "min": 0, "max": 10, "step": 0.1, "default": 2.0, "send": true}
      ]
    }
  ],
  "presets": {
    "默认": {"kp": 1.5, "ki": 1.5, "kd": 0.06, "deadband": 2.0}
  }
}
```

## 适配新项目的步骤

1. 复制 `tools/pid_sim/` 到新项目
2. 修改 `config.json` 中的参数范围和默认值
3. 修改固件添加 UART 数据输出
4. 启动 `python server.py`，连接串口

## 固件 UART 数据输出格式

固件需要每 25ms 发送一行 CSV 数据：
```
setpoint,measurement,output\n
```

例如：`0.0,0.15,-12.5\n`

如果有多轴，连续发送：
```
sp1,meas1,out1,sp2,meas2,out2\n
```

## 固件参数接收格式

固件需要解析以下命令：
```
P:kp,ki,kd,deadband\n
```

例如：`P:1.50,1.50,0.06,2.0\n`

收到后更新 PID 结构体参数并回显确认：
```
OK:1.50,1.50,0.060,2.0\n
```

## 脚本说明

| 脚本 | 功能 |
|------|------|
| `scripts/detect_pid_config.py` | 自动检测项目 PID 配置 |
| `scripts/setup_sim.py` | 创建仿真环境 |
| `scripts/adapt_firmware.py` | 适配固件 UART 输出 |

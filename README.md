# PID Tuner — 通用 STM32 PID 仿真调参工具

基于浏览器的 PID 控制器仿真和实时调参环境，支持离线仿真和串口连接硬件。

## 功能

| 功能 | 说明 |
|------|------|
| 🎮 离线仿真 | PC 端模拟 PID + 物理模型，无需硬件 |
| 📡 实时串口 | 连接 STM32，实时显示 PID 波形 |
| ✏️ 双向改参 | 拖动滑块自动发送到 STM32，实时生效 |
| ⚡ 自动调参 | 网格搜索最优参数组合 |
| 📐 多轴支持 | Roll / Pitch / Yaw 同时显示 |
| 📋 多协议 | CSV、VOFA+ FireWater |
| 💾 参数预设 | 保存/加载不同场景配置 |
| 🔍 图表交互 | 缩放、平移、隐藏/显示曲线 |

## 快速开始

### 1. 安装依赖

```bash
pip install flask flask-socketio pyserial
```

### 2. 自动检测项目配置

```bash
python scripts/detect_pid_config.py --project <你的STM32项目目录> --output config.json --summary
```

自动识别：
- PID 参数（Kp, Ki, Kd）
- 串口配置（USART1/2/3, 波特率）
- 轴数（Roll/Pitch/Yaw）
- MCU 系列（F1/F4/H7）

### 3. 创建仿真环境

```bash
python scripts/setup_sim.py --project <项目目录> --config config.json
```

### 4. 启动

```bash
cd <项目目录>/tools/pid_sim
python server.py
```

浏览器打开 http://localhost:5000

## 界面预览

Keil / STM32CubeIDE 风格界面：
- 左侧：参数面板（可折叠分区）
- 中央：实时图表（缩放/平移/隐藏曲线）
- 底部：状态栏

**键盘快捷键：**
- `Space` — 开始/停止仿真
- `R` — 重置
- `Home` — 重置图表缩放

## 串口连接

### 接线

| STM32 | CH340 |
|-------|-------|
| PA9 (TX) | RX |
| PA10 (RX) | TX |
| GND | GND |

### 固件适配

```bash
python scripts/adapt_firmware.py --project <项目目录>
```

自动生成 `usart.c` / `usart.h`，并提示需要添加的代码。

### 数据格式

**STM32 → PC**（每 25ms）：
```
setpoint,measurement,output\n
```

**PC → STM32**（滑块改变时）：
```
P:kp,ki,kd,deadband\n
```

## 配置文件

`config.json` 示例：

```json
{
  "project": "稳定平台",
  "serial": {
    "port": "COM3",
    "baud": 115200,
    "protocol": "csv",
    "send_format": "P:{kp:.2f},{ki:.2f},{kd:.3f},{deadband:.1f}\\n"
  },
  "axes": [
    {
      "name": "Roll",
      "color": "#2196F3",
      "data_format": "sp,meas,out",
      "params": [
        {"id": "kp", "label": "Kp", "min": 0, "max": 5, "step": 0.01, "default": 1.5, "send": true},
        {"id": "ki", "label": "Ki", "min": 0, "max": 5, "step": 0.01, "default": 1.5, "send": true},
        {"id": "kd", "label": "Kd", "min": 0, "max": 0.5, "step": 0.001, "default": 0.06, "send": true}
      ]
    }
  ],
  "presets": {
    "默认": {"kp": 1.5, "ki": 1.5, "kd": 0.06, "deadband": 2.0}
  }
}
```

## 适配新项目

1. 复制 `tools/pid_sim/` 到新项目
2. 修改 `config.json` 参数范围和默认值
3. 固件添加 UART 数据输出
4. 启动 `python server.py`

## 文件结构

```
pid-sim/
├── SKILL.md                    # 技能定义（Claude Code 使用）
├── README.md                   # 本文件
├── evals/evals.json            # 测试用例
├── references/config.json      # 参考配置
├── templates/index.html        # 前端页面
└── scripts/
    ├── detect_pid_config.py    # 自动检测项目配置
    ├── setup_sim.py            # 创建仿真环境
    ├── adapt_firmware.py       # 适配固件 UART
    ├── server.py               # Flask 后端
    ├── pid_engine.py           # PID 仿真引擎
    └── plant_model.py          # 物理模型
```

## 技术栈

- **后端**: Python, Flask, Flask-SocketIO
- **前端**: HTML, CSS, JavaScript, Chart.js, chartjs-plugin-zoom
- **通信**: WebSocket, 串口 (pyserial)

## 许可

MIT

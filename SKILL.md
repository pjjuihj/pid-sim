---
name: pid-sim
description: >
  当用户提到 PID 控制、PID 调参、PID 仿真、稳态振荡、舵机控制、姿态控制、
  平台稳定、卡尔曼滤波调参、PID 参数优化、自动调参，或者用户说"帮我调 PID"、
  "PID 波形"、"看下 PID 响应"、"消除振荡"等类似表述时，使用此技能。
  即使用户没有明确说"仿真"，只要涉及 PID 控制器的参数调整或性能分析，就应触发。
  支持中英文界面，自动检测项目配置（包括 #define 宏定义），生成分析报告。
  支持离线仿真和实物自动调参。
---

# PID 仿真调参工具

为 STM32 项目创建一个基于浏览器的 PID 仿真和实时调参环境。

## 功能

- **离线仿真**：PC 端模拟 PID 控制器 + 物理模型，无需硬件即可调参
- **实物自动调参**：连接 STM32 硬件，基于继电反馈法自动调参
- **实时串口**：连接 STM32 硬件，实时显示 PID 数据
- **双向改参**：拖动滑块自动发送参数到 STM32，实时生效
- **自动调参**：网格搜索、遗传算法、PSO、Ziegler-Nichols
- **多轴支持**：同时显示 Pitch 和 Roll（或任意轴）
- **多协议**：CSV、VOFA+ FireWater、JustFloat
- **参数预设**：保存/加载不同场景的参数配置
- **图表交互**：缩放、平移、隐藏/显示曲线
- **性能指标**：实时显示稳态误差、超调量、振荡次数、ITAE
- **深色模式**：支持主题切换，localStorage 持久化
- **响应式设计**：支持手机/平板/桌面
- **Docker 部署**：一键容器化部署
- **参数范围选择**：下拉菜单选择预设范围或自定义输入

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

**支持的配置格式：**
- `#define PID_KP 100.0` — 宏定义格式
- `.Kp = 100.0` — 结构体初始化格式
- `PID_Init(&pid, 100.0, 0, 10.0)` — 函数调用格式

如果检测失败，提示用户手动配置。提供配置模板：
```json
{
  "project": "项目名称",
  "serial": {"port": "COM3", "baud": 115200, "protocol": "csv"},
  "axes": [{"name": "PID", "params": [
    {"id": "kp", "label": "Kp", "min": 0, "max": 200, "default": 100},
    {"id": "ki", "label": "Ki", "min": 0, "max": 10, "default": 0},
    {"id": "kd", "label": "Kd", "min": 0, "max": 50, "default": 10}
  ]}]
}
```

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
- `tools/pid_sim/real_auto_tune.py` — 实物自动调参模块
- `tools/pid_sim/templates/index.html` — 前端页面
- `tools/pid_sim/requirements.txt` — Python 依赖

### 步骤 3：安装依赖

```bash
pip install flask flask-socketio pyserial numpy pytest gevent gevent-websocket
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

## 调参算法

| 算法 | 说明 | 适用场景 |
|------|------|----------|
| **网格搜索** | 遍历所有参数组合 | 参数空间小 |
| **遗传算法** | 模拟自然选择 | 参数空间大 |
| **PSO** | 粒子群优化 | 连续优化 |
| **Ziegler-Nichols** | 基于临界振荡 | 已知系统响应 |
| **继电反馈法** | 连接实物自动调参 | 实物调参 |

## 实物自动调参

### 使用方法

1. 连接 STM32 串口
2. 点击网页上的「🔌 实物调参」按钮
3. 等待 30-60 秒
4. 查看最优 PID 参数

### 调参流程

```
阶段 1：继电反馈振荡（10-20秒）
  └─ 发送继电器信号，产生振荡
  └─ 测量振荡幅值和周期

阶段 2：参数计算
  └─ 计算 Ku（临界增益）和 Tu（临界周期）
  └─ 用 Ziegler-Nichols 公式计算 PID

阶段 3：验证阶跃响应（5秒）
  └─ 发送阶跃信号
  └─ 测量响应，计算超调量
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
        {"id": "kp", "label": "Kp", "min": 0, "max": 5, "step": 0.01, "default": 1.5, "send": true, "range_select": true},
        {"id": "ki", "label": "Ki", "min": 0, "max": 5, "step": 0.01, "default": 1.5, "send": true, "range_select": true},
        {"id": "kd", "label": "Kd", "min": 0, "max": 0.5, "step": 0.001, "default": 0.06, "send": true, "range_select": true},
        {"id": "deadband", "label": "Deadband", "min": 0, "max": 10, "step": 0.1, "default": 2.0, "send": true, "range_select": true}
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

## Docker 部署

```bash
cd tools/pid_sim
docker-compose up -d
```

浏览器打开 http://localhost:5000

## 测试

```bash
cd tools/pid_sim
python -m pytest test_*.py -v
```

## 脚本说明

| 脚本 | 功能 |
|------|------|
| `scripts/detect_pid_config.py` | 自动检测项目 PID 配置 |
| `scripts/setup_sim.py` | 创建仿真环境 |
| `scripts/adapt_firmware.py` | 适配固件 UART 输出 |
| `scripts/tuning_algorithms.py` | 调参算法集合 |

## 故障排除

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| 振荡严重 | Kp 过大或 Kd 过小 | 降低 Kp 或增大 Kd |
| 稳态误差大 | Ki 过小或死区过大 | 增大 Ki 或减小死区 |
| 响应慢 | Kp 过小 | 增大 Kp |
| 超调大 | Kd 过小 | 增大 Kd |
| 串口连接失败 | 端口错误或波特率不匹配 | 检查设备管理器端口号 |
| 实物调参失败 | 未连接串口或振荡不明显 | 先连接串口，增大继电器幅值 |
| CDN 资源加载失败 | 浏览器阻止外部资源 | 下载资源到本地 static/js/ 目录 |
| WebSocket 连接失败 | Edge Tracking Prevention | 禁用 Tracking Prevention 或使用 Chrome |

## 性能指标

| 指标 | 数值 |
|------|------|
| **单元测试** | 71 个，100% 通过 |
| **PID 仿真速度** | 743,447 步/秒 |
| **调参速度** | 0.041s/次 |
| **内存使用** | 0.34 MB |
| **综合评分** | 92.15/100 |

## 文件结构

```
tools/pid_sim/
├── server.py              # Flask 后端
├── pid_engine.py          # PID 仿真引擎
├── plant_model.py         # 物理模型
├── real_auto_tune.py      # 实物自动调参
├── tuning_algorithms.py   # 调参算法
├── templates/
│   └── index.html         # 前端页面
├── static/js/             # 本地 CDN 资源
├── test_pid_engine.py     # 单元测试
├── test_plant_model.py    # 单元测试
├── test_tuning_algorithms.py  # 单元测试
├── config.json            # 项目配置
└── README.md              # 项目文档
```

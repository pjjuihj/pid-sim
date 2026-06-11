"""舵机 + 平台物理模型。"""

import math
import random


class ServoPlant:
    """简化的舵机 + 平台二阶系统模型。"""

    def __init__(self, dt=0.005):
        self.dt = dt
        # 舵机参数
        self.servo_tau = 0.05       # 舵机一阶惯性时间常数 (s)
        self.servo_angle = 0.0      # 当前舵机角度 (°)
        self.servo_deadband = 0.5   # 舵机死区 (°)
        # 平台参数
        self.omega_n = 2.0 * math.pi * 10.0  # 自然频率 10Hz
        self.zeta = 0.7             # 阻尼比
        self.platform_angle = 0.0   # 平台角度 (°)
        self.platform_velocity = 0.0  # 平台角速度 (°/s)
        # 传感器噪声
        self.noise_sigma = 0.1      # 噪声标准差 (°)

    def reset(self):
        """重置状态。"""
        self.servo_angle = 0.0
        self.platform_angle = 0.0
        self.platform_velocity = 0.0

    def update(self, pid_output):
        """
        输入 PID 输出（PWM 偏移量，±800），返回带噪声的平台角度。

        模型：
        1. PID 输出 → 目标舵机角度（线性映射：±800 → ±25°）
        2. 舵机一阶惯性响应
        3. 平台二阶系统响应
        4. 加传感器噪声
        """
        # PID 输出 (-800~+800) → 目标角度 (-25°~+25°)
        target_angle = pid_output * 25.0 / 800.0

        # 舵机一阶惯性：angle += (target - angle) * (1 - exp(-dt/tau))
        alpha_servo = 1.0 - math.exp(-self.dt / self.servo_tau)
        self.servo_angle += alpha_servo * (target_angle - self.servo_angle)

        # 平台二阶系统（阻尼弹簧模型）
        force = self.omega_n ** 2 * (self.servo_angle - self.platform_angle) \
                - 2.0 * self.zeta * self.omega_n * self.platform_velocity
        self.platform_velocity += force * self.dt
        self.platform_angle += self.platform_velocity * self.dt

        # 加传感器噪声
        noise = random.gauss(0, self.noise_sigma)
        measurement = self.platform_angle + noise

        return measurement

    def get_state(self):
        """返回当前状态。"""
        return {
            'servo_angle': self.servo_angle,
            'platform_angle': self.platform_angle,
            'platform_velocity': self.platform_velocity,
        }

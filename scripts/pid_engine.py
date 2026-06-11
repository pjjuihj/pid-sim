"""PID 仿真引擎 — 与固件 PID.c 算法完全一致。"""


class PIDController:
    """PID 控制器，算法与固件 PID_Compute 一致。"""

    def __init__(self, kp=1.5, ki=1.5, kd=0.06, dt=0.005,
                 out_min=-800.0, out_max=800.0,
                 d_tau=0.05, sp_weight=1.0, deadband=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.out_min = out_min
        self.out_max = out_max
        self.d_filter_tau = d_tau
        self.sp_weight = sp_weight
        self.deadband = deadband
        self.reset()

    def reset(self):
        """重置内部状态。"""
        self.integrator = 0.0
        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.d_filtered = 0.0
        self.output = 0.0

    def compute(self, setpoint, measurement):
        """计算 PID 输出，算法与固件 PID_Compute 完全一致。"""
        error = setpoint - measurement

        # 1. 死区判断
        if -self.deadband < error < self.deadband:
            self.prev_error = error
            self.prev_measurement = measurement
            self.output = 0.0
            return 0.0

        # 2. 比例项：setpoint weighting
        proportional = self.kp * (self.sp_weight * setpoint - measurement)

        # 3. 微分项：基于测量值变化率 + 一阶低通滤波
        raw_derivative = (self.prev_measurement - measurement) / self.dt
        alpha = self.dt / (self.dt + self.d_filter_tau)
        self.d_filtered = alpha * raw_derivative + (1.0 - alpha) * self.d_filtered
        derivative = self.kd * self.d_filtered

        # 4. 积分项：条件积分 anti-windup
        self.integrator += error * self.dt
        if self.integrator > self.out_max:
            self.integrator = self.out_max
        if self.integrator < self.out_min:
            self.integrator = self.out_min

        # 5. 计算输出
        output = proportional + self.ki * self.integrator + derivative

        # 6. 输出限幅
        if output > self.out_max:
            output = self.out_max
        if output < self.out_min:
            output = self.out_min

        self.prev_error = error
        self.prev_measurement = measurement
        self.output = output
        return output

    def update_params(self, params):
        """动态更新参数。"""
        if 'kp' in params:
            self.kp = params['kp']
        if 'ki' in params:
            self.ki = params['ki']
        if 'kd' in params:
            self.kd = params['kd']
        if 'd_tau' in params:
            self.d_filter_tau = params['d_tau']
        if 'sp_weight' in params:
            self.sp_weight = params['sp_weight']
        if 'deadband' in params:
            self.deadband = params['deadband']

    def get_params(self):
        """返回当前参数字典。"""
        return {
            'kp': self.kp, 'ki': self.ki, 'kd': self.kd,
            'dt': self.dt, 'out_min': self.out_min, 'out_max': self.out_max,
            'd_tau': self.d_filter_tau, 'sp_weight': self.sp_weight,
            'deadband': self.deadband,
        }

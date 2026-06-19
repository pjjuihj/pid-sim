"""调参算法集合。"""

import numpy as np
import random
from dataclasses import dataclass
from typing import Optional

@dataclass
class PIDParams:
    kp: float
    ki: float
    kd: float

@dataclass
class TuningResult:
    params: PIDParams
    score: float
    method: str

@dataclass
class FOPTDModel:
    """一阶加纯滞后模型: G(s) = K * exp(-L*s) / (T*s + 1)"""
    K: float       # 过程增益
    L: float       # 纯滞后时间 (dead time)
    T: float       # 时间常数

    def validate(self):
        """验证模型参数有效性。"""
        if self.K <= 0:
            raise ValueError(f"过程增益 K 必须为正, 当前值: {self.K}")
        if self.L < 0:
            raise ValueError(f"纯滞后时间 L 不能为负, 当前值: {self.L}")
        if self.T <= 0:
            raise ValueError(f"时间常数 T 必须为正, 当前值: {self.T}")


class CohenCoon:
    """
    Cohen-Coon 调参算法。

    基于一阶加纯滞后 (FOPTD) 模型 G(s) = K * exp(-L*s) / (T*s + 1),
    通过过程反应曲线辨识模型参数后计算 PID 参数。

    公式:
      Kp = (1/K) * (T/L) * (4/3 + L/(4T))
      Ti = L * (32 + 6*L/T) / (13 + 8*L/T)
      Td = L * 4 / (11 + 2*L/T)

    特点:
      - 相比 Ziegler-Nichols, Cohen-Coon 对大滞后系统有更好的补偿
      - 在 L/T 较大时仍能保持较好的控制效果
      - 计算简单, 适合嵌入式系统实时调参
    """

    def __init__(self, model: FOPTDModel):
        """
        初始化 Cohen-Coon 调参器。

        Args:
            model: 一阶加纯滞后模型参数
        """
        self.model = model
        self.model.validate()

    def compute_pid(self) -> PIDParams:
        """计算 PID 参数。"""
        K, L, T = self.model.K, self.model.L, self.model.T

        if L == 0:
            # 无纯滞后: 简化为纯一阶系统
            return PIDParams(kp=1.0 / K, ki=1.0 / (K * T), kd=0.0)

        ratio = L / T  # 滞后时间与时间常数之比

        Kp = (1.0 / K) * (T / L) * (4.0 / 3.0 + ratio / 4.0)
        Ti = L * (32.0 + 6.0 * ratio) / (13.0 + 8.0 * ratio)
        Td = L * 4.0 / (11.0 + 2.0 * ratio)

        Ki = Kp / Ti if Ti > 0 else 0.0
        Kd = Kp * Td

        return PIDParams(kp=Kp, ki=Ki, kd=Kd)

    def compute_pi(self) -> PIDParams:
        """计算 PI 参数 (Kd=0)。"""
        K, L, T = self.model.K, self.model.L, self.model.T

        if L == 0:
            return PIDParams(kp=1.0 / K, ki=1.0 / (K * T), kd=0.0)

        ratio = L / T

        Kp = (1.0 / K) * (T / L) * (4.0 / 3.0 + ratio / 4.0)
        Ti = L * (32.0 + 6.0 * ratio) / (13.0 + 8.0 * ratio)

        Ki = Kp / Ti if Ti > 0 else 0.0

        return PIDParams(kp=Kp, ki=Ki, kd=0.0)

    def compute_p(self) -> PIDParams:
        """计算纯 P 控制器参数。"""
        K, L, T = self.model.K, self.model.L, self.model.T

        if L == 0:
            return PIDParams(kp=1.0 / K, ki=0.0, kd=0.0)

        ratio = L / T
        Kp = (1.0 / K) * (T / L) * (4.0 / 3.0 + ratio / 4.0)

        return PIDParams(kp=Kp, ki=0.0, kd=0.0)


class ZieglerNichols:
    """
    Ziegler-Nichols 调参算法。

    支持两种模式:
    1. 开环模式: 基于 FOPTD 模型的过程反应曲线法
    2. 闭环模式: 基于临界增益 Ku 和临界周期 Tu 的极限灵敏度法

    开环公式 (反应曲线法):
      Kp = 1.2 * T / (K * L)
      Ti = 2 * L
      Td = 0.5 * L

    闭环公式 (极限灵敏度法):
      Kp = 0.6 * Ku
      Ti = 0.5 * Tu
      Td = 0.125 * Tu
    """

    def __init__(self, model: Optional[FOPTDModel] = None,
                 Ku: Optional[float] = None, Tu: Optional[float] = None):
        """
        初始化 Ziegler-Nichols 调参器。

        Args:
            model: FOPTD 模型 (开环模式)
            Ku: 临界增益 (闭环模式)
            Tu: 临界周期 (闭环模式)
        """
        self.model = model
        self.Ku = Ku
        self.Tu = Tu

        if model is None and (Ku is None or Tu is None):
            raise ValueError("需要提供 FOPTD 模型或临界增益/周期参数")
        if model is not None:
            model.validate()

    def compute_pid_open_loop(self) -> PIDParams:
        """开环模式: 基于 FOPTD 模型计算 PID 参数。"""
        K, L, T = self.model.K, self.model.L, self.model.T

        if L == 0:
            return PIDParams(kp=1.0 / K, ki=1.0 / (2.0 * K * T), kd=0.0)

        Kp = 1.2 * T / (K * L)
        Ti = 2.0 * L
        Td = 0.5 * L

        Ki = Kp / Ti if Ti > 0 else 0.0
        Kd = Kp * Td

        return PIDParams(kp=Kp, ki=Ki, kd=Kd)

    def compute_pid_closed_loop(self) -> PIDParams:
        """闭环模式: 基于 Ku 和 Tu 计算 PID 参数。"""
        Ku, Tu = self.Ku, self.Tu

        Kp = 0.6 * Ku
        Ti = 0.5 * Tu
        Td = 0.125 * Tu

        Ki = Kp / Ti if Ti > 0 else 0.0
        Kd = Kp * Td

        return PIDParams(kp=Kp, ki=Ki, kd=Kd)

    def compute_pi_open_loop(self) -> PIDParams:
        """开环模式: 基于 FOPTD 模型计算 PI 参数。"""
        K, L, T = self.model.K, self.model.L, self.model.T

        if L == 0:
            return PIDParams(kp=1.0 / K, ki=1.0 / (K * T), kd=0.0)

        Kp = 0.9 * T / (K * L)
        Ti = L / 0.3  # = 3.333 * L

        Ki = Kp / Ti if Ti > 0 else 0.0

        return PIDParams(kp=Kp, ki=Ki, kd=0.0)

    def compute_pid(self) -> PIDParams:
        """自动选择模式计算 PID 参数。"""
        if self.model is not None:
            return self.compute_pid_open_loop()
        else:
            return self.compute_pid_closed_loop()


class GridSearch:
    def __init__(self, kp_range=(10,200), ki_range=(0,10), kd_range=(0,50), steps=10):
        self.kp_range, self.ki_range, self.kd_range, self.steps = kp_range, ki_range, kd_range, steps
    
    def search(self, evaluate):
        results = []
        for kp in np.linspace(*self.kp_range, self.steps):
            for ki in np.linspace(*self.ki_range, self.steps):
                for kd in np.linspace(*self.kd_range, self.steps):
                    p = PIDParams(kp, ki, kd)
                    results.append(TuningResult(p, evaluate(p), 'grid'))
        results.sort(key=lambda x: x.score)
        return results

class GeneticAlgorithm:
    def __init__(self, kp_range=(10,200), ki_range=(0,10), kd_range=(0,50), pop=50, gens=100, mut=0.1):
        self.kp_range, self.ki_range, self.kd_range = kp_range, ki_range, kd_range
        self.pop, self.gens, self.mut = pop, gens, mut
    
    def optimize(self, evaluate):
        population = [PIDParams(random.uniform(*self.kp_range), random.uniform(*self.ki_range), random.uniform(*self.kd_range)) for _ in range(self.pop)]
        best = []
        for g in range(self.gens):
            scored = sorted([(p, evaluate(p)) for p in population], key=lambda x: x[1])
            best.append(TuningResult(scored[0][0], scored[0][1], 'ga'))
            survivors = [p for p, _ in scored[:self.pop//2]]
            new_pop = survivors.copy()
            while len(new_pop) < self.pop:
                p1, p2 = random.choice(survivors), random.choice(survivors)
                child = PIDParams(p1.kp if random.random()<0.5 else p2.kp, p1.ki if random.random()<0.5 else p2.ki, p1.kd if random.random()<0.5 else p2.kd)
                if random.random() < self.mut:
                    child = PIDParams(np.clip(child.kp+random.gauss(0,child.kp*0.1),*self.kp_range), np.clip(child.ki+random.gauss(0,0.1),*self.ki_range), np.clip(child.kd+random.gauss(0,0.1),*self.kd_range))
                new_pop.append(child)
            population = new_pop
        return best

"""pid_engine.py 单元测试。"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(__file__))

from pid_engine import PIDController


class TestPIDControllerInit:
    """测试 PIDController 初始化。"""

    def test_default_params(self):
        pid = PIDController()
        assert pid.kp == 1.5
        assert pid.ki == 1.5
        assert pid.kd == 0.06
        assert pid.dt == 0.005
        assert pid.out_min == -800.0
        assert pid.out_max == 800.0
        assert pid.d_filter_tau == 0.05
        assert pid.sp_weight == 1.0
        assert pid.deadband == 2.0

    def test_custom_params(self):
        pid = PIDController(kp=10.0, ki=2.0, kd=0.5, dt=0.01,
                            out_min=-500.0, out_max=500.0,
                            d_tau=0.1, sp_weight=0.8, deadband=1.0)
        assert pid.kp == 10.0
        assert pid.ki == 2.0
        assert pid.kd == 0.5
        assert pid.dt == 0.01
        assert pid.out_min == -500.0
        assert pid.out_max == 500.0
        assert pid.d_filter_tau == 0.1
        assert pid.sp_weight == 0.8
        assert pid.deadband == 1.0

    def test_initial_state_after_init(self):
        pid = PIDController()
        assert pid.integrator == 0.0
        assert pid.prev_error == 0.0
        assert pid.prev_measurement == 0.0
        assert pid.d_filtered == 0.0
        assert pid.output == 0.0


class TestPIDControllerReset:
    """测试 PIDController reset。"""

    def test_reset_clears_state(self):
        pid = PIDController()
        pid.compute(10.0, 0.0)
        pid.integrator = 5.0
        pid.prev_error = 3.0
        pid.d_filtered = 2.0
        pid.output = 100.0
        pid.reset()
        assert pid.integrator == 0.0
        assert pid.prev_error == 0.0
        assert pid.prev_measurement == 0.0
        assert pid.d_filtered == 0.0
        assert pid.output == 0.0

    def test_reset_after_compute(self):
        pid = PIDController()
        pid.compute(10.0, 5.0)
        pid.reset()
        assert pid.integrator == 0.0
        assert pid.prev_measurement == 0.0


class TestPIDControllerCompute:
    """测试 PIDController compute。"""

    def test_deadband_zero_error(self):
        pid = PIDController(deadband=2.0)
        output = pid.compute(10.0, 10.0)
        assert output == 0.0

    def test_deadband_small_error(self):
        pid = PIDController(deadband=5.0)
        output = pid.compute(10.0, 12.0)  # error = -2, within deadband
        assert output == 0.0

    def test_deadband_exact_boundary(self):
        pid = PIDController(deadband=2.0)
        output_pos = pid.compute(10.0, 8.0)  # error = 2, NOT within deadband
        output_neg = pid.compute(10.0, 12.0)  # error = -2, NOT within deadband
        # Both are outside deadband (boundary is exclusive)
        assert output_pos != 0.0 or output_neg != 0.0

    def test_proportional_only(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, deadband=0.0)
        output = pid.compute(100.0, 0.0)
        # proportional = kp * (sp_weight * setpoint - measurement)
        # = 2.0 * (1.0 * 100.0 - 0.0) = 200.0
        assert output == 200.0

    def test_integral_accumulation(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, deadband=0.0, dt=0.01)
        pid.compute(100.0, 0.0)  # error=100, integrator += 100*0.01=1.0
        assert pid.integrator == 1.0
        pid.compute(100.0, 0.0)  # error=100, integrator += 100*0.01=2.0
        assert pid.integrator == 2.0

    def test_integral_anti_windup_max(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, deadband=0.0,
                            out_min=-100.0, out_max=100.0, dt=1.0)
        # Large positive error for many steps
        for _ in range(200):
            pid.compute(1000.0, 0.0)
        assert pid.integrator <= 100.0

    def test_integral_anti_windup_min(self):
        pid = PIDController(kp=0.0, ki=1.0, kd=0.0, deadband=0.0,
                            out_min=-100.0, out_max=100.0, dt=1.0)
        for _ in range(200):
            pid.compute(-1000.0, 0.0)
        assert pid.integrator >= -100.0

    def test_output_clamp_max(self):
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, deadband=0.0, out_max=50.0)
        output = pid.compute(100.0, 0.0)
        assert output == 50.0

    def test_output_clamp_min(self):
        pid = PIDController(kp=100.0, ki=0.0, kd=0.0, deadband=0.0, out_min=-50.0)
        output = pid.compute(-100.0, 0.0)
        # proportional = kp * (sp_weight * sp - meas) = 100 * (-100 - 0) = -10000
        assert output == -50.0

    def test_derivative_on_measurement(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, deadband=0.0, dt=1.0, d_tau=0.0)
        pid.compute(0.0, 0.0)
        output = pid.compute(0.0, 10.0)
        # raw_derivative = (prev_measurement - measurement) / dt = (0 - 10)/1 = -10
        # alpha = dt/(dt+d_tau) = 1.0
        # d_filtered = 1.0 * (-10) = -10
        # derivative = kd * d_filtered = -10
        assert output == -10.0

    def test_derivative_filter_smoothing(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=1.0, deadband=0.0, dt=1.0, d_tau=1.0)
        pid.compute(0.0, 0.0)
        output = pid.compute(0.0, 10.0)
        # raw_derivative = (0-10)/1 = -10
        # alpha = 1/(1+1) = 0.5
        # d_filtered = 0.5*(-10) + 0.5*0 = -5
        # derivative = 1.0 * (-5) = -5
        assert output == -5.0

    def test_setpoint_weighting(self):
        pid1 = PIDController(kp=1.0, ki=0.0, kd=0.0, deadband=0.0, sp_weight=1.0)
        pid2 = PIDController(kp=1.0, ki=0.0, kd=0.0, deadband=0.0, sp_weight=0.5)
        out1 = pid1.compute(100.0, 0.0)
        out2 = pid2.compute(100.0, 0.0)
        # out1 = 1.0 * (1.0*100 - 0) = 100
        # out2 = 1.0 * (0.5*100 - 0) = 50
        assert out1 == 100.0
        assert out2 == 50.0

    def test_output_stored(self):
        pid = PIDController(kp=2.0, ki=0.0, kd=0.0, deadband=0.0)
        out = pid.compute(10.0, 0.0)
        assert pid.output == out

    def test_prev_measurement_stored(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0, deadband=0.0)
        pid.compute(0.0, 5.0)
        assert pid.prev_measurement == 5.0

    def test_prev_error_stored(self):
        pid = PIDController(kp=0.0, ki=0.0, kd=0.0, deadband=0.0)
        pid.compute(10.0, 3.0)
        assert pid.prev_error == 7.0


class TestPIDControllerUpdateParams:
    """测试 update_params。"""

    def test_update_kp(self):
        pid = PIDController(kp=1.0)
        pid.update_params({'kp': 5.0})
        assert pid.kp == 5.0

    def test_update_ki(self):
        pid = PIDController(ki=1.0)
        pid.update_params({'ki': 3.0})
        assert pid.ki == 3.0

    def test_update_kd(self):
        pid = PIDController(kd=0.1)
        pid.update_params({'kd': 0.5})
        assert pid.kd == 0.5

    def test_update_d_tau(self):
        pid = PIDController(d_tau=0.05)
        pid.update_params({'d_tau': 0.1})
        assert pid.d_filter_tau == 0.1

    def test_update_sp_weight(self):
        pid = PIDController(sp_weight=1.0)
        pid.update_params({'sp_weight': 0.8})
        assert pid.sp_weight == 0.8

    def test_update_deadband(self):
        pid = PIDController(deadband=2.0)
        pid.update_params({'deadband': 5.0})
        assert pid.deadband == 5.0

    def test_update_multiple_params(self):
        pid = PIDController(kp=1.0, ki=1.0, kd=0.1)
        pid.update_params({'kp': 10.0, 'ki': 5.0, 'kd': 1.0})
        assert pid.kp == 10.0
        assert pid.ki == 5.0
        assert pid.kd == 1.0

    def test_update_ignores_unknown_keys(self):
        pid = PIDController(kp=1.0)
        pid.update_params({'unknown_key': 999.0})
        assert pid.kp == 1.0


class TestPIDControllerGetParams:
    """测试 get_params。"""

    def test_returns_all_params(self):
        pid = PIDController()
        params = pid.get_params()
        expected_keys = {'kp', 'ki', 'kd', 'dt', 'out_min', 'out_max',
                         'd_tau', 'sp_weight', 'deadband'}
        assert set(params.keys()) == expected_keys

    def test_values_match(self):
        pid = PIDController(kp=7.7, ki=3.3, kd=0.42)
        params = pid.get_params()
        assert params['kp'] == 7.7
        assert params['ki'] == 3.3
        assert params['kd'] == 0.42

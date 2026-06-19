"""plant_model.py 单元测试。"""

import sys
import os
import math

sys.path.insert(0, os.path.dirname(__file__))

from plant_model import ServoPlant


class TestServoPlantInit:
    """测试 ServoPlant 初始化。"""

    def test_default_params(self):
        plant = ServoPlant()
        assert plant.dt == 0.005
        assert plant.servo_tau == 0.05
        assert plant.servo_angle == 0.0
        assert plant.servo_deadband == 0.5
        assert plant.omega_n == 2.0 * math.pi * 10.0
        assert plant.zeta == 0.7
        assert plant.platform_angle == 0.0
        assert plant.platform_velocity == 0.0
        assert plant.noise_sigma == 0.1

    def test_custom_dt(self):
        plant = ServoPlant(dt=0.01)
        assert plant.dt == 0.01


class TestServoPlantReset:
    """测试 ServoPlant reset。"""

    def test_reset_clears_state(self):
        plant = ServoPlant()
        plant.servo_angle = 10.0
        plant.platform_angle = 5.0
        plant.platform_velocity = 3.0
        plant.reset()
        assert plant.servo_angle == 0.0
        assert plant.platform_angle == 0.0
        assert plant.platform_velocity == 0.0

    def test_reset_after_update(self):
        plant = ServoPlant()
        plant.update(500.0)
        plant.reset()
        assert plant.servo_angle == 0.0
        assert plant.platform_angle == 0.0
        assert plant.platform_velocity == 0.0


class TestServoPlantUpdate:
    """测试 ServoPlant update。"""

    def test_zero_input(self):
        plant = ServoPlant()
        result = plant.update(0.0)
        assert isinstance(result, float)
        # With zero input, servo should stay near 0
        assert abs(plant.servo_angle) < 0.1

    def test_positive_output_maps_to_positive_angle(self):
        plant = ServoPlant()
        plant.update(800.0)  # max PWM → 25 degrees
        assert plant.servo_angle > 0

    def test_negative_output_maps_to_negative_angle(self):
        plant = ServoPlant()
        plant.update(-800.0)  # min PWM → -25 degrees
        assert plant.servo_angle < 0

    def test_output_mapping_800_to_25(self):
        plant = ServoPlant()
        for _ in range(100):
            plant.update(800.0)
        # servo_angle should converge toward 25 degrees
        assert 20.0 < plant.servo_angle <= 25.0

    def test_output_mapping_neg800_to_neg25(self):
        plant = ServoPlant()
        for _ in range(100):
            plant.update(-800.0)
        assert -25.0 <= plant.servo_angle < -20.0

    def test_servo_inertia(self):
        plant = ServoPlant(dt=0.05)
        # Servo should not instantly reach target
        plant.update(800.0)
        assert plant.servo_angle < 25.0  # still approaching

    def test_measurement_has_noise(self):
        """Noise means repeated calls with same input give slightly different results."""
        plant = ServoPlant()
        results = [plant.update(500.0) for _ in range(20)]
        # The platform dynamics also change, but noise should create variation
        assert len(set(results)) > 1

    def test_platform_angle_response(self):
        plant = ServoPlant()
        for _ in range(100):
            plant.update(800.0)
        # Platform should move toward positive angle
        assert plant.platform_angle > 0

    def test_negative_input_response(self):
        plant = ServoPlant()
        for _ in range(100):
            plant.update(-800.0)
        assert plant.platform_angle < 0

    def test_multiple_steps_convergence(self):
        plant = ServoPlant()
        for _ in range(500):
            plant.update(400.0)
        # Platform should be moving toward target
        assert abs(plant.platform_angle) > 0


class TestServoPlantGetState:
    """测试 get_state。"""

    def test_returns_all_keys(self):
        plant = ServoPlant()
        state = plant.get_state()
        expected = {'servo_angle', 'platform_angle', 'platform_velocity'}
        assert set(state.keys()) == expected

    def test_values_match(self):
        plant = ServoPlant()
        plant.servo_angle = 5.0
        plant.platform_angle = 3.0
        plant.platform_velocity = 2.0
        state = plant.get_state()
        assert state['servo_angle'] == 5.0
        assert state['platform_angle'] == 3.0
        assert state['platform_velocity'] == 2.0

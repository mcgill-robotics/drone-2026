"""Tests for configuration dataclasses: default values and factory helpers."""
import pytest

from config.avoidance_config import AvoidanceConfig
from config.environment_config import EnvironmentConfig
from config.navigation_config import NavigationConfig
from config.physics_config import PhysicsConfig
from config.robot_config import RobotConfig, StuckDetectionConfig, VisualizationConfig


# ---------------------------------------------------------------------------
# PhysicsConfig defaults
# ---------------------------------------------------------------------------

class TestPhysicsConfig:
    def test_default_mass(self):
        assert PhysicsConfig().mass == pytest.approx(1.0)

    def test_default_max_force(self):
        assert PhysicsConfig().max_force == pytest.approx(55.0)

    def test_default_max_speed(self):
        assert PhysicsConfig().max_speed == pytest.approx(30.0)

    def test_default_dt(self):
        assert PhysicsConfig().dt == pytest.approx(0.1)

    def test_default_air_resistance(self):
        assert PhysicsConfig().air_resistance == pytest.approx(0.98)

    def test_custom_values_accepted(self):
        cfg = PhysicsConfig(mass=5.0, max_force=200.0, dt=0.05)
        assert cfg.mass == pytest.approx(5.0)
        assert cfg.max_force == pytest.approx(200.0)
        assert cfg.dt == pytest.approx(0.05)


# ---------------------------------------------------------------------------
# NavigationConfig defaults
# ---------------------------------------------------------------------------

class TestNavigationConfig:
    def test_default_arrival_radius(self):
        assert NavigationConfig().arrival_radius == pytest.approx(10.0)

    def test_default_corner_radius(self):
        assert NavigationConfig().corner_radius == pytest.approx(40.0)

    def test_default_loop_threshold(self):
        assert NavigationConfig().loop_threshold == pytest.approx(20.0)

    def test_default_slow_radius(self):
        assert NavigationConfig().slow_radius == pytest.approx(150.0)


# ---------------------------------------------------------------------------
# EnvironmentConfig defaults
# ---------------------------------------------------------------------------

class TestEnvironmentConfig:
    def test_default_scan_area_radius(self):
        assert EnvironmentConfig().scan_area_radius == pytest.approx(160.0)

    def test_default_density_thresholds(self):
        cfg = EnvironmentConfig()
        assert cfg.density_threshold_dense == pytest.approx(0.08)
        assert cfg.density_threshold_moderate == pytest.approx(0.03)

    def test_default_corridor_threshold(self):
        assert EnvironmentConfig().corridor_confidence_threshold == pytest.approx(0.6)


# ---------------------------------------------------------------------------
# AvoidanceConfig
# ---------------------------------------------------------------------------

class TestAvoidanceConfig:
    def test_all_environment_types_in_memory_thresholds(self):
        cfg = AvoidanceConfig()
        for env in ('SPARSE', 'MODERATE', 'DENSE', 'CORRIDOR'):
            assert env in cfg.memory_thresholds

    def test_all_environment_types_in_smoothing_factors(self):
        cfg = AvoidanceConfig()
        for env in ('SPARSE', 'MODERATE', 'DENSE', 'CORRIDOR'):
            assert env in cfg.smoothing_factors

    def test_all_environment_types_have_force_profiles(self):
        cfg = AvoidanceConfig()
        assert cfg.sparse is not None
        assert cfg.moderate is not None
        assert cfg.dense is not None
        assert cfg.corridor is not None

    def test_free_flight_smoothing_exists(self):
        assert AvoidanceConfig().free_flight_smoothing > 0

    def test_smoothing_factors_are_between_0_and_1(self):
        for v in AvoidanceConfig().smoothing_factors.values():
            assert 0 < v <= 1.0


# ---------------------------------------------------------------------------
# StuckDetectionConfig defaults
# ---------------------------------------------------------------------------

class TestStuckDetectionConfig:
    def test_default_velocity_threshold(self):
        assert StuckDetectionConfig().velocity_threshold == pytest.approx(2.0)

    def test_default_stuck_counter_threshold(self):
        assert StuckDetectionConfig().stuck_counter_threshold == 50

    def test_default_stuck_reset_threshold(self):
        assert StuckDetectionConfig().stuck_reset_threshold == 100

    def test_default_escape_force_mult(self):
        assert StuckDetectionConfig().escape_force_mult == pytest.approx(0.3)


# ---------------------------------------------------------------------------
# VisualizationConfig defaults
# ---------------------------------------------------------------------------

class TestVisualizationConfig:
    def test_default_trail_length(self):
        assert VisualizationConfig().trail_length == 100

    def test_env_colors_has_all_types(self):
        colors = VisualizationConfig().env_colors
        for env in ('SPARSE', 'MODERATE', 'DENSE', 'CORRIDOR'):
            assert env in colors

    def test_avoid_dist_positive(self):
        assert VisualizationConfig().avoid_dist > 0


# ---------------------------------------------------------------------------
# RobotConfig aggregation and from_parameters
# ---------------------------------------------------------------------------

class TestRobotConfig:
    def test_default_config_has_all_sub_configs(self):
        cfg = RobotConfig()
        assert cfg.physics is not None
        assert cfg.navigation is not None
        assert cfg.environment is not None
        assert cfg.avoidance is not None
        assert cfg.stuck_detection is not None
        assert cfg.visualization is not None

    def test_from_parameters_sets_physics_fields(self):
        cfg = RobotConfig.from_parameters(dt=0.2, m=3.0, f_max=80.0, diam=25.0)
        assert cfg.physics.dt == pytest.approx(0.2)
        assert cfg.physics.mass == pytest.approx(3.0)
        assert cfg.physics.max_force == pytest.approx(80.0)
        assert cfg.physics.diameter == pytest.approx(25.0)

    def test_from_parameters_leaves_other_configs_as_defaults(self):
        cfg = RobotConfig.from_parameters()
        assert cfg.navigation.arrival_radius == NavigationConfig().arrival_radius
        assert cfg.stuck_detection.stuck_counter_threshold == StuckDetectionConfig().stuck_counter_threshold

    def test_from_parameters_default_values(self):
        cfg = RobotConfig.from_parameters()
        assert cfg.physics.dt == pytest.approx(0.1)
        assert cfg.physics.mass == pytest.approx(1.0)
        assert cfg.physics.max_force == pytest.approx(55.0)

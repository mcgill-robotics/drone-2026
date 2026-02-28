"""Tests for PhysicsEngine: force application, clamping, damping, and position update."""
import pygame
import pytest

from config.physics_config import PhysicsConfig
from core.physics_engine import PhysicsEngine


def make_engine(dt=1.0, mass=1.0, max_force=100.0, max_speed=50.0, air_resistance=1.0):
    """Helper: build a PhysicsEngine with simple, easy-to-reason-about values."""
    config = PhysicsConfig(
        dt=dt,
        mass=mass,
        max_force=max_force,
        max_speed=max_speed,
        air_resistance=air_resistance,
    )
    return PhysicsEngine(pygame.Vector2(0, 0), pygame.Vector2(0, 0), config)


# ---------------------------------------------------------------------------
# apply_force
# ---------------------------------------------------------------------------

def test_zero_force_produces_no_velocity():
    engine = make_engine()
    engine.apply_force(pygame.Vector2(0, 0))
    assert engine.velocity.length() == pytest.approx(0.0)


def test_force_accelerates_at_correct_rate():
    # v += (F / m) * dt  →  v += (10 / 1) * 1 = 10
    engine = make_engine(dt=1.0, mass=1.0, air_resistance=1.0)
    engine.apply_force(pygame.Vector2(10, 0))
    assert engine.velocity.x == pytest.approx(10.0)


def test_force_scales_with_mass():
    # v += (F / m) * dt  →  v += (20 / 2) * 1 = 10
    engine = make_engine(dt=1.0, mass=2.0, air_resistance=1.0, max_force=100.0, max_speed=100.0)
    engine.apply_force(pygame.Vector2(20, 0))
    assert engine.velocity.x == pytest.approx(10.0)


def test_force_scales_with_dt():
    # v += (10 / 1) * 0.5 = 5
    engine = make_engine(dt=0.5, mass=1.0, air_resistance=1.0)
    engine.apply_force(pygame.Vector2(10, 0))
    assert engine.velocity.x == pytest.approx(5.0)


def test_force_clamped_to_max_force():
    # Input 1000 N clamped to 5 N  →  v += 5/1 * 1 = 5
    engine = make_engine(dt=1.0, mass=1.0, max_force=5.0, max_speed=100.0, air_resistance=1.0)
    engine.apply_force(pygame.Vector2(1000, 0))
    assert engine.velocity.x == pytest.approx(5.0)


def test_air_resistance_dampens_existing_velocity():
    # Existing velocity (10, 0), zero force, damping 0.5  →  v = 5
    engine = make_engine(dt=1.0, mass=1.0, air_resistance=0.5, max_force=0.0)
    engine.velocity = pygame.Vector2(10, 0)
    engine.apply_force(pygame.Vector2(0, 0))
    assert engine.velocity.x == pytest.approx(5.0)


def test_velocity_clamped_to_max_speed():
    engine = make_engine(dt=1.0, mass=1.0, max_force=1000.0, max_speed=5.0, air_resistance=1.0)
    engine.apply_force(pygame.Vector2(1000, 0))
    assert engine.velocity.length() == pytest.approx(5.0)


def test_no_velocity_overshoot_in_2d():
    engine = make_engine(dt=1.0, mass=1.0, max_force=100.0, max_speed=10.0, air_resistance=1.0)
    engine.apply_force(pygame.Vector2(100, 100))
    assert engine.velocity.length() == pytest.approx(10.0)


# ---------------------------------------------------------------------------
# update (position integration)
# ---------------------------------------------------------------------------

def test_update_advances_position_by_velocity_times_dt():
    engine = make_engine(dt=1.0)
    engine.velocity = pygame.Vector2(5, 3)
    engine.update()
    assert engine.position.x == pytest.approx(5.0)
    assert engine.position.y == pytest.approx(3.0)


def test_update_dt_scales_displacement():
    engine = make_engine(dt=0.5)
    engine.velocity = pygame.Vector2(10, 0)
    engine.update()
    assert engine.position.x == pytest.approx(5.0)


def test_update_zero_velocity_no_movement():
    engine = make_engine(dt=1.0)
    engine.update()
    assert engine.position.length() == pytest.approx(0.0)


def test_update_accumulates_position_over_multiple_steps():
    engine = make_engine(dt=1.0)
    engine.velocity = pygame.Vector2(2, 0)
    for _ in range(5):
        engine.update()
    assert engine.position.x == pytest.approx(10.0)

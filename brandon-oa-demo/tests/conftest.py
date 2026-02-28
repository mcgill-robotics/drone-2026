"""Shared pytest fixtures and setup for the drone-2026 test suite."""
import os
import sys

import pygame
import pytest

# Ensure the project root is on sys.path so imports work from any working directory.
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))


@pytest.fixture(scope="session", autouse=True)
def pygame_session():
    """Initialize pygame once for the entire test session (no display needed)."""
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    pygame.init()
    yield
    pygame.quit()


@pytest.fixture
def simple_robot(pygame_session):
    """A robot with no obstacles and no waypoints."""
    from robot import Robot
    return Robot(100, 100, obstacles=[], course=[])


@pytest.fixture
def robot_with_waypoint(pygame_session):
    """A robot with a single distant waypoint."""
    from robot import Robot
    course = [pygame.Vector2(500, 300)]
    return Robot(100, 100, obstacles=[], course=course)

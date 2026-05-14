"""Obstacle-avoidance interface used by MissionController.safe_goto."""


class Avoider:
    """Interface for obstacle avoidance.

    Implementations decide whether the straight line from `current` to
    `target` is safe, and if not, return an intermediate waypoint to head
    toward instead. The mission FSM re-enters safe_goto each tick, so
    returning a sub-waypoint is enough — the next tick replans from the
    new position.
    """

    def path_clear(self, current, target):
        raise NotImplementedError

    def get_safe_waypoint(self, current, target, boundary):
        """Return the next waypoint to head toward, or None if no safe path
        exists right now (the controller should hover and re-check next tick)."""
        raise NotImplementedError


class NullAvoider(Avoider):
    """No-op avoider: every path is clear, every target is the waypoint."""

    def path_clear(self, current, target):
        return True

    def get_safe_waypoint(self, current, target, boundary):
        return target

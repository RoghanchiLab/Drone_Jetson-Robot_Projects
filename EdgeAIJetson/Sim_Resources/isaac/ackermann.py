# isaac/ackermann.py
"""Bicycle-model Ackermann conversion. Pure Python — shared by Isaac Lab env and tests."""
import math


def compute_ackermann(speed, steer, wheelbase, track, wheel_radius):
    """(speed m/s, steer rad of virtual center wheel) ->
    (left_steer, right_steer, left_wheel_omega, right_wheel_omega)."""
    if abs(steer) < 1e-9:
        w = speed / wheel_radius
        return 0.0, 0.0, w, w
    radius = wheelbase / math.tan(abs(steer))         # rear-axle turn radius
    inner = math.atan(wheelbase / (radius - track / 2.0))
    outer = math.atan(wheelbase / (radius + track / 2.0))
    v_inner = speed * (radius - track / 2.0) / radius
    v_outer = speed * (radius + track / 2.0) / radius
    if steer > 0:  # left turn: left wheels are inner
        return inner, outer, v_inner / wheel_radius, v_outer / wheel_radius
    return -outer, -inner, v_outer / wheel_radius, v_inner / wheel_radius

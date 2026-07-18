# tests/test_ackermann.py
import math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "isaac"))
from ackermann import compute_ackermann

L, W, R = 0.213, 0.222, 0.0505  # wheelbase, track, wheel radius


def test_straight_line():
    ls, rs, lw, rw = compute_ackermann(1.0, 0.0, L, W, R)
    assert ls == rs == 0.0
    assert abs(lw - 1.0 / R) < 1e-9 and lw == rw


def test_left_turn_inner_steers_more():
    ls, rs, lw, rw = compute_ackermann(1.0, 0.3, L, W, R)
    assert ls > rs > 0            # left turn: left (inner) wheel steers more
    assert rw > lw                # outer rear wheel spins faster
    turn_radius = L / math.tan(0.3)
    assert abs(math.tan(ls) - L / (turn_radius - W / 2)) < 1e-9
    assert abs(math.tan(rs) - L / (turn_radius + W / 2)) < 1e-9


def test_right_turn_mirrors_left():
    l1 = compute_ackermann(1.0, 0.3, L, W, R)
    l2 = compute_ackermann(1.0, -0.3, L, W, R)
    assert abs(l2[0] + l1[1]) < 1e-12 and abs(l2[1] + l1[0]) < 1e-12
    assert abs(l2[2] - l1[3]) < 1e-12 and abs(l2[3] - l1[2]) < 1e-12


def test_reverse_speed():
    _, _, lw, rw = compute_ackermann(-0.5, 0.0, L, W, R)
    assert lw == rw == -0.5 / R

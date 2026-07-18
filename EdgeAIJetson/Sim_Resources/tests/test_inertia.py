# tests/test_inertia.py
import math, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "urdf"))
from inertia import box_inertia, cylinder_inertia, part_inertia, composite

def test_box_inertia_cube():
    # 12 kg unit cube: ixx = 12/12*(1+1) = 2 on every axis
    assert box_inertia(12.0, (1.0, 1.0, 1.0)) == [2.0, 2.0, 2.0, 0.0, 0.0, 0.0]

def test_cylinder_inertia_z_axis():
    m, r, h = 2.0, 0.5, 2.0
    ia = m * (3 * r * r + h * h) / 12.0   # 2*(0.75+4)/12
    ip = m * r * r / 2.0                   # 0.25
    assert cylinder_inertia(m, r, h, "z") == [ia, ia, ip, 0.0, 0.0, 0.0]

def test_cylinder_axis_permutation():
    iz = cylinder_inertia(1.0, 0.1, 0.4, "z")
    iy = cylinder_inertia(1.0, 0.1, 0.4, "y")
    assert iy == [iz[0], iz[2], iz[0], 0.0, 0.0, 0.0]  # spin axis moved to Y

def test_composite_two_point_masses():
    # Two 1kg near-point masses at x=±1: com at 0, Iyy = Izz = 2*m*d^2 = 2
    tiny = [0.0] * 6
    m, com, I = composite([(1.0, (1.0, 0, 0), tiny), (1.0, (-1.0, 0, 0), tiny)])
    assert m == 2.0
    assert com == [0.0, 0.0, 0.0]
    assert abs(I[1] - 2.0) < 1e-12 and abs(I[2] - 2.0) < 1e-12 and abs(I[0]) < 1e-12

def test_composite_products_of_inertia():
    # Masses at (1,1,0) and (-1,-1,0): Ixy = -sum(m*dx*dy) = -2
    tiny = [0.0] * 6
    _, com, I = composite([(1.0, (1.0, 1.0, 0), tiny), (1.0, (-1.0, -1.0, 0), tiny)])
    assert com == [0.0, 0.0, 0.0]
    assert abs(I[3] + 2.0) < 1e-12

def test_part_inertia_dispatch():
    box = {"type": "box", "size": [0.1, 0.2, 0.3], "pos": [0, 0, 0], "mass": 1.0}
    cyl = {"type": "cylinder", "radius": 0.05, "length": 0.04, "axis": "y", "pos": [0, 0, 0], "mass": 0.12}
    assert part_inertia(box) == box_inertia(1.0, (0.1, 0.2, 0.3))
    assert part_inertia(cyl) == cylinder_inertia(0.12, 0.05, 0.04, "y")

# urdf/inertia.py
"""Analytic inertia tensors for axis-aligned primitives. Order: [ixx, iyy, izz, ixy, ixz, iyz]."""


def box_inertia(m, size):
    lx, ly, lz = size
    return [m / 12.0 * (ly * ly + lz * lz),
            m / 12.0 * (lx * lx + lz * lz),
            m / 12.0 * (lx * lx + ly * ly),
            0.0, 0.0, 0.0]


def cylinder_inertia(m, r, h, axis):
    ia = m * (3.0 * r * r + h * h) / 12.0  # transverse
    ip = m * r * r / 2.0                    # about spin axis
    return {"x": [ip, ia, ia, 0.0, 0.0, 0.0],
            "y": [ia, ip, ia, 0.0, 0.0, 0.0],
            "z": [ia, ia, ip, 0.0, 0.0, 0.0]}[axis]


def part_inertia(part):
    if part["type"] == "box":
        return box_inertia(part["mass"], tuple(part["size"]))
    return cylinder_inertia(part["mass"], part["radius"], part["length"], part["axis"])


def composite(parts):
    """parts: [(mass, pos, I6-about-own-com), ...] -> (total_mass, com, I6 about com)."""
    mtot = sum(p[0] for p in parts)
    com = [sum(p[0] * p[1][i] for p in parts) / mtot for i in range(3)]
    I = [0.0] * 6
    for m, pos, i6 in parts:
        dx, dy, dz = (pos[0] - com[0], pos[1] - com[1], pos[2] - com[2])
        I[0] += i6[0] + m * (dy * dy + dz * dz)
        I[1] += i6[1] + m * (dx * dx + dz * dz)
        I[2] += i6[2] + m * (dx * dx + dy * dy)
        I[3] += i6[3] - m * dx * dy
        I[4] += i6[4] - m * dx * dz
        I[5] += i6[5] - m * dy * dz
    return mtot, com, I

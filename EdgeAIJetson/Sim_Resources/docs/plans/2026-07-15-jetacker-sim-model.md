# JetAcker Sim Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Fusion tasks (5, 6) must run in the MAIN session** — they need the Fusion MCP tools (`mcp__fusion__*`), which subagents may not have. Everything else is subagent-safe.

**Goal:** Parametric Fusion 360 model of the custom JetAcker (AGX Orin, RPLidar S2, Astra Pro Plus), exported to STL meshes + a generated URDF, with Isaac Sim import / Isaac Lab config / ROS2 graph scripts, per the approved spec `docs/2026-07-15-jetacker-sim-model-design.md`.

**Architecture:** Single source of truth `params.json` (dimensions, positions, masses). `fusion/build_jetacker.py` reads it and builds/export geometry via the Fusion API (direct-modeling, TemporaryBRepManager primitives, one component per URDF link, geometry in car-assembly coordinates with base_link at Fusion origin). `urdf/generate_urdf.py` reads the same file, computes inertia tensors analytically (primitives + parallel axis), and emits `jetacker.urdf`. Isaac scripts are authored here, run on the Isaac machine.

**Tech Stack:** Fusion 360 Python API (via Fusion MCP `fusion_mcp_execute`), Python 3 stdlib (`xml.etree`, `json`, `math`), pytest, Isaac Sim 4.5+/5.x, Isaac Lab.

**Working directory:** `/Users/nathanjones/Documents/Arduino/Mining Research Mother File/Mining-Research/EdgeAIJetson/Sim_Resources` (call it `$SIM` below; quote all paths — they contain spaces).

**Coordinate convention (used everywhere):** REP-103. X forward, Y left, Z up. `base_link` origin = rear-axle center at axle height (z = wheel_radius above ground). Fusion model origin == `base_link` frame. All params.json positions are relative to `base_link`.

---

### Task 1: Branch + scaffold

**Files:**
- Create: `$SIM/{fusion,meshes,urdf,isaac,tools,tests}/` directories

- [ ] **Step 1: Create feature branch** (repo is currently on `simplify-movement-demo`; branch from HEAD)

```bash
cd "/Users/nathanjones/Documents/Arduino/Mining Research Mother File/Mining-Research"
git status --porcelain   # note any pre-existing dirty files; do NOT commit them
git checkout -b feat/jetacker-sim-model
```

- [ ] **Step 2: Scaffold directories**

```bash
cd "EdgeAIJetson/Sim_Resources"
mkdir -p fusion meshes urdf isaac tools tests docs/plans
```

- [ ] **Step 3: Commit the spec + plan**

```bash
git add EdgeAIJetson/Sim_Resources/docs
git commit -m "docs: JetAcker sim model spec and implementation plan"
```

---

### Task 2: params.json — single source of truth

**Files:**
- Create: `$SIM/params.json`

- [ ] **Step 1: Write params.json** (values from spec §2; `(e)` estimates refined later against the real car)

```json
{
  "meta": {
    "units": "meters, kilograms, radians",
    "frame": "base_link at rear axle center, axle height. X fwd, Y left, Z up.",
    "mesh_scale_note": "Fusion STL export unit verified in Task 6; generator default 0.001 (mm)"
  },
  "wheel_radius": 0.0505,
  "wheel_width": 0.040,
  "wheel_mass": 0.12,
  "wheelbase": 0.213,
  "track": 0.222,
  "steer_limit": 0.5236,
  "steer_knuckle_mass": 0.05,
  "servo_max_torque": 2.0,
  "servo_max_speed": 6.0,
  "motor_max_torque": 1.5,
  "motor_max_speed": 40.0,
  "collision_box": { "size": [0.320, 0.190, 0.200], "pos": [0.095, 0.0, 0.075] },
  "sensor_links": {
    "laser":       { "pos": [0.150, 0.0, 0.075], "shape": { "type": "cylinder", "radius": 0.0385, "length": 0.039, "axis": "z" }, "mass": 0.185 },
    "camera_link": { "pos": [0.050, 0.0, 0.155], "shape": { "type": "box", "size": [0.030, 0.165, 0.030] }, "mass": 0.300 },
    "imu_link":    { "pos": [0.065, 0.0, 0.090], "shape": { "type": "box", "size": [0.010, 0.010, 0.002] }, "mass": 0.01 }
  },
  "base_parts": [
    { "name": "deck",            "type": "box",      "size": [0.300, 0.178, 0.004], "pos": [0.095, 0.0, 0.030],  "mass": 1.20 },
    { "name": "motors",          "type": "box",      "size": [0.080, 0.150, 0.040], "pos": [0.000, 0.0, 0.000],  "mass": 0.50 },
    { "name": "battery",         "type": "box",      "size": [0.090, 0.090, 0.030], "pos": [0.070, 0.0, 0.050],  "mass": 0.45 },
    { "name": "agx_orin",        "type": "box",      "size": [0.110, 0.110, 0.072], "pos": [0.000, 0.0, 0.100],  "mass": 1.45 },
    { "name": "expansion_board", "type": "box",      "size": [0.015, 0.100, 0.080], "pos": [0.065, 0.0, 0.100],  "mass": 0.15 },
    { "name": "pcb_stack",       "type": "box",      "size": [0.060, 0.120, 0.050], "pos": [-0.080, 0.0, 0.090], "mass": 0.20 },
    { "name": "camera_mast",     "type": "cylinder", "radius": 0.008, "length": 0.110, "axis": "z", "pos": [0.050, 0.0, 0.085], "mass": 0.05 }
  ]
}
```

- [ ] **Step 2: Validate JSON**

Run: `python3 -m json.tool "$SIM/params.json" > /dev/null && echo OK`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/params.json
git commit -m "feat(sim): parameter table for JetAcker model"
```

---

### Task 3: Inertia math (TDD)

**Files:**
- Create: `$SIM/urdf/inertia.py`
- Test: `$SIM/tests/test_inertia.py`

Inertia tensor order everywhere: `[ixx, iyy, izz, ixy, ixz, iyz]`.

- [ ] **Step 1: Write the failing tests**

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd "$SIM" && python3 -m pytest tests/test_inertia.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'inertia'`

- [ ] **Step 3: Implement inertia.py**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd "$SIM" && python3 -m pytest tests/test_inertia.py -v`
Expected: 6 passed

- [ ] **Step 5: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/urdf/inertia.py EdgeAIJetson/Sim_Resources/tests/test_inertia.py
git commit -m "feat(sim): analytic inertia math with tests"
```

---

### Task 4: URDF generator (TDD)

**Files:**
- Create: `$SIM/urdf/generate_urdf.py`
- Test: `$SIM/tests/test_generate_urdf.py`
- Output: `$SIM/urdf/jetacker.urdf` (generated, still committed so the Isaac machine can use it directly)

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_generate_urdf.py
import json, math, os, sys
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "urdf"))
from generate_urdf import build_urdf

SIM = os.path.join(os.path.dirname(__file__), "..")
PARAMS = json.load(open(os.path.join(SIM, "params.json")))


def tree():
    return ET.fromstring(build_urdf(PARAMS, mesh_prefix="../meshes/", mesh_scale=0.001))


def joints(root):
    return {j.get("name"): j for j in root.findall("joint")}


def test_valid_xml_and_link_names():
    root = tree()
    names = {l.get("name") for l in root.findall("link")}
    assert names == {
        "base_footprint", "base_link",
        "front_left_steering_link", "front_right_steering_link",
        "front_left_wheel_link", "front_right_wheel_link",
        "rear_left_wheel_link", "rear_right_wheel_link",
        "laser", "camera_link",
        "camera_color_optical_frame", "camera_depth_optical_frame", "imu_link",
    }


def test_joint_tree_parent_child():
    j = joints(tree())
    def pc(name):
        return (j[name].find("parent").get("link"), j[name].find("child").get("link"))
    assert pc("base_joint") == ("base_footprint", "base_link")
    assert pc("front_left_steering_joint") == ("base_link", "front_left_steering_link")
    assert pc("front_left_wheel_joint") == ("front_left_steering_link", "front_left_wheel_link")
    assert pc("rear_left_wheel_joint") == ("base_link", "rear_left_wheel_link")
    assert pc("laser_joint") == ("base_link", "laser")
    assert pc("camera_color_optical_joint") == ("camera_link", "camera_color_optical_frame")


def test_joint_types_axes_limits():
    j = joints(tree())
    steer = j["front_right_steering_joint"]
    assert steer.get("type") == "revolute"
    assert steer.find("axis").get("xyz") == "0 0 1"
    lim = steer.find("limit")
    assert abs(float(lim.get("lower")) + PARAMS["steer_limit"]) < 1e-9
    assert float(lim.get("effort")) == PARAMS["servo_max_torque"]
    wheel = j["rear_right_wheel_joint"]
    assert wheel.get("type") == "continuous"
    assert wheel.find("axis").get("xyz") == "0 1 0"


def test_geometry_positions():
    j = joints(tree())
    def xyz(name):
        return [float(v) for v in j[name].find("origin").get("xyz").split()]
    assert xyz("base_joint") == [0.0, 0.0, PARAMS["wheel_radius"]]
    assert xyz("front_left_steering_joint") == [PARAMS["wheelbase"], PARAMS["track"] / 2, 0.0]
    assert xyz("rear_right_wheel_joint") == [0.0, -PARAMS["track"] / 2, 0.0]
    assert xyz("front_left_wheel_joint") == [0.0, 0.0, 0.0]


def test_optical_frame_orientation():
    j = joints(tree())
    rpy = [float(v) for v in j["camera_color_optical_joint"].find("origin").get("rpy").split()]
    # 1e-4 tolerance: values are serialized with %.6g formatting
    assert abs(rpy[0] + math.pi / 2) < 1e-4 and abs(rpy[2] + math.pi / 2) < 1e-4


def test_total_mass_matches_params():
    root = tree()
    total = sum(float(l.find("inertial/mass").get("value"))
                for l in root.findall("link") if l.find("inertial") is not None)
    expected = (sum(p["mass"] for p in PARAMS["base_parts"])
                + 4 * PARAMS["wheel_mass"] + 2 * PARAMS["steer_knuckle_mass"]
                + sum(s["mass"] for s in PARAMS["sensor_links"].values()))
    assert abs(total - expected) < 1e-6


def test_mesh_visuals_reference_prefix_and_scale():
    root = tree()
    base = next(l for l in root.findall("link") if l.get("name") == "base_link")
    mesh = base.find("visual/geometry/mesh")
    assert mesh.get("filename") == "../meshes/base_link.stl"
    assert mesh.get("scale") == "0.001 0.001 0.001"


def test_wheel_mesh_origin_cancels_world_position():
    # Meshes are exported in car-assembly coords; wheel visual origin must be -(link world pos)
    root = tree()
    fl = next(l for l in root.findall("link") if l.get("name") == "front_left_wheel_link")
    xyz = [float(v) for v in fl.find("visual/origin").get("xyz").split()]
    assert xyz == [-PARAMS["wheelbase"], -PARAMS["track"] / 2, 0.0]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd "$SIM" && python3 -m pytest tests/test_generate_urdf.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'generate_urdf'`

- [ ] **Step 3: Implement generate_urdf.py**

```python
#!/usr/bin/env python3
# urdf/generate_urdf.py
"""Generate jetacker.urdf from ../params.json. Stdlib only — no ROS required.

Meshes are exported by fusion/build_jetacker.py in car-assembly coordinates
(base_link frame at Fusion origin), so every link's visual mesh origin is the
negative of that link's position in base_link coordinates.
"""
import argparse, json, math, os
import xml.etree.ElementTree as ET
from xml.dom import minidom
from inertia import part_inertia, composite


def _fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def _origin(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    ET.SubElement(parent, "origin", xyz=_fmt(xyz), rpy=_fmt(rpy))


def _inertial(link, mass, com, I):
    e = ET.SubElement(link, "inertial")
    _origin(e, com)
    ET.SubElement(e, "mass", value=f"{mass:.6g}")
    ET.SubElement(e, "inertia", ixx=f"{I[0]:.9g}", iyy=f"{I[1]:.9g}", izz=f"{I[2]:.9g}",
                  ixy=f"{I[3]:.9g}", ixz=f"{I[4]:.9g}", iyz=f"{I[5]:.9g}")


def _mesh_visual(link, stl, world_pos, prefix, scale):
    v = ET.SubElement(link, "visual")
    _origin(v, [-p for p in world_pos])
    g = ET.SubElement(v, "geometry")
    ET.SubElement(g, "mesh", filename=f"{prefix}{stl}", scale=_fmt([scale] * 3))


def _cyl_collision(link, radius, length):
    c = ET.SubElement(link, "collision")
    _origin(c, rpy=(math.pi / 2, 0, 0))  # URDF cylinders are Z-axis; wheels spin about Y
    g = ET.SubElement(c, "geometry")
    ET.SubElement(g, "cylinder", radius=f"{radius}", length=f"{length}")


def _joint(robot, name, jtype, parent, child, xyz, rpy=(0, 0, 0), axis=None, limit=None):
    j = ET.SubElement(robot, "joint", name=name, type=jtype)
    _origin(j, xyz, rpy)
    ET.SubElement(j, "parent", link=parent)
    ET.SubElement(j, "child", link=child)
    if axis:
        ET.SubElement(j, "axis", xyz=_fmt(axis))
    if limit:
        ET.SubElement(j, "limit", **{k: f"{v:.6g}" for k, v in limit.items()})


def build_urdf(p, mesh_prefix="../meshes/", mesh_scale=0.001):
    wr, ww, wb, tr = p["wheel_radius"], p["wheel_width"], p["wheelbase"], p["track"]
    robot = ET.Element("robot", name="jetacker")

    # -- base_footprint (massless ground projection) --
    ET.SubElement(robot, "link", name="base_footprint")

    # -- base_link: composite inertial from all rigid parts --
    base = ET.SubElement(robot, "link", name="base_link")
    parts = [(q["mass"], tuple(q["pos"]), part_inertia(q)) for q in p["base_parts"]]
    _inertial(base, *composite(parts))
    _mesh_visual(base, "base_link.stl", (0, 0, 0), mesh_prefix, mesh_scale)
    cb = p["collision_box"]
    c = ET.SubElement(base, "collision")
    _origin(c, cb["pos"])
    ET.SubElement(ET.SubElement(c, "geometry"), "box", size=_fmt(cb["size"]))
    _joint(robot, "base_joint", "fixed", "base_footprint", "base_link", (0, 0, wr))

    # -- wheels + steering knuckles --
    wheel_I = part_inertia({"type": "cylinder", "radius": wr, "length": ww,
                            "axis": "y", "mass": p["wheel_mass"]})
    knuckle_I = part_inertia({"type": "cylinder", "radius": 0.01, "length": 0.05,
                              "axis": "z", "mass": p["steer_knuckle_mass"]})
    for side, sy in (("left", 1), ("right", -1)):
        # steering knuckle (front)
        kpos = (wb, sy * tr / 2, 0)
        k = ET.SubElement(robot, "link", name=f"front_{side}_steering_link")
        _inertial(k, p["steer_knuckle_mass"], (0, 0, 0), knuckle_I)
        _mesh_visual(k, f"front_{side}_steering_link.stl", kpos, mesh_prefix, mesh_scale)
        _joint(robot, f"front_{side}_steering_joint", "revolute", "base_link",
               f"front_{side}_steering_link", kpos, axis=(0, 0, 1),
               limit={"lower": -p["steer_limit"], "upper": p["steer_limit"],
                      "effort": p["servo_max_torque"], "velocity": p["servo_max_speed"]})
        # front wheel on knuckle
        w = ET.SubElement(robot, "link", name=f"front_{side}_wheel_link")
        _inertial(w, p["wheel_mass"], (0, 0, 0), wheel_I)
        _mesh_visual(w, f"front_{side}_wheel_link.stl", kpos, mesh_prefix, mesh_scale)
        _cyl_collision(w, wr, ww)
        _joint(robot, f"front_{side}_wheel_joint", "continuous",
               f"front_{side}_steering_link", f"front_{side}_wheel_link",
               (0, 0, 0), axis=(0, 1, 0))
        # rear wheel on base
        rpos = (0, sy * tr / 2, 0)
        w = ET.SubElement(robot, "link", name=f"rear_{side}_wheel_link")
        _inertial(w, p["wheel_mass"], (0, 0, 0), wheel_I)
        _mesh_visual(w, f"rear_{side}_wheel_link.stl", rpos, mesh_prefix, mesh_scale)
        _cyl_collision(w, wr, ww)
        _joint(robot, f"rear_{side}_wheel_joint", "continuous", "base_link",
               f"rear_{side}_wheel_link", rpos, axis=(0, 1, 0),
               limit={"effort": p["motor_max_torque"], "velocity": p["motor_max_speed"]})

    # -- fixed sensor links --
    for name, s in p["sensor_links"].items():
        link = ET.SubElement(robot, "link", name=name)
        _inertial(link, s["mass"], (0, 0, 0), part_inertia({**s["shape"], "mass": s["mass"]}))
        _mesh_visual(link, f"{name}.stl", s["pos"], mesh_prefix, mesh_scale)
        _joint(robot, f"{name}_joint", "fixed", "base_link", name, s["pos"])

    # -- optical frames (ROS optical convention: Z fwd, X right, Y down) --
    for kind in ("color", "depth"):
        ET.SubElement(robot, "link", name=f"camera_{kind}_optical_frame")
        _joint(robot, f"camera_{kind}_optical_joint", "fixed", "camera_link",
               f"camera_{kind}_optical_frame", (0, 0, 0),
               rpy=(-math.pi / 2, 0, -math.pi / 2))

    return minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--params", default=os.path.join(here, "..", "params.json"))
    ap.add_argument("-o", "--out", default=os.path.join(here, "jetacker.urdf"))
    ap.add_argument("--mesh-prefix", default="../meshes/")
    ap.add_argument("--mesh-scale", type=float, default=0.001)
    args = ap.parse_args()
    urdf = build_urdf(json.load(open(args.params)), args.mesh_prefix, args.mesh_scale)
    with open(args.out, "w") as f:
        f.write(urdf)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
```

Note for the implementer: `imu_link` gets a mesh visual reference like every sensor link (`imu_link.stl` — the Fusion script exports a small plate for it). If the tests reveal any mismatch between this code and the test expectations, fix the code, not the tests.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd "$SIM" && python3 -m pytest tests/ -v`
Expected: all pass (6 inertia + 8 generator)

- [ ] **Step 5: Generate the URDF and (optionally) validate with check_urdf**

```bash
cd "$SIM/urdf" && python3 generate_urdf.py
# If ROS available (it won't be on this Mac — fine to skip):
check_urdf jetacker.urdf || echo "check_urdf not installed — skipped (runs on Isaac machine)"
```
Expected: `wrote .../jetacker.urdf`

- [ ] **Step 6: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/urdf EdgeAIJetson/Sim_Resources/tests/test_generate_urdf.py
git commit -m "feat(sim): URDF generator with analytic inertias and tests"
```

---

### Task 5: Fusion build script — geometry (MAIN SESSION — needs Fusion MCP)

**Files:**
- Create: `$SIM/fusion/build_jetacker.py`

**How to execute:** Read the file, pass its full contents as the `script` value to `mcp__fusion__fusion_mcp_execute` with `featureType: "script"`. The script creates a **new** Fusion document each run — close the previous run's unsaved document first (`fusion_mcp_execute` `document`/`close` with `userConfirmedCloseWithoutSave: true`) to avoid piling up untitled docs.

- [ ] **Step 1: Write the build script**

```python
# fusion/build_jetacker.py
"""Build the JetAcker sim model in Fusion 360 from params.json.

Run via the Fusion MCP execute tool. Rebuilds from scratch each run
(direct-modeling design; the 'parametric' source of truth is params.json).
All geometry is in car-assembly coordinates: base_link frame == Fusion origin
(rear axle center, axle height). Fusion API lengths are in cm: meters * 100.
Set EXPORT=True (Task 6) to write STLs to ../meshes/.
"""
import json
import os
import adsk.core
import adsk.fusion

SIM = ("/Users/nathanjones/Documents/Arduino/Mining Research Mother File/"
       "Mining-Research/EdgeAIJetson/Sim_Resources")
M = 100.0        # meters -> Fusion-internal cm
EXPORT = False   # flipped to True in Task 6

AXES = {"x": (1, 0, 0), "y": (0, 1, 0), "z": (0, 0, 1)}


def _pt(x, y, z):
    return adsk.core.Point3D.create(x * M, y * M, z * M)


def build(root, params):
    brep = adsk.fusion.TemporaryBRepManager.get()
    occs = {}

    def new_link(name):
        occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        occs[name] = occ
        return occ.component

    def add_box(comp, pos, size, name):
        obb = adsk.core.OrientedBoundingBox3D.create(
            _pt(*pos),
            adsk.core.Vector3D.create(1, 0, 0), adsk.core.Vector3D.create(0, 1, 0),
            size[0] * M, size[1] * M, size[2] * M)
        comp.bRepBodies.add(brep.createBox(obb)).name = name

    def add_cyl(comp, pos, axis, radius, length, name):
        ax = AXES[axis]
        p1 = _pt(*(pos[i] - ax[i] * length / 2 for i in range(3)))
        p2 = _pt(*(pos[i] + ax[i] * length / 2 for i in range(3)))
        body = brep.createCylinderOrCone(p1, radius * M, p2, radius * M)
        comp.bRepBodies.add(body).name = name

    def add_part(comp, part):
        if part["type"] == "box":
            add_box(comp, part["pos"], part["size"], part["name"])
        else:
            add_cyl(comp, part["pos"], part["axis"], part["radius"],
                    part["length"], part["name"])

    wr, ww = params["wheel_radius"], params["wheel_width"]
    wb, tr = params["wheelbase"], params["track"]

    # base_link: chassis + all rigidly-mounted parts
    base = new_link("base_link")
    for part in params["base_parts"]:
        add_part(base, part)

    # wheels + front steering knuckles
    for side, sy in (("left", 1), ("right", -1)):
        kpos = (wb, sy * tr / 2, 0)
        knuckle = new_link("front_%s_steering_link" % side)
        add_cyl(knuckle, (kpos[0], kpos[1] - sy * 0.01, kpos[2]), "z", 0.010, 0.050, "knuckle")
        wheel = new_link("front_%s_wheel_link" % side)
        add_cyl(wheel, kpos, "y", wr, ww, "tire")
        add_cyl(wheel, kpos, "y", wr * 0.55, ww * 1.1, "hub")
        wheel = new_link("rear_%s_wheel_link" % side)
        add_cyl(wheel, (0, sy * tr / 2, 0), "y", wr, ww, "tire")
        add_cyl(wheel, (0, sy * tr / 2, 0), "y", wr * 0.55, ww * 1.1, "hub")

    # fixed sensor links
    for name, s in params["sensor_links"].items():
        comp = new_link(name)
        add_part(comp, {**s["shape"], "pos": s["pos"], "name": name})

    return occs


def run(_context):
    app = adsk.core.Application.get()
    doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.DirectDesignType
    design.fusionUnitsManager.distanceDisplayUnits = \
        adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    root = design.rootComponent

    with open(os.path.join(SIM, "params.json")) as f:
        params = json.load(f)

    occs = build(root, params)

    # report: link list + overall envelope in mm (union of occurrence bboxes; cm -> mm)
    mins = [1e30] * 3
    maxs = [-1e30] * 3
    for occ in occs.values():
        bb = occ.boundingBox
        for i, (lo, hi) in enumerate([(bb.minPoint.x, bb.maxPoint.x),
                                      (bb.minPoint.y, bb.maxPoint.y),
                                      (bb.minPoint.z, bb.maxPoint.z)]):
            mins[i] = min(mins[i], lo)
            maxs[i] = max(maxs[i], hi)
    size_mm = [(maxs[i] - mins[i]) * 10 for i in range(3)]
    print("links: %s" % sorted(occs.keys()))
    print("envelope LxWxH mm: %.1f x %.1f x %.1f" % tuple(size_mm))
    print("ground clearance note: model z=0 is axle height; ground at z=%.1f mm"
          % (-params["wheel_radius"] * 1000))

    if EXPORT:
        export_stls(design, occs)


def export_stls(design, occs):
    mesh_dir = os.path.join(SIM, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    mgr = design.exportManager
    for name, occ in occs.items():
        path = os.path.join(mesh_dir, name + ".stl")
        opts = mgr.createSTLExportOptions(occ, path)
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        mgr.execute(opts)
        print("exported %s" % path)
```

- [ ] **Step 2: Execute in Fusion via MCP**

Run: `mcp__fusion__fusion_mcp_execute` with `featureType: "script"`, `object.script` = full file contents.
Expected output: `links: ['base_link', 'camera_link', ...]` (10 components) and `envelope LxWxH mm: ~314 x ~262 x ~220` (length ≈ 314 = rear tire to front tire; width 262 = track 222 + wheel width 40; height ≈ camera top 170 + wheel bottom 50).

If it errors: read the traceback, consult `fusion_mcp_read` `apiDocumentation` for the failing API, fix the script file, close the broken document, re-run. Do NOT wrap the body in try/except — the traceback is the debugging signal.

- [ ] **Step 3: Screenshot verification**

Run `mcp__fusion__fusion_mcp_read` `queryType: "screenshot"` with `direction` = `iso-top-right`, then `front`, then `left`.
Check against the user's photos: wheels at 4 corners, front wheels have knuckles, lidar puck on front deck, camera bar up on the mast, AGX Orin box at rear over axle, PCB stack hanging off the back. Report screenshots to the user for approval before proceeding.

- [ ] **Step 4: Envelope sanity check vs tape measurements**

Compare printed envelope with user's 317 × ~250 (over tires) × 229 mm. Height will read lower than 229 because the model's z=0 is axle height and bbox spans model geometry only — verify `envelope_height + wheel_radius*1000 ≈ 229 ± 15`. If off, adjust `params.json` positions (not the script) and re-run.

- [ ] **Step 5: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/fusion/build_jetacker.py
git commit -m "feat(sim): parametric Fusion build script for JetAcker model"
```

---

### Task 6: STL export + mesh-scale verification (MAIN SESSION — needs Fusion MCP)

**Files:**
- Modify: `$SIM/fusion/build_jetacker.py` (flip `EXPORT = False` → `True`)
- Create: `$SIM/tools/check_stl.py`
- Output: `$SIM/meshes/*.stl` (10 files)

- [ ] **Step 1: Write the STL bounding-box checker**

```python
#!/usr/bin/env python3
# tools/check_stl.py
"""Print the bounding-box extents of a binary STL. Usage: check_stl.py file.stl"""
import struct, sys


def bbox(path):
    with open(path, "rb") as f:
        f.read(80)
        (n,) = struct.unpack("<I", f.read(4))
        mins, maxs = [1e30] * 3, [-1e30] * 3
        for _ in range(n):
            rec = struct.unpack("<12fH", f.read(50))
            for v in range(3):
                for k in range(3):
                    val = rec[3 + v * 3 + k]
                    mins[k] = min(mins[k], val)
                    maxs[k] = max(maxs[k], val)
    return [maxs[k] - mins[k] for k in range(3)]


if __name__ == "__main__":
    print(["%.2f" % e for e in bbox(sys.argv[1])])
```

- [ ] **Step 2: Flip EXPORT to True and re-run the build script via MCP**

Edit `EXPORT = False` → `EXPORT = True`. Close the previous Fusion doc (unsaved), re-execute the script.
Expected: 10 `exported .../meshes/<link>.stl` lines.

- [ ] **Step 3: Verify mesh units and origins**

```bash
python3 "$SIM/tools/check_stl.py" "$SIM/meshes/rear_left_wheel_link.stl"
```
- Extents `['44.00', '101.00', '101.00']`-ish (x,y,z order may differ; wheel Ø101 across two axes, ~44 wide) → STL is in **mm**, keep `--mesh-scale 0.001`.
- If it prints `4.40 / 10.10` → STL is in **cm**, regenerate URDF with `--mesh-scale 0.01` and record that in `params.json` meta note.

Also check `base_link.stl` extents ≈ `360 x 178 x 170` (deck + overhangs, mm).

- [ ] **Step 4: Regenerate URDF (locks in verified scale) and re-run all tests**

```bash
cd "$SIM/urdf" && python3 generate_urdf.py && cd "$SIM" && python3 -m pytest tests/ -q
```
Expected: `wrote .../jetacker.urdf`, all tests pass.

- [ ] **Step 5: Save the Fusion document**

Via `mcp__fusion__fusion_mcp_execute` `featureType: "document"`, `operation: "save"` (name it `JetAcker_Sim` when prompted / via script `doc.saveAs`). Ask the user which cloud project to save into if unclear.

- [ ] **Step 6: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/fusion/build_jetacker.py \
        EdgeAIJetson/Sim_Resources/tools/check_stl.py \
        EdgeAIJetson/Sim_Resources/meshes \
        EdgeAIJetson/Sim_Resources/urdf/jetacker.urdf
git commit -m "feat(sim): export link meshes and generated URDF"
```

---

### Task 7: Isaac Sim URDF import script

**Files:**
- Create: `$SIM/isaac/import_jetacker.py`

Runs on the Isaac machine (Linux + RTX), not this Mac. Local verification = `py_compile` only.

- [ ] **Step 1: Write the import script**

```python
#!/usr/bin/env python3
# isaac/import_jetacker.py
"""Import jetacker.urdf into Isaac Sim and save jetacker.usd.

Run ON THE ISAAC MACHINE:
    ./python.sh Sim_Resources/isaac/import_jetacker.py
Written for Isaac Sim 4.5+/5.x (isaacsim.asset.importer.urdf).
"""
import os
from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import omni.kit.commands  # noqa: E402  (must import after SimulationApp)
import omni.usd  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "urdf", "jetacker.urdf")
USD_OUT = os.path.join(HERE, "jetacker.usd")

status, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints = False       # keep laser/camera_link/imu frames for TF
cfg.fix_base = False                 # mobile robot
cfg.convex_decomp = False            # collisions are primitives already
cfg.self_collision = False
cfg.distance_scale = 1.0             # URDF is in meters
cfg.density = 0.0                    # use URDF masses/inertias, never density

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile", urdf_path=URDF, import_config=cfg)
print("imported at prim:", prim_path)

stage = omni.usd.get_context().get_stage()
stage.GetRootLayer().Export(USD_OUT)
print("saved:", USD_OUT)
sim_app.close()
```

- [ ] **Step 2: Compile-check locally**

Run: `python3 -m py_compile "$SIM/isaac/import_jetacker.py" && echo OK`
Expected: `OK` (imports only resolve on the Isaac machine; py_compile checks syntax)

- [ ] **Step 3: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/isaac/import_jetacker.py
git commit -m "feat(sim): Isaac Sim URDF import script"
```

---

### Task 8: Ackermann math + Isaac Lab articulation config (TDD on the math)

**Files:**
- Create: `$SIM/isaac/ackermann.py` (pure Python — testable here)
- Create: `$SIM/isaac/jetacker_cfg.py` (Isaac Lab — py_compile only)
- Test: `$SIM/tests/test_ackermann.py`

- [ ] **Step 1: Write the failing tests**

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd "$SIM" && python3 -m pytest tests/test_ackermann.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'ackermann'`

- [ ] **Step 3: Implement ackermann.py**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd "$SIM" && python3 -m pytest tests/ -q`
Expected: all pass

- [ ] **Step 5: Write jetacker_cfg.py (Isaac Lab)**

```python
# isaac/jetacker_cfg.py
"""Isaac Lab ArticulationCfg for the JetAcker. Import from an Isaac Lab env/task.

Usage in a scene cfg:
    from jetacker_cfg import JETACKER_CFG
    robot: ArticulationCfg = JETACKER_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
Action mapping: use isaac/ackermann.py compute_ackermann() to turn (speed, steer)
policy actions into position targets for *_steering_joint and velocity targets
for rear_*_wheel_joint.
"""
import json
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

_HERE = os.path.dirname(os.path.abspath(__file__))
_P = json.load(open(os.path.join(_HERE, "..", "params.json")))
USD_PATH = os.path.join(_HERE, "jetacker.usd")

JETACKER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            max_linear_velocity=5.0, max_angular_velocity=20.0,
            disable_gravity=False),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=2),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, _P["wheel_radius"] + 0.005),
        joint_pos={".*": 0.0},
    ),
    actuators={
        # HTS-20H bus servo approximation: stiff position drive
        "steering": ImplicitActuatorCfg(
            joint_names_expr=["front_.*_steering_joint"],
            effort_limit=_P["servo_max_torque"],
            velocity_limit=_P["servo_max_speed"],
            stiffness=20.0, damping=1.0),
        # 520 encoder motors: velocity drive (stiffness must be 0)
        "drive": ImplicitActuatorCfg(
            joint_names_expr=["rear_.*_wheel_joint"],
            effort_limit=_P["motor_max_torque"],
            velocity_limit=_P["motor_max_speed"],
            stiffness=0.0, damping=0.6),
        # front wheels roll freely
        "front_rollers": ImplicitActuatorCfg(
            joint_names_expr=["front_.*_wheel_joint"],
            effort_limit=0.0, velocity_limit=_P["motor_max_speed"],
            stiffness=0.0, damping=0.001),
    },
)
```

- [ ] **Step 6: Compile-check + commit**

```bash
python3 -m py_compile "$SIM/isaac/jetacker_cfg.py" && echo OK
git add EdgeAIJetson/Sim_Resources/isaac/ackermann.py \
        EdgeAIJetson/Sim_Resources/isaac/jetacker_cfg.py \
        EdgeAIJetson/Sim_Resources/tests/test_ackermann.py
git commit -m "feat(sim): Ackermann math (tested) and Isaac Lab articulation config"
```

---

### Task 9: ROS2 bridge OmniGraph setup script

**Files:**
- Create: `$SIM/isaac/setup_ros2_graph.py`

Runs on the Isaac machine with the ROS2 bridge extension enabled. Node type names below are for Isaac Sim 4.5+/5.x — the script prints available node names on failure so version drift is easy to fix.

- [ ] **Step 1: Write the script**

```python
#!/usr/bin/env python3
# isaac/setup_ros2_graph.py
"""Open jetacker.usd, add ROS2 bridge graphs mirroring the real car's topics,
save as jetacker_ros2.usd.

Real-car contract (from EdgeAIJetson/JetAcker_SLAM_Nav2):
  /scan  LaserScan, frame 'laser'          /odom + TF odom->base_footprint
  /cmd_vel Twist (Nav2 output)             /camera/color|depth/image_raw + camera_info
  /joint_states, /tf, /clock

Run ON THE ISAAC MACHINE:
    ./python.sh Sim_Resources/isaac/setup_ros2_graph.py
"""
import json
import os
from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import omni.graph.core as og            # noqa: E402
import omni.kit.commands                # noqa: E402
import omni.usd                         # noqa: E402
from pxr import Gf, UsdGeom             # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
P = json.load(open(os.path.join(HERE, "..", "params.json")))
USD_IN = os.path.join(HERE, "jetacker.usd")
USD_OUT = os.path.join(HERE, "jetacker_ros2.usd")
ROBOT = "/jetacker"                       # articulation root prim from import

ctx = omni.usd.get_context()
ctx.open_stage(USD_IN)
stage = ctx.get_stage()

# ---- RTX lidar on the laser frame (RPLidar S2: 360 deg, 30 m, 10 Hz) ----
_, lidar = omni.kit.commands.execute(
    "IsaacSensorCreateRtxLidar",
    path="/lidar_sensor",
    parent=f"{ROBOT}/laser",
    config="RPLIDAR_S2E",   # ships with Isaac Sim; closest S2 profile
    translation=(0.0, 0.0, 0.02),
    orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0))

# ---- camera prim on the color optical frame (Astra Pro Plus-ish) ----
cam_path = f"{ROBOT}/camera_color_optical_frame/camera"
camera = UsdGeom.Camera.Define(stage, cam_path)
camera.GetFocalLengthAttr().Set(1.88)          # ~60 deg HFOV
camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.25, 8.0))  # Astra depth range

# ---- control + state graph ----
og.Controller.edit(
    {"graph_path": "/jetacker_ros2", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("ros_ctx", "isaacsim.ros2.bridge.ROS2Context"),
            ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("pub_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("sub_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ("ackermann", "isaacsim.robot.wheeled_robots.AckermannController"),
            ("art_steer", "isaacsim.core.nodes.IsaacArticulationController"),
            ("art_drive", "isaacsim.core.nodes.IsaacArticulationController"),
            ("pub_js", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("pub_tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("pub_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("odom_calc", "isaacsim.core.nodes.IsaacComputeOdometry"),
            # Twist fields are vector3 — break out scalars for the Ackermann node
            ("break_lin", "omni.graph.nodes.BreakVector3"),
            ("break_ang", "omni.graph.nodes.BreakVector3"),
            # steering = atan(wheelbase * yaw_rate / max(|vx|, eps)); Nav2 sends
            # Twist (vx, wz), the Ackermann node wants (speed, steeringAngle)
            ("twist2steer", "omni.graph.scriptnode.ScriptNode"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("sub_twist.inputs:topicName", "cmd_vel"),
            # ScriptNode ports (vx, wz -> speed, steeringAngle) are dynamic
            # attributes; create them in the GUI once or via
            # og.Controller.create_attribute before wiring. Script body:
            ("twist2steer.inputs:script", (
                "import math\n"
                "def compute(db):\n"
                "    vx, wz = db.inputs.vx, db.inputs.wz\n"
                "    db.outputs.speed = vx\n"
                "    db.outputs.steeringAngle = (\n"
                "        math.atan(%f * wz / vx) if abs(vx) > 1e-3 else 0.0)\n"
                "    return True\n" % P["wheelbase"])),
            ("ackermann.inputs:wheelBase", P["wheelbase"]),
            ("ackermann.inputs:trackWidth", P["track"]),
            ("ackermann.inputs:turningWheelRadius", P["wheel_radius"]),
            ("ackermann.inputs:useAcceleration", False),
            ("art_steer.inputs:robotPath", ROBOT),
            ("art_steer.inputs:jointNames",
             ["front_left_steering_joint", "front_right_steering_joint"]),
            ("art_drive.inputs:robotPath", ROBOT),
            ("art_drive.inputs:jointNames",
             ["rear_left_wheel_joint", "rear_right_wheel_joint"]),
            ("pub_js.inputs:targetPrim", ROBOT),
            ("pub_js.inputs:topicName", "joint_states"),
            ("pub_tf.inputs:targetPrims", [ROBOT]),
            ("odom_calc.inputs:chassisPrim", f"{ROBOT}/base_link"),
            ("pub_odom.inputs:topicName", "odom"),
            ("pub_odom.inputs:odomFrameId", "odom"),
            ("pub_odom.inputs:chassisFrameId", "base_footprint"),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "pub_clock.inputs:execIn"),
            ("tick.outputs:tick", "sub_twist.inputs:execIn"),
            ("tick.outputs:tick", "pub_js.inputs:execIn"),
            ("tick.outputs:tick", "pub_tf.inputs:execIn"),
            ("tick.outputs:tick", "odom_calc.inputs:execIn"),
            ("sim_time.outputs:simulationTime", "pub_clock.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "pub_js.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "pub_tf.inputs:timeStamp"),
            ("sim_time.outputs:simulationTime", "pub_odom.inputs:timeStamp"),
            ("ros_ctx.outputs:context", "pub_clock.inputs:context"),
            ("ros_ctx.outputs:context", "sub_twist.inputs:context"),
            ("ros_ctx.outputs:context", "pub_js.inputs:context"),
            ("ros_ctx.outputs:context", "pub_tf.inputs:context"),
            ("ros_ctx.outputs:context", "pub_odom.inputs:context"),
            ("sub_twist.outputs:execOut", "ackermann.inputs:execIn"),
            ("sub_twist.outputs:linearVelocity", "break_lin.inputs:tuple"),
            ("sub_twist.outputs:angularVelocity", "break_ang.inputs:tuple"),
            ("break_lin.outputs:x", "twist2steer.inputs:vx"),
            ("break_ang.outputs:z", "twist2steer.inputs:wz"),
            ("twist2steer.outputs:speed", "ackermann.inputs:speed"),
            ("twist2steer.outputs:steeringAngle", "ackermann.inputs:steeringAngle"),
            ("ackermann.outputs:execOut", "art_steer.inputs:execIn"),
            ("ackermann.outputs:execOut", "art_drive.inputs:execIn"),
            ("ackermann.outputs:leftWheelAngle", "art_steer.inputs:positionCommand"),
            ("ackermann.outputs:wheelRotationVelocity", "art_drive.inputs:velocityCommand"),
            ("odom_calc.outputs:execOut", "pub_odom.inputs:execIn"),
            ("odom_calc.outputs:position", "pub_odom.inputs:position"),
            ("odom_calc.outputs:orientation", "pub_odom.inputs:orientation"),
            ("odom_calc.outputs:linearVelocity", "pub_odom.inputs:linearVelocity"),
            ("odom_calc.outputs:angularVelocity", "pub_odom.inputs:angularVelocity"),
        ],
    },
)

# ---- sensor publishers via replicator writers ----
import isaacsim.ros2.bridge as ros2_bridge  # noqa: E402,F401
import omni.replicator.core as rep          # noqa: E402

# lidar -> /scan
hydra = rep.create.render_product(lidar.GetPath(), [1, 1])
writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
writer.initialize(topicName="scan", frameId="laser")
writer.attach([hydra])

# camera -> /camera/color + /camera/depth (640x480)
cam_rp = rep.create.render_product(cam_path, [640, 480])
for writer_name, topic in [
        ("ROS2PublishImage", "camera/color/image_raw"),
        ("ROS2PublishCameraInfo", "camera/color/camera_info"),
        ("ROS2PublishDepthImage", "camera/depth/image_raw")]:
    w = rep.writers.get(writer_name)
    w.initialize(topicName=topic, frameId="camera_color_optical_frame")
    w.attach([cam_rp])

ctx.save_as_stage(USD_OUT)
print("saved:", USD_OUT)
sim_app.close()
```

**Known version-drift risks (leave this note in the file's docstring if edited):** node type names (`isaacsim.ros2.bridge.*` vs older `omni.isaac.ros2_bridge.*`), the `RPLIDAR_S2E` config name, Ackermann controller attribute names, and replicator writer names vary across Isaac releases. The `twist2steer` ScriptNode needs its `vx/wz/speed/steeringAngle` ports created as dynamic attributes (`og.Controller.create_attribute`, or once in the GUI) before the CONNECT entries touching it will resolve. On the Isaac machine, if a CREATE_NODES entry fails, run `og.get_registered_nodes()` (or check the OmniGraph node library in the GUI) and substitute. The graph *shape* is correct.

- [ ] **Step 2: Compile-check locally**

Run: `python3 -m py_compile "$SIM/isaac/setup_ros2_graph.py" && echo OK`
Expected: `OK`

- [ ] **Step 3: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/isaac/setup_ros2_graph.py
git commit -m "feat(sim): ROS2 bridge OmniGraph setup mirroring real-car topics"
```

---

### Task 10: README + final review

**Files:**
- Create: `$SIM/README.md`

- [ ] **Step 1: Write README**

```markdown
# JetAcker Sim Resources

Fusion-360-generated sim model of the custom JetAcker (AGX Orin + RPLidar S2 +
Astra Pro Plus) for Isaac Sim / Isaac Lab. Spec: `docs/2026-07-15-jetacker-sim-model-design.md`.

## Single source of truth
`params.json` — every dimension/mass. Measured a better value on the real car?
Edit it here, then rebuild (steps below). Never edit `urdf/jetacker.urdf` by hand.

## Rebuild pipeline (on the Mac, Fusion 360 + Fusion MCP running)
1. Run `fusion/build_jetacker.py` via the Fusion MCP execute tool
   (`EXPORT = True` writes `meshes/*.stl`).
2. `cd urdf && python3 generate_urdf.py`   → `urdf/jetacker.urdf`
3. `python3 -m pytest tests/`              → all green

## On the Isaac machine (Linux + RTX, Isaac Sim 4.5+)
4. `./python.sh isaac/import_jetacker.py`  → `isaac/jetacker.usd`
5. `./python.sh isaac/setup_ros2_graph.py` → `isaac/jetacker_ros2.usd`
   (ROS2 topics: /scan /odom /cmd_vel /joint_states /tf /clock /camera/...)
6. Isaac Lab: `from jetacker_cfg import JETACKER_CFG`; use
   `isaac/ackermann.py::compute_ackermann` to map (speed, steer) actions to joints.

## RViz (any ROS2 Humble machine)
    ros2 run robot_state_publisher robot_state_publisher urdf/jetacker.urdf
With sim running, /joint_states + /tf complete the tree.

## Verification checklist (from spec §7)
- [ ] `check_urdf urdf/jetacker.urdf` clean
- [ ] Isaac import: rests on 4 wheels, no articulation warnings
- [ ] Full-steer turn radius ≈ wheelbase/tan(30°) ≈ 0.37 m
- [ ] `ros2 topic hz /scan` ≈ 10 Hz; RViz TF tree all green
- [ ] Weigh the real car; update masses in params.json (current total ≈ 5.4 kg estimated)
```

- [ ] **Step 2: Run the full test suite one last time**

Run: `cd "$SIM" && python3 -m pytest tests/ -v`
Expected: all pass

- [ ] **Step 3: Commit**

```bash
git add EdgeAIJetson/Sim_Resources/README.md
git commit -m "docs(sim): README with rebuild pipeline and verification checklist"
```

- [ ] **Step 4: Report** — summarize to the user: what was built, screenshots of the Fusion model, the verification items that must run on the Isaac machine, and the measured-value refinements still open (car weight, steering angle limit, component positions).

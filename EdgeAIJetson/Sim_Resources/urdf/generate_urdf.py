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

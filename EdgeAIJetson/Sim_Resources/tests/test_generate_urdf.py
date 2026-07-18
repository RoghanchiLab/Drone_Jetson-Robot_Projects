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

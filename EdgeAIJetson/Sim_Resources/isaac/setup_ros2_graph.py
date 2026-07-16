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

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

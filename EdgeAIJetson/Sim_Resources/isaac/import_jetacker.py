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

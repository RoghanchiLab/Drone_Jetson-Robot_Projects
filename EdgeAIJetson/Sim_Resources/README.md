# JetAcker Sim Resources

Fusion-360-generated sim model of the custom JetAcker (AGX Orin + RPLidar S2 +
Astra Pro Plus) for Isaac Sim / Isaac Lab. Spec: `docs/2026-07-15-jetacker-sim-model-design.md`
(kept local, not committed). Fusion cloud doc: `JetAcker_Sim` in **Default Project**.

Verified model envelope: **318.5 × 266.0 × 220.5 mm** (vs measured car: 12.5 in long).
All parts connectivity-checked — no floating bodies.

## Single source of truth
`params.json` — every dimension/mass. Measured a better value on the real car?
Edit it here, then rebuild (steps below). Never edit `urdf/jetacker.urdf` by hand.

## Rebuild pipeline (on the Mac, Fusion 360 + Fusion MCP running)
1. Run `fusion/build_jetacker.py` via the Fusion MCP execute tool
   (`EXPORT = True` writes `meshes/*.stl`; STLs are in **mm**).
2. `cd urdf && python3 generate_urdf.py`   → `urdf/jetacker.urdf`
3. `python3 -m pytest tests/`              → all green (18 tests)
4. Sanity: `python3 tools/check_stl.py meshes/rear_left_wheel_link.stl`
   → expect ~101 on two axes (wheel Ø in mm; confirms `--mesh-scale 0.001`)

## On the Isaac machine (Linux + RTX, Isaac Sim 4.5+)
5. `./python.sh isaac/import_jetacker.py`  → `isaac/jetacker.usd`
6. `./python.sh isaac/setup_ros2_graph.py` → `isaac/jetacker_ros2.usd`
   (ROS2 topics mirroring the real car: /scan /odom /cmd_vel /joint_states /tf /clock /camera/...)
   Node type names drift between Isaac releases — see the script's docstring if a
   node fails to create.
7. Isaac Lab: `from jetacker_cfg import JETACKER_CFG`; use
   `isaac/ackermann.py::compute_ackermann` to map (speed, steer) actions to
   steering-position + rear-wheel-velocity joint targets.

## RViz (any ROS2 Humble machine)
RViz can't resolve the URDF's relative mesh paths — generate an RViz variant
with absolute `file://` mesh URIs (or drop the meshes into a ROS package and use
`package://`):

    cd urdf
    python3 generate_urdf.py -o jetacker_rviz.urdf --mesh-prefix "file://$(cd ../meshes && pwd)/"
    ros2 run robot_state_publisher robot_state_publisher jetacker_rviz.urdf

With the sim running, `/joint_states` + `/tf` complete the tree. The default
`jetacker.urdf` (relative `../meshes/` prefix) is what the Isaac importer wants.

## Verification checklist (from spec §7)
- [x] Fusion envelope matches tape measurements (318.5 mm vs 317 mm length)
- [x] STL units verified (mm) via tools/check_stl.py
- [x] URDF parses, 13 links / 12 joints, all pytest green
- [x] No floating bodies (bbox-adjacency flood fill from deck)
- [ ] `check_urdf urdf/jetacker.urdf` clean (run on ROS machine)
- [ ] Isaac import: rests on 4 wheels, no articulation warnings
- [ ] Full-steer turn radius ≈ wheelbase/tan(30°) ≈ 0.37 m
- [ ] `ros2 topic hz /scan` ≈ 10 Hz; RViz TF tree all green
- [ ] Weigh the real car; update masses in params.json (current total ≈ 5.13 kg estimated)
- [ ] Measure real steering angle limit (currently ±30° assumed)

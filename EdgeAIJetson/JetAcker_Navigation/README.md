# JetAcker Navigation

Autonomous navigation system for the JetAcker 2WD Ackermann robot.  
Integrates **Nav2 + SLAM Toolbox + Nvblox** with the **AckermannController** hardware driver directly — replacing the `motor_node`/`servo_node` middleware with a single Python bridge node.

## Architecture

```
RPLidar S2  ──► SLAM Toolbox ──► 2D map + localization
Orbbec RGB-D ──► Nvblox       ──► 3D costmap layer
                                        │
                                   Nav2 (RegulatedPurePursuit)
                                        │  /cmd_vel (Twist)
                                        ▼
                              ackermann_nav_node.py
                                        │
                         ┌──────────────┴──────────────┐
                    linear.x → wheel RPS         angular.z → servo pos
                         │                               │
                  MotorController                   Board SDK
                  (/dev/ttyACM0)                   (/dev/rrc)
```

`ackermann_nav_node.py` owns the hardware exclusively. Do **not** run `motor_node` or `servo_node` alongside it.

## Prerequisites

| Dependency | Version |
|---|---|
| ROS 2 | Humble |
| Nav2 | `nav2_bringup`, full stack |
| SLAM Toolbox | `slam_toolbox` |
| Nvblox | `nvblox_ros` |
| RPLidar | `sllidar_ros2` |
| Orbbec camera | `orbbec_camera` (via `jetacker_bringup`) |
| JetAcker bringup | `jetacker_bringup` (URDF, BT XML, RViz config) |
| HiWonder SDK | `motor_controller`, `ros_robot_controller_sdk` |

The HiWonder SDK must be on the system at:
```
/mnt/nova_ssd/workspaces/isaac_ros-dev/src/HiWonder_Software/
  cadeJetson/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller/
```

## Hardware connections

| Device | Default port | Description |
|---|---|---|
| STM32 motor controller | `/dev/ttyACM0` | Rear drive motors |
| Servo board | `/dev/rrc` | Front steering servo |
| RPLidar S2 | `/dev/rplidar` | 2D LiDAR (symlink recommended) |

Create stable udev symlinks (recommended):
```bash
# /etc/udev/rules.d/99-jetacker.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="ttyACM0"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",  SYMLINK+="rplidar"
```

## Running

### 1. Mapping mode (build a new map)

Launch the full stack — robot will map the environment as it is navigated:

```bash
ros2 launch jetacker_navigation navigation.launch.py
```

With RViz visualization:

```bash
ros2 launch jetacker_navigation navigation.launch.py use_rviz:=true
```

### 2. Navigation mode (use an existing map)

Once a map has been saved with SLAM Toolbox, switch to localization-only mode:

```bash
ros2 launch jetacker_navigation navigation.launch.py localization_mode:=true
```

### 3. Custom ports

```bash
ros2 launch jetacker_navigation navigation.launch.py \
    motor_port:=/dev/ttyACM0 \
    steer_device:=/dev/rrc \
    lidar_port:=/dev/rplidar
```

### 4. Sending navigation goals

After launch, send a goal pose from the terminal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{ pose: { header: { frame_id: 'map' },
             pose: { position: { x: 1.0, y: 0.5, z: 0.0 },
                     orientation: { w: 1.0 } } } }"
```

Or use RViz2 **2D Nav Goal** tool (launch with `use_rviz:=true`).

### 5. Saving the map

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{ name: { data: '/path/to/maps/my_map' } }"
```

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `use_rviz` | `false` | Launch RViz2 |
| `localization_mode` | `false` | Localization only (requires saved map) |
| `motor_port` | `/dev/ttyACM0` | Motor controller serial port |
| `steer_device` | `/dev/rrc` | Steering servo board device |
| `lidar_port` | `/dev/rplidar` | RPLidar S2 serial port |
| `nav2_params_file` | `config/nav2_params.yaml` | Nav2 parameters file |
| `left_rps_scale` | `1.0` | Left motor RPS per (m/s) of `cmd_vel` |
| `right_rps_scale` | `-2.0` | Right motor RPS per (m/s) — negative for inverted mount |
| `wheelbase` | `0.14` | Front-to-rear axle distance [m] |
| `cmd_vel_timeout` | `0.5` | Auto-stop delay when `cmd_vel` goes silent [s] |

## Calibration

The default `left_rps_scale` / `right_rps_scale` values are derived from the
demo in `Movement/ackermann_controller.py` (`LEFT_DEFAULT_RPS=0.5`,
`RIGHT_DEFAULT_RPS=-1.0` → ratio `−2.0`).

To calibrate for a specific linear speed:

1. Command a known velocity and measure actual speed over a fixed distance.
2. Scale `left_rps_scale` proportionally:
   ```
   left_rps_scale = measured_rps / commanded_linear_vel_ms
   ```
3. Keep `right_rps_scale = left_rps_scale × (−2.0)` unless the robot drifts.
4. Adjust `wheelbase` until turns match expected arc radius.

## Velocity → hardware conversion

```
# Steering (Ackermann geometry)
steer_angle = atan2(angular_z × wheelbase, |linear_x|)   # radians
servo_pos   = 500 − clamp(steer_angle / 0.6458, −1, 1) × 200
              # 500 = center, 300 = full-left (~37°), 700 = full-right

# Drive
left_rps  = linear_x × left_rps_scale
right_rps = linear_x × right_rps_scale
```

## File overview

```
JetAcker_Navigation/
├── ackermann_nav_node.py        ROS2 node: /cmd_vel → AckermannController
├── config/
│   ├── nav2_params.yaml         Nav2 stack parameters (RPP controller, costmaps)
│   ├── slam_toolbox_params.yaml SLAM Toolbox mapping/localization parameters
│   └── nvblox_params.yaml       Nvblox 3D voxel mapping parameters
└── launch/
    ├── navigation.launch.py     Master launch (RSP + nav node + sensors + Nav2)
    └── rplidar.launch.py        RPLidar S2 driver launch
```

## Relation to JetAcker_SLAM_Nav2

`JetAcker_Navigation` replaces the `jetacker_bringup` motor/servo ROS middleware
with a direct hardware connection through `AckermannController`. Everything else
(Nav2 config, SLAM config, sensor launch) is equivalent.

| | JetAcker_SLAM_Nav2 | JetAcker_Navigation |
|---|---|---|
| Hardware driver | `motor_node` + `servo_node` | `ackermann_nav_node.py` |
| Hardware interface | ROS driver package | `AckermannController` (direct SDK) |
| Bringup | `robot.launch.py` (full) | RSP only |
| Nav2 / SLAM / sensors | Same | Same |

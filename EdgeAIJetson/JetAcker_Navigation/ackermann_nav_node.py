#!/usr/bin/env python3
"""
JetAcker Autonomous Navigation Node

Bridges Nav2 cmd_vel (geometry_msgs/Twist) to the AckermannController
hardware interface (MotorController + steering servo).

Architecture:
  Nav2 RegulatedPurePursuitController
       │
       │  /cmd_vel  (geometry_msgs/Twist)
       ▼
  AckermannNavNode  ←── this node
       │
       ├── linear.x  (m/s) → wheel RPS via speed_gain
       └── angular.z (rad/s) → steering servo via Ackermann geometry

Velocity → hardware mapping:
  steering_angle = atan2(angular_z * wheelbase, |linear_x|)   [rad]
  servo_pos = 500 - clamp(steering_angle / max_steer_angle, -1, 1) * 200
              (500=center, 300=full-left, 700=full-right)

  left_rps  = linear_x * left_rps_scale
  right_rps = linear_x * right_rps_scale
  (scales incorporate wheel circumference, gear ratio, and motor orientation)

ROS2 parameters (all tunable at launch):
  motor_port        (string)  '/dev/ttyACM0'  STM32 motor controller port
  steer_device      (string)  '/dev/rrc'      steering servo board
  wheelbase         (float)   0.14            robot wheelbase [m]
  max_steer_angle   (float)   0.6458          max steering angle [rad] (~37°)
  left_rps_scale    (float)   1.0             left  motor: RPS per m/s cmd
  right_rps_scale   (float)  -2.0             right motor: RPS per m/s cmd
                                              (negative = opposite mount orientation,
                                               ratio from demo: 0.5 left / -1.0 right)
  steer_duration    (float)   0.1             servo move duration [s] per cmd
  cmd_vel_timeout   (float)   0.5             stop if no cmd received [s]
  min_linear_speed  (float)   0.05            below this, steer proportionally [m/s]

Calibration guide:
  1. Set left_rps_scale / right_rps_scale so the robot drives straight.
     Demo forward speed: left=0.5 RPS, right=-1.0 RPS → ratio = -2.0.
     Adjust left_rps_scale until Nav2's desired_linear_vel matches actual speed.
  2. Set wheelbase to actual front-axle to rear-axle distance (meters).
  3. Verify max_steer_angle matches physical servo limits.
"""

import math
import os
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Locate Movement/ relative to this file (EdgeAIJetson/JetAcker_Navigation → Movement)
_this_dir = os.path.dirname(os.path.abspath(__file__))
_movement_dir = os.path.join(_this_dir, '..', 'Movement')
sys.path.insert(0, os.path.abspath(_movement_dir))

# SDK path (same as ackermann_controller.py)
sys.path.insert(0, '/mnt/nova_ssd/workspaces/isaac_ros-dev/src/HiWonder_Software/'
                   'cadeJetson/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller')

from ackermann_controller import AckermannController  # noqa: E402

# Servo position constants (mirror ackermann_controller.py)
SERVO_CENTER = 500
SERVO_MAX_OFFSET = 200  # 500 ± 200 → 300 (full-left) .. 700 (full-right)


class AckermannNavNode(Node):
    """
    ROS2 node that translates Nav2 Twist commands to JetAcker hardware.

    Subscribes:  /cmd_vel  (geometry_msgs/Twist)
    Hardware:    AckermannController (MotorController + Board SDK)
    """

    def __init__(self):
        super().__init__('ackermann_nav_node')

        # ── Declare & read parameters ──────────────────────────────────────────
        self.declare_parameter('motor_port',       '/dev/ttyACM0')
        self.declare_parameter('steer_device',     '/dev/rrc')
        self.declare_parameter('wheelbase',         0.14)
        self.declare_parameter('max_steer_angle',   0.6458)   # rad (~37°)
        self.declare_parameter('left_rps_scale',    1.0)
        self.declare_parameter('right_rps_scale',  -2.0)
        self.declare_parameter('steer_duration',    0.1)
        self.declare_parameter('cmd_vel_timeout',   0.5)
        self.declare_parameter('min_linear_speed',  0.05)

        motor_port       = self.get_parameter('motor_port').value
        steer_device     = self.get_parameter('steer_device').value
        self.wheelbase        = self.get_parameter('wheelbase').value
        self.max_steer_angle  = self.get_parameter('max_steer_angle').value
        self.left_rps_scale   = self.get_parameter('left_rps_scale').value
        self.right_rps_scale  = self.get_parameter('right_rps_scale').value
        self.steer_duration   = self.get_parameter('steer_duration').value
        self.cmd_vel_timeout  = self.get_parameter('cmd_vel_timeout').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value

        # ── Connect to hardware ────────────────────────────────────────────────
        self.robot = AckermannController(motor_port=motor_port,
                                         steer_device=steer_device)
        self.robot.connect()
        self.robot.warm_up()
        self.robot.steer_center()
        self.get_logger().info(
            f'Connected — motor: {motor_port}, steer: {steer_device}'
        )

        # ── State ──────────────────────────────────────────────────────────────
        self._stopped = True
        self._lock = threading.Lock()
        self._last_cmd_time = self.get_clock().now()

        # ── Subscriber & watchdog ──────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_timer(0.1, self._watchdog_cb)

        self.get_logger().info('AckermannNavNode ready — listening on /cmd_vel')

    # ── cmd_vel handler ────────────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist) -> None:
        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        with self._lock:
            self._last_cmd_time = self.get_clock().now()

        servo_pos = self._compute_servo(linear_x, angular_z)
        left_rps, right_rps = self._compute_wheel_rps(linear_x)

        # Send steering first so wheels turn to correct angle before driving
        self.robot.steer(servo_pos, duration=self.steer_duration)

        if abs(linear_x) < 1e-3:
            if not self._stopped:
                self.robot.stop()
                self._stopped = True
        else:
            self.robot.drive(left_rps, right_rps)
            self._stopped = False

    # ── Velocity → hardware conversions ───────────────────────────────────────

    def _compute_servo(self, linear_x: float, angular_z: float) -> int:
        """
        Convert cmd_vel velocities to steering servo position.

        Uses Ackermann geometry when moving, proportional fallback when slow.
        Positive angular_z (CCW / left turn) → servo < 500.
        """
        if abs(linear_x) >= self.min_linear_speed:
            # Ackermann: steer_angle = atan(angular_z * wheelbase / |linear_x|)
            steer_rad = math.atan2(angular_z * self.wheelbase, abs(linear_x))
        else:
            # Nearly stopped: scale angular_z down to a small steering offset
            # so the robot can pre-steer without full Ackermann math
            max_slow_steer = self.max_steer_angle * 0.5
            steer_rad = angular_z * (max_slow_steer / max(self.min_linear_speed, 0.01))
            steer_rad = max(-max_slow_steer, min(max_slow_steer, steer_rad))

        steer_rad = max(-self.max_steer_angle, min(self.max_steer_angle, steer_rad))
        normalized = steer_rad / self.max_steer_angle           # –1.0 … +1.0
        servo_pos = int(SERVO_CENTER - normalized * SERVO_MAX_OFFSET)
        return max(0, min(1000, servo_pos))

    def _compute_wheel_rps(self, linear_x: float):
        """
        Convert linear velocity (m/s) to left/right motor RPS.

        left_rps_scale / right_rps_scale encode wheel circumference, gear ratio,
        and motor mounting orientation from the ackermann_controller.py demo
        (left=0.5 RPS, right=−1.0 RPS for forward → ratio −2.0).
        """
        left_rps  = linear_x * self.left_rps_scale
        right_rps = linear_x * self.right_rps_scale
        return left_rps, right_rps

    # ── Watchdog ───────────────────────────────────────────────────────────────

    def _watchdog_cb(self) -> None:
        """Stop the robot if no cmd_vel has been received within the timeout."""
        with self._lock:
            elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9

        if elapsed > self.cmd_vel_timeout and not self._stopped:
            self.get_logger().warn(
                f'No cmd_vel for {elapsed:.2f}s — stopping robot'
            )
            self.robot.stop()
            self._stopped = True

    # ── Cleanup ────────────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self.get_logger().info('Shutting down — stopping motors and centering steering')
        self.robot.stop()
        self.robot.steer_center()
        import time
        time.sleep(0.3)
        self.robot.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AckermannNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

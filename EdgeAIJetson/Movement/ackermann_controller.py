#!/usr/bin/env python3
"""
JetAcker 2WD Ackermann Steering Robot Controller

Combines rear wheel drive (MotorController via /dev/ttyACM0) and
front steering servo (Board SDK via /dev/rrc) into a single interface.

Usage:
  python3 ackermann_controller.py                        # Run demo sequence
  python3 ackermann_controller.py --steer-position 300  # Move steering to specific position and exit
"""

import argparse
import signal
import sys
import os
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, '/mnt/nova_ssd/workspaces/isaac_ros-dev/src/HiWonder_Software/'
                   'cadeJetson/ros2_ws/src/driver/ros_robot_controller/ros_robot_controller')

from motor_controller import MotorController
from ros_robot_controller_sdk import Board

# Drive defaults
LEFT_DEFAULT_RPS = 0.5
RIGHT_DEFAULT_RPS = -1.0

# Steering servo constants
SERVO_ID = 9
STEER_CENTER = 500
STEER_LEFT = 300    # ~37 deg left
STEER_RIGHT = 700   # ~37 deg right


class AckermannController:
    """Controls both rear drive motors and front steering servo for JetAcker 2WD."""

    def __init__(self, motor_port='/dev/ttyACM0', steer_device='/dev/rrc'):
        self.motor_port = motor_port
        self.steer_device = steer_device
        self.mc = None
        self.board = None

    def connect(self):
        print(f'Connecting to motor controller on {self.motor_port}...')
        self.mc = MotorController(port=self.motor_port)
        self.mc.connect()

        print(f'Connecting to steering board on {self.steer_device}...')
        self.board = Board(device=self.steer_device)
        time.sleep(0.5)
        print('Both systems connected.')

    def disconnect(self):
        if self.mc:
            self.mc.stop()
            self.mc.disconnect()
        if self.board:
            self.steer_center(duration=0.5)
            time.sleep(0.3)
        print('Disconnected.')

    def warm_up(self):
        """Warm up drive motors to eliminate cold-start delay."""
        self.mc.warm_up()

    def drive(self, left_rps, right_rps):
        """Set rear wheel speeds in RPS (positive = forward)."""
        self.mc.set_wheel_speeds(right_rps=right_rps, left_rps=left_rps)

    def stop(self):
        """Stop drive motors."""
        self.mc.stop()

    def steer(self, position, duration=0.5):
        """
        Move steering servo to position.
          position: 0-1000 (500=center, 300=left ~37deg, 700=right ~37deg)
          duration: seconds for servo to reach position
        """
        position = max(0, min(1000, position))
        self.board.bus_servo_set_position(duration, [[SERVO_ID, position]])

    def steer_left(self, duration=0.5):
        self.steer(STEER_LEFT, duration)

    def steer_right(self, duration=0.5):
        self.steer(STEER_RIGHT, duration)

    def steer_center(self, duration=0.5):
        self.steer(STEER_CENTER, duration)

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


def run_demo(robot):
    """Demo: warm up, then demonstrate 6 basic movement modes."""
    print('\nWarming up motors...')
    robot.warm_up()
    time.sleep(1)

    print('\nCentering steering...')
    robot.steer_center()
    time.sleep(0.8)

    # 1. Forward (straight)
    print('\n1. Forward (straight)...')
    robot.steer_center()
    time.sleep(0.5)
    robot.drive(LEFT_DEFAULT_RPS, RIGHT_DEFAULT_RPS)
    time.sleep(2)
    robot.stop()
    time.sleep(1)

    # 2. Backward (straight)
    print('\n2. Backward (straight)...')
    robot.steer_center()
    time.sleep(0.5)
    robot.drive(-LEFT_DEFAULT_RPS, -RIGHT_DEFAULT_RPS)
    time.sleep(2)
    robot.stop()
    time.sleep(1)

    # 3. Turn forward left
    print('\n3. Turn forward left...')
    robot.steer_left()
    time.sleep(0.5)
    robot.drive(LEFT_DEFAULT_RPS, RIGHT_DEFAULT_RPS)
    time.sleep(2)
    robot.stop()
    robot.steer_center()
    time.sleep(1)

    # 4. Turn forward right
    print('\n4. Turn forward right...')
    robot.steer_right()
    time.sleep(0.5)
    robot.drive(LEFT_DEFAULT_RPS, RIGHT_DEFAULT_RPS)
    time.sleep(2)
    robot.stop()
    robot.steer_center()
    time.sleep(1)

    # 5. Backward left
    print('\n5. Backward left...')
    robot.steer_left()
    time.sleep(0.5)
    robot.drive(-LEFT_DEFAULT_RPS, -RIGHT_DEFAULT_RPS)
    time.sleep(2)
    robot.stop()
    robot.steer_center()
    time.sleep(1)

    # 6. Backward right
    print('\n6. Backward right...')
    robot.steer_right()
    time.sleep(0.5)
    robot.drive(-LEFT_DEFAULT_RPS, -RIGHT_DEFAULT_RPS)
    time.sleep(2)
    robot.stop()
    robot.steer_center()
    time.sleep(1)

    print('\nDemo complete.')


def main():
    parser = argparse.ArgumentParser(description='JetAcker 2WD Ackermann robot controller')
    parser.add_argument('--motor-port', default='/dev/ttyACM0',
                        help='Motor controller serial port (default: /dev/ttyACM0)')
    parser.add_argument('--steer-device', default='/dev/rrc',
                        help='Steering board serial device (default: /dev/rrc)')
    parser.add_argument('--steer-position', type=int,
                        help='Move steering to specific position (0-1000) and exit')
    parser.add_argument('--steer-duration', type=float, default=0.5,
                        help='Steering move duration in seconds (default: 0.5)')
    args = parser.parse_args()

    if args.steer_position is not None and not 0 <= args.steer_position <= 1000:
        print(f'Error: --steer-position must be 0-1000, got {args.steer_position}')
        sys.exit(1)

    with AckermannController(motor_port=args.motor_port, steer_device=args.steer_device) as robot:

        def handle_signal(sig, frame):
            print('\nInterrupted! Stopping and centering steering...')
            robot.stop()
            robot.steer_center()
            time.sleep(0.3)
            sys.exit(0)

        signal.signal(signal.SIGINT, handle_signal)

        try:
            if args.steer_position is not None:
                robot.steer(args.steer_position, args.steer_duration)
                time.sleep(args.steer_duration + 0.3)
            else:
                run_demo(robot)
        except KeyboardInterrupt:
            print('\nEmergency stop.')
            robot.stop()
            robot.steer_center()


if __name__ == '__main__':
    main()

"""ROS 2 joint-command publisher for the H1 Isaac Sim graph."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


JOINTS = [
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_elbow_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_elbow_joint',
]


class DrywallController(Node):
    def __init__(self) -> None:
        super().__init__('drywall_ros_controller')
        self.publisher = self.create_publisher(JointState, '/joint_command', 10)
        self.status_publisher = self.create_publisher(String, '/drywall_install/status', 10)
        self.timer = self.create_timer(0.1, self.publish_command)
        self.start_time = self.get_clock().now()
        self.get_logger().info('Publishing H1 joint targets on /joint_command')

    def publish_command(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        wave = 0.12 * math.sin(elapsed * 1.5)
        msg.position = [-0.35 + wave, 0.25, -0.85, -0.35 - wave, -0.25, -0.85]
        self.publisher.publish(msg)

        status = String()
        status.data = 'ros2_controller_active'
        self.status_publisher.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DrywallController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

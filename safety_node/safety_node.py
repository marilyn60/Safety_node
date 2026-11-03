#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.speed = 0.0
        self.threshold = 0.5

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.odom_callback, 10)

        self.publisher = self.create_publisher(
            AckermannDriveStamped, '/drive', 10)

    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        min_ittc = float('inf')

        for i in range(len(scan_msg.ranges)):
            r = scan_msg.ranges[i]

            if r != r:
                continue
            if r == float('inf'):
                continue
            if r < scan_msg.range_min or r > scan_msg.range_max:
                continue

            theta = scan_msg.angle_min + i * scan_msg.angle_increment
            r_dot = -self.speed * math.cos(theta)

            if r_dot >= 0:
                continue

            ittc = r / (-r_dot)

            if ittc < min_ittc:
                min_ittc = ittc

        if min_ittc < self.threshold:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0
            self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

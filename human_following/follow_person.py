#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

class FollowPersonNode(Node):
    def __init__(self):
        super().__init__('follow_person_node')
        self.subscription = self.create_subscription(Point, '/human_position', self.position_callback, 10)
        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_bool = self.create_publisher(Bool, '/too_close', 10)

        self.target_x = 320
        self.kp_angular = 0.005
        self.kp_linear = 0.5
        self.min_distance = 0.6
        self.stop_distance = 1.0

    def position_callback(self, msg):
        cx, cy, distance = msg.x, msg.y, msg.z
        cmd = Twist()
        too_close_msg = Bool()

        if distance > self.stop_distance:
            error_x = cx - self.target_x
            cmd.angular.z = -self.kp_angular * error_x
            cmd.linear.x = min(self.kp_linear * (distance - self.min_distance), 0.5)
            too_close_msg.data = False
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            too_close_msg.data = True

        self.publisher_cmd.publish(cmd)
        self.publisher_bool.publish(too_close_msg)


def main():
    rclpy.init()
    node = FollowPersonNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

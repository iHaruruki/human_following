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
        self.kp_angular = 0.02  # 追従の応答速度を向上
        self.kp_linear = 0.6  # 追従の速度を向上
        self.min_distance = 0.6
        self.stop_distance = 1.0

        self.tracking_id = None  # 最初に検出した人のID
        self.last_seen_time = self.get_clock().now()
        self.lost_threshold = 2.0  # 人を見失ったと判断する時間（秒）

    def position_callback(self, msg):
        cx, cy, distance = msg.x, msg.y, msg.z
        cmd = Twist()
        too_close_msg = Bool()
        self.last_seen_time = self.get_clock().now()

        if self.tracking_id is None:
            self.tracking_id = (cx, cy)  # 最初に検出した人を記録
        elif abs(cx - self.tracking_id[0]) > 50 or abs(cy - self.tracking_id[1]) > 50:
            return  # 別の人と判断し無視

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

    def check_lost_person(self):
        if (self.get_clock().now() - self.last_seen_time).nanoseconds * 1e-9 > self.lost_threshold:
            cmd = Twist()
            self.publisher_cmd.publish(cmd)  # 停止命令を送る


def main():
    rclpy.init()
    node = FollowPersonNode()
    timer_period = 0.5  # 0.5秒ごとに人を見失ったかチェック
    node.create_timer(timer_period, node.check_lost_person)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

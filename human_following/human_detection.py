#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class HumanDetectionNode(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        self.get_logger().info("人検出ノードを起動")

        self.sub_color = self.create_subscription(Image, '/camera/color/image_raw', self.color_callback, 10)
        self.sub_depth = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.publisher = self.create_publisher(Point, '/human_position', 10)

        self.bridge = CvBridge()
        self.yolo_model = YOLO('yolov8n.pt')

        self.latest_color_msg = None
        self.latest_depth_msg = None
        self.timer = self.create_timer(0.1, self.timer_callback)

    def color_callback(self, msg):
        self.latest_color_msg = msg

    def depth_callback(self, msg):
        self.latest_depth_msg = msg

    def timer_callback(self):
        if self.latest_color_msg is None or self.latest_depth_msg is None:
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(self.latest_color_msg, 'bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth_msg, '16UC1')
        except Exception as e:
            self.get_logger().error(f"画像変換エラー: {e}")
            return

        results = self.yolo_model(color_image)
        for result in results:
            boxes = result.boxes.data.cpu().numpy()
            for box in boxes:
                x1, y1, x2, y2, conf, cls = box
                if int(cls) == 0:
                    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                    if 0 <= cx < depth_image.shape[1] and 0 <= cy < depth_image.shape[0]:
                        distance = depth_image[cy, cx] / 1000.0

                        person_position = Point()
                        person_position.x = float(cx)  # floatに変換
                        person_position.y = float(cy)  # floatに変換
                        person_position.z = float(distance)  # floatに変換
                        self.publisher.publish(person_position)

                        text = f"Person: {distance:.2f}m"
                        cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.putText(color_image, text, (int(x1), int(y1)-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        self.get_logger().info(text)

        cv2.imshow("YOLOv8 Detection", color_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = HumanDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

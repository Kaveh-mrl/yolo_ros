#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Point32
from sensor_msgs.msg import CameraInfo

from yolo_msgs.msg import (
    DetectionArray,
    DetectionWithCorners,
    DetectionWithCornersArray,
)


class YoloCornersAdapter(Node):
    """
    Converts yolo_msgs/DetectionArray into yolo_msgs/DetectionWithCornersArray.

    The node:
      - reads image width/height from CameraInfo
      - computes bbox corners in normalized image coordinates
    """

    def __init__(self):
        super().__init__("yolo_corners_adapter")

        self.declare_parameter("input_detections_topic", "detections")
        self.declare_parameter("input_camera_info_topic", "camera_info")
        self.declare_parameter(
            "output_detections_corners_topic",
            "detections_with_corners",
        )

        input_detections_topic = (
            self.get_parameter("input_detections_topic")
            .get_parameter_value()
            .string_value
        )
        input_camera_info_topic = (
            self.get_parameter("input_camera_info_topic")
            .get_parameter_value()
            .string_value
        )
        output_topic = (
            self.get_parameter("output_detections_corners_topic")
            .get_parameter_value()
            .string_value
        )

        self.image_width = None
        self.image_height = None

        self.pub = self.create_publisher(
            DetectionWithCornersArray,
            output_topic,
            10,
        )

        self.create_subscription(
            DetectionArray,
            input_detections_topic,
            self.detections_cb,
            10,
        )

        self.create_subscription(
            CameraInfo,
            input_camera_info_topic,
            self.camera_info_cb,
            qos_profile_sensor_data,
        )

        self.get_logger().info("Detection corners adapter started")
        self.get_logger().info(f"Input detections: {input_detections_topic}")
        self.get_logger().info(f"Input camera info: {input_camera_info_topic}")
        self.get_logger().info(f"Output corners: {output_topic}")

    def camera_info_cb(self, msg: CameraInfo):
        self.image_width = int(msg.width)
        self.image_height = int(msg.height)

    def detections_cb(self, msg: DetectionArray):
        if self.image_width is None or self.image_height is None:
            self.get_logger().warn("No CameraInfo received yet, skipping detections")
            return

        out = DetectionWithCornersArray()
        out.header = msg.header
        out.image_width = self.image_width
        out.image_height = self.image_height

        for det in msg.detections:
            if float(det.bbox.size.x) <= 0.0 or float(det.bbox.size.y) <= 0.0:
                continue

            out.detections.append(self.convert_detection(det))

        self.pub.publish(out)

    def convert_detection(self, det):
        cx = float(det.bbox.center.position.x)
        cy = float(det.bbox.center.position.y)
        theta = float(det.bbox.center.theta)
        w = float(det.bbox.size.x)
        h = float(det.bbox.size.y)

        corners = self.obb_to_corners(cx, cy, w, h, theta)

        out = DetectionWithCorners()
        out.class_name = str(det.class_name)
        out.score = float(det.score)

        out.bbox.center = det.bbox.center
        out.bbox.size = det.bbox.size

        for px, py in corners:

            x_norm, y_norm = self.pixel_to_normalized(px, py)

            norm_point = Point32()
            norm_point.x = float(x_norm)
            norm_point.y = float(y_norm)
            norm_point.z = 0.0
            out.bbox.normalized_corners.points.append(norm_point)

        return out

    def pixel_to_normalized(self, px: float, py: float):
        x_norm = (px - self.image_width / 2.0) / (self.image_width / 2.0)
        y_norm = (py - self.image_height / 2.0) / (self.image_height / 2.0)

        return x_norm, y_norm

    @staticmethod
    def obb_to_corners(cx: float, cy: float, w: float, h: float, theta: float):
        local_corners = [
            (-w / 2.0, -h / 2.0),
            (w / 2.0, -h / 2.0),
            (w / 2.0, h / 2.0),
            (-w / 2.0, h / 2.0),
        ]

        c = math.cos(theta)
        s = math.sin(theta)

        corners = []
        for x, y in local_corners:
            px = cx + x * c - y * s
            py = cy + x * s + y * c
            corners.append((px, py))

        return corners


def main(args=None):
    rclpy.init(args=args)
    node = YoloCornersAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
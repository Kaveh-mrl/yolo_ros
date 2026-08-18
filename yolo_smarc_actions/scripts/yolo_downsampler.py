#!/usr/bin/env python3

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Image, CompressedImage


class ImageDownsampler(Node):
    def __init__(self):
        super().__init__("image_downsampler")

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter(
            "input_topic",
            "/M350/yolo/dbg_image",
        )
        self.declare_parameter(
            "output_topic",
            "/M350/yolo/dbg_image_down",
        )

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 360)

        # <= 0 disables FPS limiting
        self.declare_parameter("max_fps", 10.0)

        # Optional compressed output:
        # <output_topic>/compressed
        self.declare_parameter("publish_compressed", False)
        self.declare_parameter("jpeg_quality", 70)

        self.input_topic = (
            self.get_parameter("input_topic")
            .get_parameter_value()
            .string_value
        )

        self.output_topic = (
            self.get_parameter("output_topic")
            .get_parameter_value()
            .string_value
        )

        self.width = (
            self.get_parameter("width")
            .get_parameter_value()
            .integer_value
        )

        self.height = (
            self.get_parameter("height")
            .get_parameter_value()
            .integer_value
        )

        self.max_fps = (
            self.get_parameter("max_fps")
            .get_parameter_value()
            .double_value
        )

        self.publish_compressed = (
            self.get_parameter("publish_compressed")
            .get_parameter_value()
            .bool_value
        )

        self.jpeg_quality = (
            self.get_parameter("jpeg_quality")
            .get_parameter_value()
            .integer_value
        )

        # ---------------------------------------------------------------------
        # QoS
        # ---------------------------------------------------------------------

        # Input from YOLO:
        input_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Output for web/visualization:
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            self.output_topic,
            output_qos,
        )

        self.compressed_publisher = None

        if self.publish_compressed:
            compressed_topic = self.output_topic + "/compressed"

            self.compressed_publisher = self.create_publisher(
                CompressedImage,
                compressed_topic,
                output_qos,
            )

            self.get_logger().info(
                f"Compressed output: {compressed_topic}"
            )

        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            input_qos,
        )

        # Timestamp of last frame actually processed/published
        self.last_publish_time_ns = None

        if self.max_fps > 0.0:
            self.min_period_ns = int(1e9 / self.max_fps)
        else:
            self.min_period_ns = 0

        self.get_logger().info(
            "Image downsampler started:\n"
            f"  input:   {self.input_topic}\n"
            f"  output:  {self.output_topic}\n"
            f"  size:    {self.width}x{self.height}\n"
            f"  max FPS: {self.max_fps}"
        )

    def image_callback(self, msg: Image):
        now_ns = self.get_clock().now().nanoseconds

        # ---------------------------------------------------------------------
        # FPS throttle FIRST.
        #
        # Don't waste CPU converting/resizing frames that we are going
        # to discard anyway.
        # ---------------------------------------------------------------------
        if (
            self.min_period_ns > 0
            and self.last_publish_time_ns is not None
        ):
            elapsed_ns = now_ns - self.last_publish_time_ns

            if elapsed_ns < self.min_period_ns:
                return

        self.last_publish_time_ns = now_ns

        try:
            # YOLO debug output is normally color
            image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            resized = cv2.resize(
                image,
                (self.width, self.height),
                interpolation=cv2.INTER_AREA,
            )

            # -------------------------------------------------------------
            # Publish normal sensor_msgs/Image
            # -------------------------------------------------------------
            output_msg = self.bridge.cv2_to_imgmsg(
                resized,
                encoding="bgr8",
            )

            output_msg.header = msg.header

            self.publisher.publish(output_msg)

            # -------------------------------------------------------------
            # Optional JPEG-compressed topic
            # -------------------------------------------------------------
            if self.compressed_publisher is not None:
                success, encoded = cv2.imencode(
                    ".jpg",
                    resized,
                    [
                        cv2.IMWRITE_JPEG_QUALITY,
                        self.jpeg_quality,
                    ],
                )

                if success:
                    compressed_msg = CompressedImage()

                    compressed_msg.header = msg.header
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = encoded.tobytes()

                    self.compressed_publisher.publish(
                        compressed_msg
                    )

        except Exception as exc:
            self.get_logger().error(
                f"Image processing failed: {exc}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = ImageDownsampler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
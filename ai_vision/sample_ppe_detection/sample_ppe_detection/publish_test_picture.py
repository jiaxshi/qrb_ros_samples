# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import os
import glob


SUPPORTED_EXTS = (".jpg", ".jpeg", ".png", ".bmp")


class PublishPicture(Node):
    """
    Reads image(s) from disk and publishes them to /image_raw at a
    configurable rate.  Supports single file or a directory.
    """

    def __init__(self):
        super().__init__("publish_picture")

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter("image_path", "/opt/test_images/")
        self.declare_parameter("topic",      "/image_raw")
        self.declare_parameter("rate",       1.0)    # Hz
        self.declare_parameter("loop",       True)   # loop over images

        image_path = self.get_parameter("image_path").value
        topic      = self.get_parameter("topic").value
        rate_hz    = self.get_parameter("rate").value
        self.loop  = self.get_parameter("loop").value

        # ── Collect image list ────────────────────────────────────────────
        self.image_list = self._collect_images(image_path)
        if not self.image_list:
            self.get_logger().error(f"No images found at: {image_path}")
            raise SystemExit(1)
        self.get_logger().info(
            f"[PublishPicture] Found {len(self.image_list)} image(s). "
            f"Publishing to '{topic}' at {rate_hz} Hz")

        self.index  = 0
        self.bridge = CvBridge()
        self.pub    = self.create_publisher(Image, topic, 10)
        self.timer  = self.create_timer(1.0 / rate_hz, self._timer_callback)

    # ── Image collection ───────────────────────────────────────────────────
    def _collect_images(self, path: str):
        """Return sorted list of image paths (file or directory)."""
        if os.path.isfile(path):
            return [path]
        elif os.path.isdir(path):
            files = []
            for ext in SUPPORTED_EXTS:
                files += glob.glob(os.path.join(path, f"*{ext}"))
                files += glob.glob(os.path.join(path, f"*{ext.upper()}"))
            return sorted(set(files))
        else:
            self.get_logger().error(f"Path not found: {path}")
            return []

    # ── Timer callback ────────────────────────────────────────────────────
    def _timer_callback(self):
        if self.index >= len(self.image_list):
            if self.loop:
                self.index = 0
                self.get_logger().info("[PublishPicture] Looping images...")
            else:
                self.get_logger().info("[PublishPicture] All images published. Stopping.")
                self.timer.cancel()
                return

        img_path = self.image_list[self.index]
        frame = cv2.imread(img_path)
        if frame is None:
            self.get_logger().warn(f"Cannot read image: {img_path}")
            self.index += 1
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self.pub.publish(msg)
            self.get_logger().info(
                f"[PublishPicture] Published [{self.index + 1}/{len(self.image_list)}]: "
                f"{os.path.basename(img_path)}")
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PublishPicture()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

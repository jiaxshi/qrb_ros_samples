#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class PublishVideo(Node):

    def __init__(self):
        super().__init__("publish_video")

        # ── Parameters ───────────────────────────────────────────────────
        # video_path: path to a video file, OR a numeric string like "0" for a
        #             camera device index, OR "/dev/videoX".
        self.declare_parameter("video_path", "/data/videos/ppe_test.mp4")
        self.declare_parameter("topic",      "/image_raw")
        # rate: how many frames per second we PUBLISH (not the video's own fps).
        #       Keep it low (2~5) because NPU inference is serial.
        self.declare_parameter("rate",       2.0)     # Hz
        self.declare_parameter("loop",       True)    # restart video when it ends
        # frame_step: if > 0, publish every Nth decoded frame instead of using
        #             time-based sampling. 0 = auto (derive from source fps/rate).
        self.declare_parameter("frame_step", 0)

        video_path_param = str(self.get_parameter("video_path").value)
        topic            = self.get_parameter("topic").value
        rate_hz          = float(self.get_parameter("rate").value)
        self.loop        = bool(self.get_parameter("loop").value)
        self.frame_step  = int(self.get_parameter("frame_step").value)

        if rate_hz <= 0.0:
            self.get_logger().error("rate must be > 0")
            raise SystemExit(1)

        # ── Open capture source (file or camera index/device) ─────────────
        self.source, self.is_camera = self._resolve_source(video_path_param)
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open video source: {video_path_param}")
            raise SystemExit(1)

        # Guard against an infinite "looping..." spin: if we just reopened the
        # file but still read nothing, stop instead of spinning forever.
        self._reopened_no_frame = False

        src_fps   = self.cap.get(cv2.CAP_PROP_FPS) or 0.0
        frame_cnt = self.cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0.0

        # Decide how many source frames to skip between publishes so we don't
        # decode-then-drop. e.g. 30 fps source publishing at 2 Hz → skip 15.
        if self.frame_step > 0:
            self._step = self.frame_step
        elif not self.is_camera and src_fps > 0.0:
            self._step = max(1, int(round(src_fps / rate_hz)))
        else:
            self._step = 1  # camera or unknown fps: take latest frame each tick

        self.get_logger().info(
            f"[PublishVideo] source={video_path_param} (camera={self.is_camera}) "
            f"src_fps={src_fps:.1f} frames={int(frame_cnt)}")
        self.get_logger().info(
            f"[PublishVideo] publishing to '{topic}' at {rate_hz} Hz, "
            f"loop={self.loop}, sampling every {self._step} source frame(s)")

        self.published = 0
        self.bridge    = CvBridge()
        self.pub       = self.create_publisher(Image, topic, 10)
        # Timer fires at the PUBLISH rate; each tick grabs one (sampled) frame.
        self.timer     = self.create_timer(1.0 / rate_hz, self._timer_callback)

    # ── Resolve "0" → camera index 0, "/dev/videoX" or path → file/device ──
    def _resolve_source(self, value: str):
        if value.isdigit():
            return int(value), True
        if value.startswith("/dev/video"):
            return value, True
        return value, False

    # ── Skip (step-1) frames cheaply, then read one; for file sources ──────
    def _grab_sampled_frame(self):
        for _ in range(self._step - 1):
            if not self.cap.grab():       # grab() decodes-and-discards cheaply
                return None
        ok, frame = self.cap.read()
        return frame if ok else None

    # ── Timer callback ─────────────────────────────────────────────────────
    def _timer_callback(self):
        if self.is_camera:
            # Camera: take the latest frame (real-time stream).
            ok, frame = self.cap.read()
        else:
            frame = self._grab_sampled_frame()
            ok = frame is not None

        # End-of-video handling (files only).
        if not ok or frame is None:
            if self.is_camera:
                self.get_logger().warn("[PublishVideo] Camera read failed.")
                return
            if self.loop:
                # Some containers/codecs (esp. via the GStreamer backend) don't
                # support CAP_PROP_POS_FRAMES seek, so rewind fails and we'd spin
                # forever printing "looping...". Reopen the file instead — more
                # reliable than seeking.
                if self._reopened_no_frame:
                    self.get_logger().error(
                        "[PublishVideo] Reopened video but still no frame; "
                        "seek/loop unsupported for this file. Stopping. "
                        "(Tip: re-encode to a seekable format, e.g. "
                        "ffmpeg -i in.mp4 -an -c:v mjpeg out.avi)")
                    self.timer.cancel()
                    if self.cap is not None:
                        self.cap.release()
                    return
                self.get_logger().info("[PublishVideo] End of video, reopening to loop...")
                self.cap.release()
                self.cap = cv2.VideoCapture(self.source)
                self._reopened_no_frame = True   # cleared once a frame is read
                return
            else:
                self.get_logger().info(
                    f"[PublishVideo] Video finished. "
                    f"Published {self.published} frame(s). Stopping.")
                self.timer.cancel()
                self.cap.release()
                return

        try:
            # ppe_detection_node subscribes /image_raw and decodes with
            # desired_encoding='rgb8'; publishing bgr8 (OpenCV native) lets
            # cv_bridge convert correctly — same as publish_test_picture.py.
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            self.pub.publish(msg)
            self.published += 1
            self._reopened_no_frame = False   # got a frame → reset loop guard
            self.get_logger().info(
                f"[PublishVideo] Published frame #{self.published} "
                f"({frame.shape[1]}x{frame.shape[0]})")
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")

    def destroy_node(self):
        if getattr(self, "cap", None) is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PublishVideo()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

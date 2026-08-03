#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import numpy as np
from PIL import Image as PILImage
from PIL import ImageDraw
import json
import cv2

from qrb_ros_tensor_list_msgs.msg import Tensor, TensorList
# shape: (1,3,320,192)
TARGET_W    = 192
TARGET_H    = 320

CONF_THRESH = 0.5
IOU_THRESH  = 0.5

LABELS = ['helmet', 'vest']
COLORS = {'helmet': 'red', 'vest': 'green'}

DTYPE_MAP = {
    0: np.uint8,
    1: np.int8,
    2: np.float32,
    3: np.float64,
}

BOXES_SCALE  = 1.5097410678863525
BOXES_ZP     = 27
SCORES_SCALE = 0.003834740025922656
SCORES_ZP    = 0


class PPEDetectionNode(Node):

    def __init__(self):
        super().__init__('ppe_detection_node')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('conf_thresh', CONF_THRESH)
        self.declare_parameter('iou_thresh',  IOU_THRESH)
        self.declare_parameter('save_path', '')
        self.declare_parameter('save_video_path', '')
        self.declare_parameter('output_fps', 2.0)
        self.declare_parameter('box_hold_frames', 5)
        self.conf_thresh = self.get_parameter('conf_thresh').value
        self.iou_thresh  = self.get_parameter('iou_thresh').value
        self.save_path   = self.get_parameter('save_path').value
        self.save_video_path = self.get_parameter('save_video_path').value
        self.output_fps  = float(self.get_parameter('output_fps').value)
        self.box_hold_frames = int(self.get_parameter('box_hold_frames').value)

        self._held = {}

        self.video_writer = None

        self._pending_meta = None  # (orig_img, scale, pad_x, pad_y, width, height, header)
        # ── ROS I/O ──────────────────────────────────────────────────────
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        self.qnn_input_pub = self.create_publisher(
            TensorList, 'qrb_inference_input_tensor', 10)

        self.qnn_output_sub = self.create_subscription(
            TensorList, 'qrb_inference_output_tensor',
            self.infer_callback, 10)

        self.pub_result = self.create_publisher(
            String, '/ppe_detection/result', 10)
        self.pub_image = self.create_publisher(
            Image,  '/ppe_detection/image',  10)

        self.get_logger().info('Subscribe /image_raw')
        self.get_logger().info(
            f'[PPEDetectionNode] Ready  input=(1,3,{TARGET_H},{TARGET_W})  '
            f'via topics qrb_inference_input_tensor / qrb_inference_output_tensor')
    # ── Letterbox resize ─────────────────────
    def letterbox_resize(self, pil_img, target_w=TARGET_W, target_h=TARGET_H):
        orig_w, orig_h = pil_img.size
        scale = min(target_w / orig_w, target_h / orig_h)
        new_w = int(orig_w * scale)
        new_h = int(orig_h * scale)
        resized = pil_img.resize((new_w, new_h))

        letterboxed = PILImage.new('RGB', (target_w, target_h), (0, 0, 0))
        pad_x = (target_w - new_w) // 2
        pad_y = (target_h - new_h) // 2
        letterboxed.paste(resized, (pad_x, pad_y))
        return letterboxed, scale, pad_x, pad_y

    # ── NMS ───────────────────────────────────
    def simple_nms(self, boxes, scores, iou_threshold=0.5):
        x1 = boxes[:, 0]
        y1 = boxes[:, 1]
        x2 = boxes[:, 2]
        y2 = boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou   = inter / (areas[i] + areas[order[1:]] - inter)
            order = order[1:][iou < iou_threshold]
        return np.array(keep)

    # ── main callback ──────────
    def image_callback(self, msg):
        if self._pending_meta is not None:
            self.get_logger().warn('Previous frame is still being processed, skipping current frame')
            return
        self.get_logger().debug('Received image message')
        self.get_logger().debug(
            f'  image size: {msg.height}x{msg.width}, encoding: {msg.encoding}')

        try:
            img_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f'Failed to decode image: {e}')
            return

        pil_img = PILImage.fromarray(img_array, 'RGB')
        pil_img_lb, scale, pad_x, pad_y = self.letterbox_resize(pil_img)
        self.get_logger().debug(
            f'  letterbox: scale={scale:.4f}, pad_x={pad_x}, pad_y={pad_y}')

        input_arr = np.array(pil_img_lb, dtype=np.uint8)
        input_arr = np.expand_dims(input_arr, axis=0)   # (H,W,C) -> (1,H,W,C)
        input_arr = np.ascontiguousarray(input_arr)
        self.get_logger().debug(
            f'  input shape: {input_arr.shape}, dtype: {input_arr.dtype} (NHWC)')
        self.get_logger().debug(
            f'  [DIAG] preprocessed pixel value: '
            f'min={input_arr.min()} max={input_arr.max()} '
            f'mean={input_arr.astype(np.float32).mean():.2f}')

        self._pending_meta = (img_array, scale, pad_x, pad_y, msg.width, msg.height, msg.header)

        tensor = Tensor()
        tensor.data_type = 0  # uint8
        tensor.name = "gear_guard_input_tensor"
        tensor.shape = list(input_arr.shape)  # [1, 320, 192, 3] NHWC
        tensor.data = input_arr.tobytes()

        tensor_list = TensorList()
        tensor_list.tensor_list.append(tensor)
        self.qnn_input_pub.publish(tensor_list)
        self.get_logger().debug('Published input tensor, waiting for inference result...')

    def infer_callback(self, msg):
        if self._pending_meta is None:
            self.get_logger().warn('Received inference result but no pending frame metadata, dropping')
            return

        img_array, scale, pad_x, pad_y, width, height, header = self._pending_meta
        self._pending_meta = None

        try:
            tensors = msg.tensor_list
            tensor_by_name = {t.name: t for t in tensors}
            self.get_logger().debug(
                f'  received {len(tensors)} output tensor(s): '
                f'{[(t.name, t.data_type, list(t.shape)) for t in tensors]}')

            def _to_array(t):
                dtype = DTYPE_MAP.get(t.data_type, np.float32)
                return np.frombuffer(t.data, dtype=dtype)

            raw_boxes = _to_array(tensor_by_name['boxes'])
            raw_scores = _to_array(tensor_by_name['scores'])
            raw_class_idx = _to_array(tensor_by_name['class_idx'])

            boxes = (raw_boxes.astype(np.float32).reshape(-1, 4) - BOXES_ZP) * BOXES_SCALE
            scores = (raw_scores.astype(np.float32).reshape(-1) - SCORES_ZP) * SCORES_SCALE
            class_idx = raw_class_idx.astype(np.int32).reshape(-1)

            self.get_logger().debug(
                f'  scores nonzero: {np.count_nonzero(raw_scores)}/{raw_scores.size}, '
                f'max={scores.max():.4f}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse output tensors: {e}')
            return
        self.get_logger().debug(
            f'  class of highest score: {int(class_idx[scores.argmax()])}')

        mask               = scores > self.conf_thresh
        filtered_boxes     = boxes[mask]
        filtered_scores    = scores[mask]
        filtered_class_idx = class_idx[mask]
        self.get_logger().debug(f'  boxes after confidence filter: {len(filtered_boxes)}')

        if self.get_logger().get_effective_level() <= LoggingSeverity.DEBUG:
            for j in range(len(filtered_boxes)):
                bx = filtered_boxes[j]
                cj = int(filtered_class_idx[j])
                lbl = LABELS[cj] if cj < len(LABELS) else str(cj)
                self.get_logger().debug(
                    f'    [pre-NMS] cand[{j}] cls={cj}({lbl}) '
                    f'score={filtered_scores[j]:.3f} '
                    f'box=({bx[0]:.1f},{bx[1]:.1f},{bx[2]:.1f},{bx[3]:.1f})')

        keep_indices = []
        for cls_id in range(len(LABELS)):
            cls_mask = filtered_class_idx == cls_id
            if cls_mask.sum() == 0:
                continue
            keep = self.simple_nms(
                filtered_boxes[cls_mask],
                filtered_scores[cls_mask],
                iou_threshold=self.iou_thresh
            )
            keep_indices.extend(np.where(cls_mask)[0][keep])

        final_boxes     = filtered_boxes[keep_indices]
        final_scores    = filtered_scores[keep_indices]
        final_class_idx = filtered_class_idx[keep_indices]
        self.get_logger().debug(f'  boxes after NMS: {len(final_boxes)}')

        detections = []
        for i, box in enumerate(final_boxes):
            x1_raw, y1_raw, x2_raw, y2_raw = box
            x1 = max(0,           int((x1_raw - pad_x) / scale))
            y1 = max(0,           int((y1_raw - pad_y) / scale))
            x2 = min(width - 1,   int((x2_raw - pad_x) / scale))
            y2 = min(height - 1,  int((y2_raw - pad_y) / scale))
            self.get_logger().debug(
                f'  box[{i}] raw=({x1_raw:.1f},{y1_raw:.1f},{x2_raw:.1f},{y2_raw:.1f})'
                f'  mapped=({x1},{y1},{x2},{y2})')
            score = float(final_scores[i])
            cls   = int(final_class_idx[i])
            label = LABELS[cls] if cls < len(LABELS) else str(cls)
            detections.append({
                'label': label, 'cls': cls, 'score': round(score, 4),
                'box': [x1, y1, x2, y2],
            })

        draw_dets = self._apply_box_hold(detections)

        result_img = PILImage.fromarray(img_array, 'RGB')
        draw       = ImageDraw.Draw(result_img)
        for d in draw_dets:
            x1, y1, x2, y2 = d['box']
            label = d['label']
            color = COLORS.get(label, 'red')
            text  = f"{label}: {d['score']:.2f}"
            text_h = 18
            draw.rectangle([x1, y1, x1 + len(text) * 7, y1 + text_h], fill=color)
            draw.rectangle([x1, y1, x2, y2], outline=color, width=3)
            draw.text((x1 + 2, y1 + 2), text, fill='white')

        result_msg = String()
        result_msg.data = json.dumps(
                        {'count': len(draw_dets),
             'detections': [{'label': d['label'], 'score': d['score'],
                             'box': d['box']} for d in draw_dets]})
        self.pub_result.publish(result_msg)
        self.get_logger().info(f'Detection result: {result_msg.data}')

        annotated = np.array(result_img, dtype=np.uint8)
        img_msg   = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
        img_msg.header = header
        self.pub_image.publish(img_msg)

        if self.save_path:
            try:
                result_img.save(self.save_path)
                self.get_logger().debug(f'  annotated image saved to: {self.save_path}')
            except Exception as e:
                self.get_logger().error(f'Failed to save annotated image: {e}')

        if self.save_video_path:
            try:
                frame_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
                if self.video_writer is None:
                    h, w = frame_bgr.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                    self.video_writer = cv2.VideoWriter(
                        self.save_video_path, fourcc, self.output_fps, (w, h))
                    if not self.video_writer.isOpened():
                        self.get_logger().error(
                            f'Failed to create video writer: {self.save_video_path} '
                            f'(try a .avi extension or check OpenCV codec support)')
                        self.save_video_path = ''   # disable recording to avoid repeated errors
                        self.video_writer = None
                    else:
                        self.get_logger().info(
                            f'  Start recording demo video: {self.save_video_path} '
                            f'({w}x{h} @ {self.output_fps}fps, MJPG)')
                if self.video_writer is not None:
                    self.video_writer.write(frame_bgr)
            except Exception as e:
                self.get_logger().error(f'Failed to write demo video: {e}')

        self.get_logger().debug('Inference done')

    # Temporal smoothing: reuse the most recently detected box to fill in a
    # class missed in the current frame (reduces per-frame detection flicker).
    def _apply_box_hold(self, detections):
        if self.box_hold_frames <= 0:
            return detections

        # Keep the highest-scoring detection per class for this frame
        # (this model scenario usually has one target per class).
        best_by_cls = {}
        for d in detections:
            c = d['cls']
            if c not in best_by_cls or d['score'] > best_by_cls[c]['score']:
                best_by_cls[c] = d

        out = []
        for c in range(len(LABELS)):
            if c in best_by_cls:
                self._held[c] = {'det': best_by_cls[c], 'age': 0}
                out.append(best_by_cls[c])
            elif c in self._held:
                held = self._held[c]
                if held['age'] < self.box_hold_frames:
                    held['age'] += 1
                    out.append(held['det'])   
                    self.get_logger().debug(
                        f"  [hold] {LABELS[c]} reusing previous box "
                        f"(age={held['age']}/{self.box_hold_frames})")
                else:
                    del self._held[c]      
        return out

    def destroy_node(self):
        if getattr(self, 'video_writer', None) is not None:
            self.video_writer.release()
            self.get_logger().info(f'Demo video saved: {self.save_video_path}')
        super().destroy_node()


def main():
    rclpy.init()
    node = PPEDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

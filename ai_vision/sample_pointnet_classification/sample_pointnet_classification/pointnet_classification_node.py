# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import rclpy
import numpy as np
import struct
import threading
import signal
import sys
import time
import json
import os

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from qrb_ros_tensor_list_msgs.msg import Tensor, TensorList

N_POINTS = 1024


class PointNetClassificationNode(Node):

    def __init__(self):
        super().__init__('pointnet_classification_node')
        self.stopping = False
        self.inference_in_progress = False
        self.inference_lock = threading.Lock()

        _share = get_package_share_directory('sample_pointnet_classification')
        classes_json = self.declare_parameter(
            'classes_path', os.path.join(_share, 'classes.json')
        ).get_parameter_value().string_value
        self.idx_to_class = self._load_classes(classes_json)

        self.subscriber = self.create_subscription(
            PointCloud2,
            '/livox/lidar',
            self.preprocess,
            10
        )
        self.publisher = self.create_publisher(
            TensorList,
            'qrb_inference_input_tensor',
            10
        )
        self.result_publisher = self.create_publisher(
            String,
            'pointnet_output',
            10
        )
        self.infer_subscriber = self.create_subscription(
            TensorList,
            'qrb_inference_output_tensor',
            self.infer_callback,
            10
        )
        self.get_logger().info('Initialized PointNetClassificationNode')

    def _load_classes(self, path: str) -> dict:
        try:
            with open(path, 'r') as f:
                raw = json.load(f)
            idx_to_class = {int(k): v for k, v in raw.items()}
            self.get_logger().info(f'Loaded {len(idx_to_class)} classes from {path}')
            return idx_to_class
        except Exception as e:
            self.get_logger().warning(
                f'Failed to load classes from {path}: {e}. Using index only.'
            )
            return {}

    def _pointcloud2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        """Parse PointCloud2 binary buffer → float32 array (N, 3)."""
        field_offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in field_offsets for k in ('x', 'y', 'z')):
            raise ValueError('PointCloud2 message missing x/y/z fields')

        point_step = msg.point_step
        n_points = msg.width * msg.height
        data = bytes(msg.data)

        xyz = np.empty((n_points, 3), dtype=np.float32)
        for col, name in enumerate(('x', 'y', 'z')):
            offset = field_offsets[name]
            xyz[:, col] = struct.unpack_from(
                f'<{n_points}f',
                b''.join(
                    data[i * point_step + offset: i * point_step + offset + 4]
                    for i in range(n_points)
                )
            )
        return xyz

    def _sample_points(self, xyz: np.ndarray, n: int) -> np.ndarray:
        """Random sample (with replacement when needed) to exactly n points."""
        replace = len(xyz) < n
        idx = np.random.choice(len(xyz), n, replace=replace)
        return xyz[idx]

    def _normalize(self, xyz: np.ndarray) -> np.ndarray:
        """Center and scale to unit sphere (standard PointNet preprocessing)."""
        xyz = xyz - xyz.mean(axis=0)
        scale = np.max(np.linalg.norm(xyz, axis=1))
        if scale > 0:
            xyz /= scale
        return xyz

    def preprocess(self, msg: PointCloud2):
        if self.stopping:
            return
        self.get_logger().debug(
            f'[preprocess] received PointCloud2: width={msg.width} height={msg.height} '
            f'total_points={msg.width * msg.height} point_step={msg.point_step} '
            f'fields={[f.name for f in msg.fields]}'
        )
        try:
            xyz = self._pointcloud2_to_xyz(msg)

            # Filter out NaN/Inf points (common in Livox scans)
            valid = np.isfinite(xyz).all(axis=1)
            xyz = xyz[valid]
            self.get_logger().debug(
                f'[preprocess] valid points after NaN filter: '
                f'{len(xyz)}/{msg.width * msg.height}'
            )
            if len(xyz) == 0:
                self.get_logger().warning(
                    'PointCloud2 message contained no valid points; skipping.'
                )
                return

            xyz = self._sample_points(xyz, N_POINTS)
            xyz = self._normalize(xyz)
            self.get_logger().debug(
                f'[preprocess] tensor ready, publishing to qrb_inference_input_tensor'
            )

            # (N, 3) → (3, N) → (1, 3, N) → flat bytes
            tensor_data = xyz.T.astype(np.float32).tobytes()  # shape [3, 1024]

            out_msg = TensorList()
            tensor = Tensor()
            tensor.data_type = 0  # float32
            tensor.name = 'pointnet_input_tensor'
            tensor.shape = [1, 3, N_POINTS]
            tensor.data = tensor_data
            out_msg.tensor_list.append(tensor)

            with self.inference_lock:
                if not self.stopping:
                    self.inference_in_progress = True
                    try:
                        self.publisher.publish(out_msg)
                    except Exception as e:
                        self.inference_in_progress = False
                        self.get_logger().info(f'Failed to publish TensorList: {e}')

        except Exception as e:
            self.get_logger().error(f'Error processing PointCloud2: {e}')

    def infer_callback(self, msg: TensorList):
        try:
            with self.inference_lock:
                self.inference_in_progress = False

            if self.stopping:
                return

            self.get_logger().debug(
                f'infer_callback received {len(msg.tensor_list)} tensor(s)'
            )
            for i, t in enumerate(msg.tensor_list):
                self.get_logger().debug(
                    f'  tensor[{i}] name={t.name} shape={list(t.shape)} '
                    f'dtype={t.data_type} len(data)={len(t.data)}'
                )

            logits_tensor = next(
                (t for t in msg.tensor_list if t.name == 'x'), None
            )
            if logits_tensor is None:
                raise ValueError(
                    f'Tensor "x" not found in output. '
                    f'Available: {[t.name for t in msg.tensor_list]}'
                )
            result = self.postprocess(logits_tensor)
            out_msg = String()
            out_msg.data = result
            self.result_publisher.publish(out_msg)
            self.get_logger().info(f'PointNet classification: {result}')
        except Exception as e:
            with self.inference_lock:
                self.inference_in_progress = False
            if not self.stopping:
                self.get_logger().error(f'Error processing inference output: {e}')

    def postprocess(self, tensor) -> str:
        raw_data = getattr(tensor, 'data', None)
        if raw_data is None or len(raw_data) == 0:
            raise ValueError('Empty tensor data')
        log_probs = np.frombuffer(bytes(raw_data), dtype=np.float32)
        predicted_idx = int(np.argmax(log_probs))
        class_name = self.idx_to_class.get(predicted_idx, str(predicted_idx))
        return f'[{predicted_idx}] {class_name}'


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = None
    executor = None
    shutdown_requested = threading.Event()

    def signal_handler(signum, frame):
        """Custom signal handler for graceful shutdown."""
        print(f'\nReceived signal {signum}, initiating graceful shutdown...')
        shutdown_requested.set()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        node = PointNetClassificationNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)

        while rclpy.ok() and not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.1)

    except Exception as e:
        print(f'Exception during execution: {e}')
    finally:
        print('Starting cleanup sequence...')

        if node:
            node.stopping = True

            max_wait_time = 3.0
            wait_interval = 0.1
            elapsed = 0.0

            print('Waiting for in-flight inference to complete...')
            while elapsed < max_wait_time:
                with node.inference_lock:
                    if not node.inference_in_progress:
                        break
                time.sleep(wait_interval)
                elapsed += wait_interval

            if elapsed >= max_wait_time:
                print('Warning: Inference did not complete within timeout')
            else:
                print('In-flight inference completed successfully')

            print('Allowing QNN node to complete cleanup...')
            time.sleep(1.5)

        if executor:
            try:
                print('Shutting down executor...')
                executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass

        if node:
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

        print('Shutdown complete')
        sys.exit(0)


if __name__ == '__main__':
    main()
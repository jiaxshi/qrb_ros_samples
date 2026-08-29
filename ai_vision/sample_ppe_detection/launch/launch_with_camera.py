# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

# Launch PPE detection from a live USB (V4L2) camera.
#
#   image source : usb_cam (ros-jazzy-usb-cam)  ->  /image_raw
#   inference    : qrb_ros_nn_inference (NPU / HTP, ComposableNode)
#   post-process : ppe_detection_node  ->  /ppe_detection/{image,result}
#
# NOTE: the USB camera pushes frames at `framerate`, but NPU inference is
#       serial. ppe_detection_node self-throttles: while a frame is still
#       being inferred it drops incoming frames, so effective throughput
#       matches the NPU. Lower `framerate` to reduce wasted CPU decoding.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ── Camera arguments (forwarded to usb_cam) ─────────────────────────
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 device node of the USB camera (e.g. /dev/video0).'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Capture width in pixels.'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Capture height in pixels.'
    )

    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='30.0',
        description='Camera capture rate (Hz). ppe_detection_node drops frames '
                    'it cannot keep up with; lower this to reduce CPU decoding.'
    )

    # yuyv2rgb: widest webcam compatibility (esp. 640x480).
    # mjpeg2rgb: needed by most webcams for 720p/1080p at high fps.
    pixel_format_arg = DeclareLaunchArgument(
        'pixel_format',
        default_value='yuyv2rgb',
        description='usb_cam pixel format. Use "mjpeg2rgb" for 720p/1080p webcams. '
                    'Output is rgb8, as expected by ppe_detection_node.'
    )

    # ── Model / backend arguments ───────────────────────────────────────
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/opt/model/gear_guard_net_ctx.bin',
        description='Path to the PPE detection model. Must be a precompiled QNN '
                    'context binary (.bin), NOT the raw .dlc file.'
    )

    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value='/usr/lib/libQnnHtp.so',
        description='QNN backend library (HTP for NPU).'
    )

    # ── Detection / output arguments (forwarded to ppe_detection_node) ───
    conf_thresh_arg = DeclareLaunchArgument(
        'conf_thresh',
        default_value='0.5',
        description='Confidence threshold for detections.'
    )

    iou_thresh_arg = DeclareLaunchArgument(
        'iou_thresh',
        default_value='0.5',
        description='IoU threshold for per-class NMS.'
    )

    box_hold_frames_arg = DeclareLaunchArgument(
        'box_hold_frames',
        default_value='5',
        description='Keep last detected box for N frames to reduce flicker. 0 = off.'
    )

    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='',
        description='If set, overwrite-save the latest annotated frame to this image file.'
    )

    save_video_path_arg = DeclareLaunchArgument(
        'save_video_path',
        default_value='',
        description='If set, record annotated frames into this video file (.avi/MJPG recommended).'
    )

    output_fps_arg = DeclareLaunchArgument(
        'output_fps',
        default_value='2.0',
        description='Frame rate of the recorded demo video (match effective throughput).'
    )

    # ── Launch Configurations ───────────────────────────────────────────
    video_device = LaunchConfiguration('video_device')
    image_width  = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    framerate    = LaunchConfiguration('framerate')
    pixel_format = LaunchConfiguration('pixel_format')
    model_path   = LaunchConfiguration('model_path')
    backend      = LaunchConfiguration('backend')
    conf_thresh     = LaunchConfiguration('conf_thresh')
    iou_thresh      = LaunchConfiguration('iou_thresh')
    box_hold_frames = LaunchConfiguration('box_hold_frames')
    save_path       = LaunchConfiguration('save_path')
    save_video_path = LaunchConfiguration('save_video_path')
    output_fps      = LaunchConfiguration('output_fps')

    namespace = 'ppe_detection_container'

    # ── USB Camera Node (image source) ──────────────────────────────────
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'image_width':  image_width,
            'image_height': image_height,
            'framerate':    framerate,
            'pixel_format': pixel_format,
            'camera_name':  'usb_cam',
            'frame_id':     'usb_cam',
        }],
        remappings=[
            ('image_raw', '/image_raw'),
        ]
    )

    # ── QNN Inference Node (ComposableNode) ─────────────────────────────
    nn_inference_node = ComposableNode(
        package='qrb_ros_nn_inference',
        namespace=namespace,
        plugin='qrb_ros::nn_inference::QrbRosInferenceNode',
        name='nn_inference_node',
        parameters=[{
            'backend_option': backend,
            'model_path': model_path,
            'log_level': 'info',
        }]
    )

    # ── Container for ComposableNodes ───────────────────────────────────
    container = ComposableNodeContainer(
        name='ppe_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[nn_inference_node],
        sigterm_timeout='3',
        sigkill_timeout='5'
    )

    # ── PPE Detection Node ──────────────────────────────────────────────
    ppe_detection_node = Node(
        package='sample_ppe_detection',
        executable='ppe_detection_node',
        name='ppe_detection_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'conf_thresh':     conf_thresh,
            'iou_thresh':      iou_thresh,
            'box_hold_frames': box_hold_frames,
            'save_path':       save_path,
            'save_video_path': save_video_path,
            'output_fps':      output_fps,
        }],
        remappings=[
            ('qrb_inference_input_tensor',
             '/' + namespace + '/qrb_inference_input_tensor'),
            ('qrb_inference_output_tensor',
             '/' + namespace + '/qrb_inference_output_tensor'),
        ]
    )

    return LaunchDescription([
        video_device_arg,
        image_width_arg,
        image_height_arg,
        framerate_arg,
        pixel_format_arg,
        model_path_arg,
        backend_arg,
        conf_thresh_arg,
        iou_thresh_arg,
        box_hold_frames_arg,
        save_path_arg,
        save_video_path_arg,
        output_fps_arg,
        LogInfo(msg=['   Starting PPE Detection with USB CAMERA']),
        LogInfo(msg=['   Device: ', video_device,
                     '  (', image_width, 'x', image_height, ' @ ', framerate, ' Hz, ', pixel_format, ')']),
        LogInfo(msg=['   Model : ', model_path]),
        usb_cam_node,
        container,
        ppe_detection_node,
    ])

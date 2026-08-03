import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # ── Launch Arguments ────────────────────────────────────────────────
    # video_path: a video file (e.g. /data/videos/ppe_test.mp4), OR "0" /
    #             "/dev/video0" to read a live camera device instead.
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value="/data/videos/ppe_test.mp4",
        description='Path to the input video file, or "0"/"/dev/video0" for a live camera'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value="/opt/model/gear_guard_net_ctx.bin",
        description=(
            'Path to the PPE detection model file. '
            'Must be a precompiled QNN context binary (.bin), NOT the raw .dlc.'
        )
    )

    # publish_rate: frames PUBLISHED per second (not the video fps). Keep low
    # (1~3 Hz): NPU inference is serial, so a high rate just triggers frame drops.
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value="2.0",
        description='Frames published per second (Hz). Keep low; NPU inference is serial.'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value="true",
        description='Loop the video when it ends (continuous testing)'
    )

    frame_step_arg = DeclareLaunchArgument(
        'frame_step',
        default_value="0",
        description='Publish every Nth source frame. 0 = auto from source fps/rate.'
    )

    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value="/usr/lib/libQnnHtp.so",
        description='QNN backend library (HTP for NPU)'
    )

    # ── Detection / output arguments (forwarded to ppe_detection_node) ───
    conf_thresh_arg = DeclareLaunchArgument(
        'conf_thresh',
        default_value="0.5",
        description='Confidence threshold for detections.'
    )

    iou_thresh_arg = DeclareLaunchArgument(
        'iou_thresh',
        default_value="0.5",
        description='IoU threshold for per-class NMS.'
    )

    box_hold_frames_arg = DeclareLaunchArgument(
        'box_hold_frames',
        default_value="5",
        description='Keep last detected box for N frames to reduce flicker. 0 = off.'
    )

    # save_path: if non-empty, overwrite-save the latest annotated frame to this
    #            single image file (handy over SSH with no display).
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value="",
        description='If set, overwrite-save the latest annotated frame to this image file.'
    )

    # save_video_path: if non-empty, record all annotated frames into a video
    #                  file (use .avi + MJPG, most compatible on the board).
    save_video_path_arg = DeclareLaunchArgument(
        'save_video_path',
        default_value="",
        description='If set, record annotated frames into this video file (.avi/MJPG recommended).'
    )

    # output_fps: frame rate of the recorded demo video. Set it to your actual
    #             publish_rate so the demo timeline is correct.
    output_fps_arg = DeclareLaunchArgument(
        'output_fps',
        default_value="2.0",
        description='Frame rate of the recorded demo video (match publish_rate).'
    )

    # ── Launch Configurations ───────────────────────────────────────────
    video_path   = LaunchConfiguration('video_path')
    model_path   = LaunchConfiguration('model_path')
    publish_rate = LaunchConfiguration('publish_rate')
    loop         = LaunchConfiguration('loop')
    frame_step   = LaunchConfiguration('frame_step')
    backend      = LaunchConfiguration('backend')
    conf_thresh     = LaunchConfiguration('conf_thresh')
    iou_thresh      = LaunchConfiguration('iou_thresh')
    box_hold_frames = LaunchConfiguration('box_hold_frames')
    save_path       = LaunchConfiguration('save_path')
    save_video_path = LaunchConfiguration('save_video_path')
    output_fps      = LaunchConfiguration('output_fps')

    namespace = "ppe_detection_container"

    # ── Video Stream Publisher Node (image source, mimics a camera) ──────
    video_publisher_node = Node(
        package='sample_ppe_detection',
        executable='publish_test_video',
        name='publish_test_video',
        output='screen',
        parameters=[{
            'video_path': video_path,
            'rate':       publish_rate,
            'loop':       loop,
            'frame_step': frame_step,
            'topic':      '/image_raw',
        }]
    )

    # ── QNN Inference Node (ComposableNode) ─────────────────────────────
    nn_inference_node = ComposableNode(
        package="qrb_ros_nn_inference",
        namespace=namespace,
        plugin="qrb_ros::nn_inference::QrbRosInferenceNode",
        name="nn_inference_node",
        parameters=[{
            "backend_option": backend,
            "model_path": model_path,
            "log_level": "info",
        }]
    )

    # ── Container for ComposableNodes ───────────────────────────────────
    container = ComposableNodeContainer(
        name="ppe_container",
        namespace=namespace,
        package="rclcpp_components",
        executable='component_container',
        output="screen",
        composable_node_descriptions=[nn_inference_node],
        sigterm_timeout='3',
        sigkill_timeout='5'
    )

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
        video_path_arg,
        model_path_arg,
        publish_rate_arg,
        loop_arg,
        frame_step_arg,
        backend_arg,
        conf_thresh_arg,
        iou_thresh_arg,
        box_hold_frames_arg,
        save_path_arg,
        save_video_path_arg,
        output_fps_arg,
        LogInfo(msg=['   Starting PPE Detection with VIDEO stream']),
        LogInfo(msg=['   Video: ', video_path]),
        LogInfo(msg=['   Model: ', model_path]),
        LogInfo(msg=['   Rate : ', publish_rate, ' Hz  (loop=', loop, ')']),
        LogInfo(msg=['   Save video: ', save_video_path]),
        video_publisher_node,
        container,
        ppe_detection_node,
    ])

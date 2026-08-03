import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import LogInfo


def generate_launch_description():
    # ── Launch Arguments ────────────────────────────────────────────────
    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value="/data/images/original.jpg",
        description='Path to the input test image file'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value="/opt/model/gear_guard_net_ctx.bin",
        description=(
            'Path to the PPE detection model file. '
            'Must be a precompiled QNN context binary (.bin), '
            'NOT the raw .dlc file. Convert .dlc -> .bin using: '
            'qnn-context-binary-generator --backend <backend> '
            '--model /usr/lib/libQnnModelDlc.so --dlc_path <xxx.dlc> '
            '--binary_file <name> --output_dir <dir>'
        )
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value="1.0",
        description='Image publishing rate in Hz'
    )

    backend_arg = DeclareLaunchArgument(
        'backend',
        default_value="/usr/lib/libQnnHtp.so",
        description='QNN backend library (HTP for NPU, Cpu for CPU)'
    )

    # ── Launch Configurations ───────────────────────────────────────────
    image_path = LaunchConfiguration('image_path')
    model_path = LaunchConfiguration('model_path')
    backend = LaunchConfiguration('backend')
    publish_rate = LaunchConfiguration('publish_rate')

    namespace = "ppe_detection_container"

    # ── Image Publisher Node ────────────────────────────────────────────
    image_publisher_node = Node(
        package='image_publisher',
        executable='image_publisher_node',
        name='image_publisher_node',
        output='screen',
        parameters=[{
            'filename': image_path,
            'rate': publish_rate,
        }],
        remappings=[
            ('image_raw', '/image_raw'),
        ]
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
            "log_level": "info"  
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

    # ── PPE D
    ppe_detection_node = Node(
        package='sample_ppe_detection',
        executable='ppe_detection_node',
        name='ppe_detection_node',
        namespace=namespace,
        output='screen',
        parameters=[{
            'conf_thresh': 0.5,
            'iou_thresh': 0.5,
        }]
    )

    return LaunchDescription([
        image_path_arg,
        model_path_arg,
        backend_arg,
        publish_rate_arg,
        LogInfo(msg=['   Starting QNN Inference Test']),
        LogInfo(msg=['   Image: ', image_path]),
        LogInfo(msg=['   Model: ', model_path]),
        LogInfo(msg=['   Backend: ', backend]),
        image_publisher_node,
        container,
        ppe_detection_node,
    ])

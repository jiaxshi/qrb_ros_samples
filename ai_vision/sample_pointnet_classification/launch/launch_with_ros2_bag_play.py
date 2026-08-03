# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.logging import get_logger


def generate_launch_description():
    logger = get_logger('pointnet_mcap_launch')

    declared_args = [
        DeclareLaunchArgument(
            'model_path',
            default_value='/opt/model/libpointnet.so',
            description='Path to PointNet model file'
        ),
        DeclareLaunchArgument(
            'mcap_path',
            default_value='',
            description='Path to MCAP bag file'
        ),
        DeclareLaunchArgument(
            'classes_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('sample_pointnet_classification'), 'classes.json'
            ]),
            description='Path to class index JSON file'
        ),
        # QoS override file path — needed when bag was recorded with BEST_EFFORT
        # (common for Livox LiDAR). Leave empty to use default RELIABLE QoS.
        DeclareLaunchArgument(
            'qos_override_path',
            default_value='',
            description='Path to QoS override YAML for ros2 bag play (optional)'
        ),
    ]

    model_path = LaunchConfiguration('model_path')
    mcap_path = LaunchConfiguration('mcap_path')
    qos_override_path = LaunchConfiguration('qos_override_path')
    classes_path = LaunchConfiguration('classes_path')
    namespace = ''

    preprocess_node = Node(
        package='sample_pointnet_classification',
        executable='pointnet_classification_node',
        namespace=namespace,
        name='pointnet_classification_node',
        output='screen',
        parameters=[{'classes_path': classes_path}],
        ros_arguments=['--log-level', 'DEBUG'],
    )

    nn_inference_node = ComposableNode(
        package='qrb_ros_nn_inference',
        namespace=namespace,
        plugin='qrb_ros::nn_inference::QrbRosInferenceNode',
        name='nn_inference_node',
        parameters=[
            {
                'backend_option': '/usr/lib/libQnnHtp.so',
                'model_path': model_path,
            }
        ]
    )

    container = ComposableNodeContainer(
        name='container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=[nn_inference_node]
    )

    # Play MCAP bag file when mcap_path is provided, restricted to /livox/lidar topic.
    # Delayed 2 s to allow preprocess_node and nn_inference_node to finish initializing
    # before the first PointCloud2 message arrives, avoiding dropped frames at startup.
    # Only runs when mcap_path is non-empty; skipped for live sensor use.
    bag_play = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression(["'", mcap_path, "' != ''"])
                ),
                cmd=[
                    'ros2', 'bag', 'play', mcap_path,
                    '--topics', '/livox/lidar',
                    '--loop',
                    # Slow down playback to 1x realtime (set > 1.0 to speed up)
                    '--rate', '1.0',
                ],
                output='screen',
            ),
            # QoS override variant — used when /livox/lidar was recorded with
            # BEST_EFFORT reliability (default for Livox drivers). Activates only
            # when both mcap_path and qos_override_path are non-empty.
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression([
                        "'", mcap_path, "' != '' and '", qos_override_path, "' != ''"
                    ])
                ),
                cmd=[
                    'ros2', 'bag', 'play', mcap_path,
                    '--topics', '/livox/lidar',
                    '--rate', '1.0',
                    '--qos-profile-overrides-path', qos_override_path,
                ],
                output='screen',
            ),
        ]
    )

    return launch.LaunchDescription(
        declared_args + [preprocess_node, container, bag_play]
    )

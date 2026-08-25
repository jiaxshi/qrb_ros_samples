# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from setuptools import find_packages, setup

package_name = 'sample_pointnet_classification'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'classes.json',
        ]),
        ('lib/' + package_name, [
            package_name + '/pointnet_classification_node.py',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/launch_with_ros2_bag_play.py',
        ]),
        ('share/' + package_name + '/resource/chair_0029_rotating',
            glob('resource/chair_0029_rotating/*.mcap') +
            glob('resource/chair_0029_rotating/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kainan Li',
    maintainer_email='kainli@qti.qualcomm.com',
    description='PointNet 3D point cloud classifier using Livox LiDAR PointCloud2 input',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointnet_classification_node = '
            'sample_pointnet_classification'
            '.pointnet_classification_node:main',
        ],
    },
)

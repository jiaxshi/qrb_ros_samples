# Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sample_ppe_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'images'),
            ['resource/original.jpg']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hang Shen',
    maintainer_email='hangshen@qti.qualcomm.com',
    description=(
        'PPE detection sample using QNN NPU inference via '
        'qrb_ros_nn_inference. Bridges /image_raw to '
        'qrb_inference_input_tensor / qrb_inference_output_tensor.'
    ),
    license='BSD-3-Clause-Clear',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ppe_detection_node = '
            'sample_ppe_detection.ppe_detection_node:main',
        ],
    },
)

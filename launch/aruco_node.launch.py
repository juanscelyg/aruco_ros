# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('aruco_ros')
    param_file = os.path.join(pkg_dir, 'config', 'aruco_config.yaml')
    camera_param_file = os.path.join(pkg_dir, 'config', 'camera_calibration.yaml')

    camera_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('usb_cam'),
            'launch',
            'camera.launch.py')))
    #camera_cmd = Node(package='usb_cam',
    #                            executable='usb_cam_node_exe',
    #                            output='screen',
    #                            parameters=[camera_param_file],
    #                            )

    aruco_ros_cmd = Node(package='aruco_ros',
                                executable='aruco_program',
                                output='screen',
                                parameters=[param_file],
                                # prefix=['xterm -e gdb -ex run  --args'],
                                # prefix=['perf record --call-graph dwarf -o perf.data'],
                                arguments=[],
                                #remappings=[
                                #  ('input_pc', '/robot/front_laser/points'),
                                #  ('input_path', '/path'),
                                #])
                                )

    ld = LaunchDescription()
    ld.add_action(aruco_ros_cmd)
    ld.add_action(camera_cmd)

    return ld
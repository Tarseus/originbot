# Copyright (c) 2022，Horizon Robotics.
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

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # 设置环境变量 CAM_TYPE
    cam_type_env_var = SetEnvironmentVariable(
        name='CAM_TYPE',
        value='usb'
    )
    gesture_control_node = Node(
        package='gesture_control',
        executable='gesture_control',
        output='screen',
        parameters=[
            {"ai_msg_sub_topic_name": "/hobot_hand_gesture_detection"},
            {"twist_pub_topic_name": "/cmd_vel"},
            {"activate_wakeup_gesture": 0},
            {"track_serial_lost_num_thr": 100},
            {"move_step": 0.5},
            {"rotate_step": 0.5}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )
    #此处嵌套了相机模组相关启动文件
    hand_gesture_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hand_gesture_detection'),
                'launch/hand_gesture_detection.launch.py'))
    )

    return LaunchDescription([
        cam_type_env_var,
        hand_gesture_det_node,
        gesture_control_node
    ])
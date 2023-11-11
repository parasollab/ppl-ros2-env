# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl
#
# Description: After a robot has been loaded, this will execute a series of trajectories.
import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    initial_position = PathJoinSubstitution(
        [FindPackageShare("ur_demo"), "config", "ur_controller_prefixed.yaml"]
    )

    return LaunchDescription(
        [   
            DeclareLaunchArgument('computing_device', default_value='cuda:0', description='Device parameter'),
            DeclareLaunchArgument('yolo_image_topic', default_value='/camera/color/image_raw', description='Image topic parameter'),
            DeclareLaunchArgument('yolo_depth_topic', default_value='/camera/aligned_depth_to_color/image_raw', description='Image topic parameter'), # Changing this to /camera/camera_info causes spin error!!
            DeclareLaunchArgument('yolo_camera_info_topic', default_value='/camera/aligned_depth_to_color/camera_info', description='Image topic parameter'), # Changing this to /camera/camera_info causes spin error!!

            DeclareLaunchArgument('computing_device', default_value='cpu', description='Device parameter'),
            DeclareLaunchArgument('aruco_image_topic', default_value='/camera/color/image_raw', description='Image topic parameter'),
            DeclareLaunchArgument('aruco_depth_topic', default_value='/camera/aligned_depth_to_color/image_raw', description='Image topic parameter'), # Changing this to /camera/camera_info causes spin error!!
            DeclareLaunchArgument('aruco_camera_info_topic', default_value='/camera/aligned_depth_to_color/camera_info', description='Image topic parameter'), # Changing this to /camera/camera_info causes spin error!!



            Node(
                package='ur_demo',  # Replace with your package name
                executable='publisher_ik',  # Replace with your Python script
                name='publisher_ik',
                output='screen',
            ),

            # Node(
            #     package='ur_demo',  # Replace with your package name
            #     executable='publisher_gpg',  # Replace with your Python script
            #     name='publisher_gpg',
            #     output='screen',
            # ),

            Node(
                package="yolov5_ros2",
                executable="yolo_detect_2d",
                name="yolo_detect_2d",
                parameters=[{'computing_device': LaunchConfiguration('computing_device'), 
                            'yolo_image_topic': LaunchConfiguration('yolo_image_topic'), 
                            'yolo_depth_topic': LaunchConfiguration('yolo_depth_topic'), 
                            'yolo_camera_info_topic': LaunchConfiguration('yolo_camera_info_topic')}],
                output="screen",
            ),

            Node(
                package="yolov5_ros2",
                executable="aruco_detect",
                name="aruco_detect",
                parameters=[{'computing_device': LaunchConfiguration('computing_device'), 
                            'aruco_image_topic': LaunchConfiguration('aruco_image_topic'), 
                            'aruco_depth_topic': LaunchConfiguration('aruco_depth_topic'), 
                            'aruco_camera_info_topic': LaunchConfiguration('aruco_camera_info_topic')}],
                output="screen",
            ),
            

            Node(
                package="ur_demo",
                executable="publisher_joint_trajectory_controller",
                name="publisher_scaled_joint_trajectory_controller",
                parameters=[initial_position],
                output="screen",
            ),

            Node(
                package="ur_demo",
                executable="handoff_demo",
                name="handoff_demo",
                output="screen",
            )
        ]
    )



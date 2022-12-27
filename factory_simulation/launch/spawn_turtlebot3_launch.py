# Copyright 2019 Open Source Robotics Foundation, Inc.
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xml.etree.ElementTree as ET
import random
import string

def generate_launch_description():
    # Get the sdf file
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    robot_name = LaunchConfiguration('robot_name', default='tb1')

    # Directly edit the sdf to remap tf to namespace/tf for the diff drive
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    for plugin in root.iter('plugin'):
      if 'ros_diff_drive' in plugin.get('filename'):
        # The only plugin we care for now is 'diff_drive' which is
        # broadcasting a transform between 'odom' and 'bsae_footprint'
        break

    ros_params = plugin.find('ros')
    ros_tf_remap = ET.SubElement(ros_params,'remapping')
    ros_tf_remap.text = f'/tf:=/' + robot_name + f'/tf'

    tmp_xml = '/tmp/model_' + ''.join(random.choices(string.ascii_lowercase,k=5)) + '.sdf'
    tree.write(tmp_xml)

    remappings = [('/tf','tf'),('/tf_static','tf_static')]

    # Declare the launch arguments
    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify namespace of the robot')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='tb1',
        description='Specify namespace of the robot')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-file', tmp_xml,
            '-robot_namespace',robot_name,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen',
        remappings=remappings
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld

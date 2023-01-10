import os
import xml.etree.ElementTree as ET
import random
import string

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
  launch_file_dir = os.path.join(get_package_share_directory('factory_simulation'), 'launch')
  
  robots = [
        {'name':'robot1','x_pose':0.0, 'y_pose':0.5, 'z_pose':0.01},
        {'name':'robot2','x_pose':2.0, 'y_pose':0.5, 'z_pose':0.01}
  ]

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
  urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
  urdf_path = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'urdf',
      urdf_file_name)

  model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
  sdf_path = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'models',
      model_folder,
      'model.sdf'
  )

  with open(urdf_path, 'r') as infp:
      robot_desc = infp.read()

  remappings = [('/tf','tf'),('/tf_static','tf_static')]
  spawn_robot_cmds = []

  for robot in robots:
    #spawn_robot_cmds.append(IncludeLaunchDescription(
    #  PythonLaunchDescriptionSource(
    #    os.path.join(launch_file_dir, 'spawn_turtlebot3_launch.py')
    #  ),
    #  launch_arguments={
    #    'x_pose': TextSubstitution(text=str(robot['x_pose'])),
    #    'y_pose': TextSubstitution(text=str(robot['y_pose'])),
    #    'robot_name': TextSubstitution(text=str(robot['name']))
    #  }.items()
    #))

    #spawn_robot_cmds.append(Node(
    #  package='factory_simulation',
    #  executable='spawn_turtlebot3.py',
    #  output='screen',
    #  arguments=[
    #    '-robot_name',robot['name'],
    #    '-robot_namespace',robot['name'],
    #    '-sdf',sdf_path,
    #    '-x',str(robot['x_pose']),
    #    '-y',str(robot['y_pose'])
    #  ]
    #))

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
    ros_tf_remap.text = f'/tf:=/' + robot['name'] + f'/tf'

    tmp_xml = '/tmp/model_' + ''.join(random.choices(string.ascii_lowercase,k=5)) + '.sdf'
    tree.write(tmp_xml)

    spawn_robot_cmds.append(Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot['name'],
            '-file', tmp_xml,
            '-robot_namespace',robot['name'],
            '-x', str(robot['x_pose']),
            '-y', str(robot['y_pose']),
            '-z', '0.01'
        ],
        output='screen',
        remappings=remappings
    ))

  
    spawn_robot_cmds.append(Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      namespace=robot['name'],
      output='screen',
      parameters=[{
        'use_sim_time':use_sim_time,
        'robot_description': robot_desc
      }],
      remappings=remappings
    ))

  ld = LaunchDescription()

  # Add the commands to the launch description
  for spawn_robot_cmd in spawn_robot_cmds:
    ld.add_action(spawn_robot_cmd)

  return ld

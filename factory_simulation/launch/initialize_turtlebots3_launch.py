import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
  launch_file_dir = os.path.join(get_package_share_directory('factory_simulation'), 'launch')
  
  robots = [
        {'name':'robot1','x_pose':0.0, 'y_pose':0.5, 'z_pose':0.01}
        #{'name':'robot2','x_pose':0.0, 'y_pose':-0.5, 'z_pose':0.01}
  ]

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

  TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
  urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
  urdf_path = os.path.join(
      get_package_share_directory('turtlebot3_gazebo'),
      'urdf',
      urdf_file_name)

  with open(urdf_path, 'r') as infp:
      robot_desc = infp.read()

  remappings = [('/tf','tf'),('/tf_static','tf_static')]
  spawn_robot_cmds = []

  for robot in robots:
    spawn_robot_cmds.append(IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
        os.path.join(launch_file_dir, 'spawn_turtlebot3_launch.py')
      ),
      launch_arguments={
        'x_pose': TextSubstitution(text=str(robot['x_pose'])),
        'y_pose': TextSubstitution(text=str(robot['y_pose']))
      }.items()
    ))

  
    spawn_robot_cmds.append(Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      #namespace=robot['name'],
      output='screen',
      parameters=[{
        'use_sim_time':use_sim_time,
        'robot_description': robot_desc
      }],
      remappings=remappings,
    ))

  ld = LaunchDescription()

  # Add the commands to the launch description
  for spawn_robot_cmd in spawn_robot_cmds:
    ld.add_action(spawn_robot_cmd)

  return ld

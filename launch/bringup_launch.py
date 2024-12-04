import os
from launch import LaunchDescription
from launch.substitutions import Command,  LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'delivery_bot'
  pkg_share = get_package_share_directory(package_name)

  xacro_file_path = os.path.join(pkg_share, 'description', 'delivery_bot.urdf.xacro')
  world_file = os.path.join(pkg_share, "worlds", "restaurant.world")
  robot_description = Command(['xacro ', xacro_file_path])

  rviz_launch = LaunchConfiguration('rviz_launch', default='true') 
  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', os.path.join(pkg_share, 'rviz', 'bringup.rviz')],
    output='screen',
    parameters=[{'use_sim_time': False}],
    condition=IfCondition(rviz_launch),
  )

  robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
  )

  joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='both',
    parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
  )

  joint_state_publisher_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    output='both',
    parameters=[{'robot_description': robot_description}, {'use_sim_time': 'true'}],
  )
  
  gazebo = ExecuteProcess(
    cmd=['gazebo', '--verbose', world_file,  '-s', 'libgazebo_ros_factory.so'] 
  )

  spawn_entity = Node(
    package= 'gazebo_ros',
    executable= 'spawn_entity.py',
    name = 'urdf_spawner',
    arguments = ['-topic', '/robot_description', '-entity', 'round_bot', '-x', '6.106026', '-y', '-2.914490', '-z', '0.10000',
    '-R', '0.0', '-P', '0.0', '-Y', '3.140074'],
    parameters=[{'use_sim_time': True}],
  )

  return LaunchDescription([
    rviz_node,
    robot_state_publisher,
    joint_state_publisher,
    #joint_state_publisher_gui,
    gazebo,
    spawn_entity
  ])
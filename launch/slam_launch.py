import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'delivery_bot'
  pkg_share = get_package_share_directory(package_name)

  controller_params_file = os.path.join(pkg_share,'params','slam.yaml')
  rviz_file = os.path.join(pkg_share, 'rviz', 'slam.rviz')
  bring_up = IncludeLaunchDescription(
              PythonLaunchDescriptionSource([os.path.join(
                  get_package_share_directory(package_name),'launch','bringup_launch.py'
              )]),
              launch_arguments={'rviz_launch': 'false'}.items() 
  )
  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_file],
    output='screen',
    parameters=[{'use_sim_time': False}]
  )

  slam = Node(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    output='screen',
    arguments=['-d', controller_params_file],
    parameters=[{'use_sim_time': True}]
  )
  
  return LaunchDescription([
    bring_up,
    rviz_node,
    slam
  ])




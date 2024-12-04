import os
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration,  TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  package_name = 'delivery_bot'
  pkg_share = get_package_share_directory(package_name)
  
  localize_lifecycle_nodes = [
    'map_server', 
    'amcl'
  ]
  nav2_lifecyle_nodes = [
    'controller_server', 
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'bt_navigator',
    'waypoint_follower',
  ]
  remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

  localization_params_file = os.path.join(pkg_share,'params','amcl.yaml')
  nav2_params_file = os.path.join(pkg_share,'params','nav2_params.yaml')
  rviz_file = os.path.join(pkg_share, 'rviz', 'navigation.rviz')
  map_yaml_file = os.path.join(pkg_share, 'maps', 'restaurant.yaml') 
  #the map file path can be given in the yaml file, if the map file path mentioned there, 
  # no need to give in the nav2_map_server node

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

  nav2_map_server = Node(
    package='nav2_map_server',
    executable='map_server',
    name='map_server',
    respawn=False,
    output='screen',
    parameters=[localization_params_file, {'yaml_filename': map_yaml_file}, {'use_sim_true': True}],
    remappings=remappings,
  )
  nav2_amcl = Node(
    package='nav2_amcl',
    executable='amcl',
    name='amcl',
    respawn=False,
    output='screen',
    parameters=[localization_params_file, {'use_sim_true': True}],
    remappings=remappings,
  )
  localize_lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_localization',
    output='screen',
    parameters=[{'autostart': True}, {'node_names': localize_lifecycle_nodes}, {'use_sim_true': True}],
  )

  nav2_controller = Node(
    package='nav2_controller',
    executable='controller_server',
    output='screen',
    parameters=[nav2_params_file],
  )
  nav2_smoother = Node(
    package='nav2_smoother',
    executable='smoother_server',
    name='smoother_server',
    output='screen',
    parameters=[nav2_params_file],
    remappings=remappings,
  )
  nav2_planner = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    parameters=[nav2_params_file],
    remappings=remappings,
  )
  nav2_behaviors = Node(
    package='nav2_behaviors',
    executable='behavior_server',
    name='behavior_server',
    output='screen',
    parameters=[nav2_params_file],
    remappings=remappings,
  )
  nav2_bt_navigator = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[nav2_params_file],
    remappings=remappings,
  )
  nav2_waypoint_follower = Node(
    package='nav2_waypoint_follower',
    executable='waypoint_follower',
    name='waypoint_follower',
    output='screen',
    parameters=[nav2_params_file],
    remappings=remappings,
  )
  nav2_velocity_smoother = Node(
    package='nav2_velocity_smoother',
    executable='velocity_smoother',
    name='velocity_smoother',
    output='screen',
    parameters=[nav2_params_file],
    remappings=remappings,
  )
  nav2_lifecycle_nodes_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    output='screen',
    parameters=[{'autostart': True}, {'node_names': nav2_lifecyle_nodes}, {'use_sim_true': False}],
  )
  
  return LaunchDescription([
    rviz_node,
    bring_up,
    nav2_map_server,
    nav2_amcl,
    localize_lifecycle_manager,
    nav2_controller,
    nav2_smoother,
    nav2_planner,
    nav2_behaviors,
    nav2_bt_navigator,
    nav2_waypoint_follower,
    nav2_velocity_smoother,
    nav2_lifecycle_nodes_manager
  ])




import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_map_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Declare launch arguments
    map_name = LaunchConfiguration('map_name')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

    # Construct the full path to the map file
    map_file = PythonExpression(["'", os.path.join(pkg_share, 'maps'), "/',", map_name, ", '.yaml'"])
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # Kill the existing nav2 and rviz nodes
        ExecuteProcess(cmd=['pkill', '-f', 'nav2_bringup'], output='screen'),
        ExecuteProcess(cmd=['pkill', '-f', 'rviz2'], output='screen'),

        # Add a small delay to ensure processes are killed before restarting
        TimerAction(period=3.0, actions=[
            # Relaunch Nav2 with the new map and initial pose
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'True',
                    'params_file': params_file,
                    'autostart': 'True',
                    'initial_pose.x': initial_pose_x,
                    'initial_pose.y': initial_pose_y,
                    'initial_pose.yaw': initial_pose_yaw,
                }.items()
            ),

            # Relaunch RViz
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
            )
        ])
    ])
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_map_navigation')
    db_path = os.path.join(pkg_share, 'db', 'wormholes.db')
    initial_map = 'map1'
    map_file = os.path.join(pkg_share, 'maps', initial_map + '.yaml')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    # Start the multi-map navigator node
    navigator_node = Node(
        package='multi_map_navigation',
        executable='multi_map_navigator_node',
        name='multi_map_navigator',
        output='screen',
        parameters=[{'db_path': db_path, 'initial_map': initial_map}]
    )

    # Start Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'cartographer_world.launch.py')),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'map1.world')}.items()
    )

    # Start the initial Nav2 stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': params_file,
            'autostart': 'True'
        }.items()
    )

    # Start RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')),
    )

    return LaunchDescription([
        navigator_node,
        gazebo_launch,
        nav2_launch,
        rviz_launch
    ])
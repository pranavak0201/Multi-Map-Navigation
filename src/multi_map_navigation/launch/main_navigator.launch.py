# In launch/main_navigator.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('multi_map_navigation')
    db_path = os.path.join(pkg_share, 'db', 'wormholes.db')

    # Start the multi-map navigator node
    navigator_node = Node(
        package='multi_map_navigation',
        executable='multi_map_navigator_node',
        name='multi_map_navigator',
        output='screen',
        parameters=[{'db_path': db_path, 'initial_map': 'map3'}] # Use map3 as the start
    )

    # Start Gazebo simulation with your new world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'cartographer_world.launch.py')),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'map3.world')}.items()
    )

    return LaunchDescription([
        gazebo_launch,
        navigator_node
    ])
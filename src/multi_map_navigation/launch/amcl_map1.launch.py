from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Map server
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": "/home/bala/multi_map_ws/src/multi_map_navigation/maps/map1.yaml"}]
        ),

        # AMCL
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),

        # Lifecycle manager
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{"use_sim_time": True},
                        {"autostart": True},
                        {"node_names": ["map_server", "amcl"]}]
        ),

        # RViz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", "/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz"]
        )
    ])

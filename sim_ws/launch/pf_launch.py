from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> None:
    localize_config = LaunchConfiguration("localize_config")
    localize_config_launch_arg = DeclareLaunchArgument(
        "localize_config", default_value="config/sim_localize.yaml"
    )
    # From: https://github.com/f1tenth/particle_filter/blob/foxy-devel/launch/localize_launch.py
    pf_node = Node(
        package="particle_filter",
        executable="particle_filter",
        name="particle_filter",
        parameters=[localize_config],
        remappings=[("/tf", "/null")],  # map TF data to unused topic
    )
    return LaunchDescription([localize_config_launch_arg, pf_node])

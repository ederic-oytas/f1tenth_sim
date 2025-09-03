# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> None:
    map_path = LaunchConfiguration("map_path")
    sim_config = LaunchConfiguration("sim_config")
    rviz_config = LaunchConfiguration("rviz_config")
    map_img_ext = LaunchConfiguration("map_img_ext")

    map_path_launch_arg = DeclareLaunchArgument(
        "map_path",
        default_value="maps/levine",
    )
    sim_config_launch_arg = DeclareLaunchArgument(
        "sim_config",
        default_value="config/sim.yaml",
    )
    rviz_config_launch_arg = DeclareLaunchArgument(
        "rviz_config", default_value="launch/gym_bridge.rviz"
    )
    map_img_ext_launch_arg = DeclareLaunchArgument("map_img_ext", default_value=".png")

    bridge_node = Node(
        package="f1tenth_gym_ros",
        executable="gym_bridge",
        name="bridge",
        parameters=[{"map_path": map_path}, {"map_img_ext": map_img_ext}, sim_config],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config],
    )
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            {"yaml_filename": [map_path, ".yaml"]},
            {"topic": "map"},
            {"frame_id": "map"},
            {"output": "screen"},
            {"use_sim_time": True},
        ],
    )
    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )
    ego_robot_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ego_robot_state_publisher",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro ",
                        os.path.join(
                            get_package_share_directory("f1tenth_gym_ros"),
                            "launch",
                            "ego_racecar.xacro",
                        ),
                    ]
                )
            }
        ],
        remappings=[("/robot_description", "ego_robot_description")],
    )

    return LaunchDescription(
        [
            map_path_launch_arg,
            sim_config_launch_arg,
            rviz_config_launch_arg,
            map_img_ext_launch_arg,
            rviz_node,
            bridge_node,
            nav_lifecycle_node,
            map_server_node,
            ego_robot_publisher,
        ]
    )

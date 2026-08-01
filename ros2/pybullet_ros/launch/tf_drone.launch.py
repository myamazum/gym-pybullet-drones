"""Launch the tf-enabled simulator bridge and robot state publisher."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_robot_description() -> str:
    """Render the installed xacro model without writing into the share tree."""

    share_dir = get_package_share_directory("pybullet_ros")
    xacro_path = os.path.join(share_dir, "urdf", "mas.urdf.xacro")
    return xacro.process_file(xacro_path).toprettyxml(indent="  ")


def generate_launch_description() -> LaunchDescription:
    """Create the pybullet ROS bridge launch description."""

    gui = LaunchConfiguration("gui")
    plot = LaunchConfiguration("plot")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": generate_robot_description()}],
    )
    drone = Node(
        package="pybullet_ros",
        executable="drone_tf",
        output="both",
        parameters=[
            {
                "gui": ParameterValue(gui, value_type=bool),
                "plot": ParameterValue(plot, value_type=bool),
            }
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("plot", default_value="false"),
        robot_state_publisher,
        drone,
    ])

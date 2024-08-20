import os
import time

import xacro
from ament_index_python.packages import get_package_share_directory

import launch_ros.actions

import launch
from launch.actions import DeclareLaunchArgument, ExecuteProcess,TimerAction,RegisterEventHandler
from launch.event_handlers import OnProcessStart,OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

share_dir_path = os.path.join(get_package_share_directory('pybullet_ros'))
xacro_path = os.path.join(share_dir_path, 'urdf', 'mas.urdf.xacro')
urdf_path = os.path.join(share_dir_path, 'urdf', 'mas.urdf')

def generate_robot_description():
    doc = xacro.process_file(xacro_path)
    robot_description = doc.toprettyxml(indent='  ')
    f = open(urdf_path, 'w')
    f.write(robot_description)
    f.close()
    return robot_description

def generate_launch_description():
    ld = launch.LaunchDescription()
    enable_dummy = LaunchConfiguration('enable_dummy', default=False)
    enable_gui = LaunchConfiguration('gui', default=False)
    params = {'robot_description': generate_robot_description()}

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  arguments=[urdf_path],
                                  condition=UnlessCondition(enable_dummy),
                                  parameters=[params])
    drn = launch_ros.actions.Node(package='pybullet_ros',
                                  executable='drone_tf',
                                  output='both',
                                  arguments=[],
                                  condition=UnlessCondition(enable_dummy),
                                  parameters=[])

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    ld.add_action(rsp)
    ld.add_action(drn)

    return ld
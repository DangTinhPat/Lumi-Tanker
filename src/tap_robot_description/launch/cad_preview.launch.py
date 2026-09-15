"""Display the user's seven CAD components with six adjustable joints."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('tap_robot_description'))
    description = xacro.process_file(
        str(share / 'description/cad_robot/robot.urdf.xacro')).toxml()
    gui = LaunchConfiguration('gui')
    parameters = [{'robot_description': description, 'use_sim_time': False}]
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='cad_preview', parameters=parameters, output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             namespace='cad_preview', parameters=parameters, condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace='cad_preview', parameters=parameters, condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2', namespace='cad_preview',
             arguments=['-d', str(share / 'config/cad_preview.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])

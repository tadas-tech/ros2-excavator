"""Show the excavator in RViz with a GUI to drive its joints.

Arguments:
  gui:=slider  (default) the excavator_gui slider window
  gui:=jsp     joint_state_publisher_gui
  gui:=none    no GUI, joint states must come from elsewhere
  rviz:=true|false
  rviz_hidpi_fix:=true|false
               (default true) disable Qt HiDPI scaling for RViz, which
               stops the RViz view glitching on this machine
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    AndSubstitution, Command, EqualsSubstitution, LaunchConfiguration,
    NotSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Same fix as the rviz2 alias in ~/.bashrc: RViz glitches with Qt HiDPI scaling.
RVIZ_HIDPI_FIX_ENV = {
    'QT_SCREEN_SCALE_FACTORS': '1',
    'QT_ENABLE_HIGHDPI_SCALING': '0',
}


def generate_launch_description():
    share_dir = get_package_share_directory('excavator_description')
    xacro_path = os.path.join(share_dir, 'urdf', 'excavator.urdf.xacro')
    rviz_path = os.path.join(share_dir, 'rviz', 'excavator.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str)

    use_rviz = LaunchConfiguration('rviz')
    hidpi_fix = LaunchConfiguration('rviz_hidpi_fix')

    def rviz_node(condition, env=None):
        return Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_path],
            additional_env=env,
            condition=IfCondition(condition),
            output='screen',
        )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='slider',
                              choices=['slider', 'jsp', 'none'],
                              description='Joint control GUI to start'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Start RViz'),
        DeclareLaunchArgument('rviz_hidpi_fix', default_value='true',
                              description='Disable Qt HiDPI scaling for RViz'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            # kdl_parser warns about every floating (hydraulic cylinder) joint;
            # robot_state_publisher skips them and excavator_hydraulics publishes their TF.
            arguments=['--ros-args', '--log-level', 'kdl_parser:=error'],
            output='screen',
        ),
        Node(
            package='excavator_hydraulics',
            executable='cylinder_tf',
            output='screen',
        ),
        Node(
            package='excavator_gui',
            executable='slider_gui',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('gui'), 'slider')),
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(EqualsSubstitution(LaunchConfiguration('gui'), 'jsp')),
            output='screen',
        ),
        rviz_node(AndSubstitution(use_rviz, hidpi_fix), RVIZ_HIDPI_FIX_ENV),
        rviz_node(AndSubstitution(use_rviz, NotSubstitution(hidpi_fix))),
    ])

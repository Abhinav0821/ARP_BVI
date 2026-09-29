"""Start the BVI node and RViz2 with the saved configuration.

ros2 launch bvi_nav bvi_demo.launch.py
ros2 launch bvi_nav bvi_demo.launch.py seed:=7 step_rate:=6.0 show_labels:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('bvi_nav')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'bvi.rviz')

    args = [
        DeclareLaunchArgument('seed', default_value='42',
                              description='RNG seed for obstacle placement'),
        DeclareLaunchArgument('grid_size', default_value='30'),
        DeclareLaunchArgument('obstacle_ratio', default_value='0.20'),
        DeclareLaunchArgument('start', default_value='[-1, -1]',
                              description='[x, y]; [-1, -1] picks automatically'),
        DeclareLaunchArgument('goal', default_value='[-1, -1]',
                              description='[x, y]; [-1, -1] picks automatically'),
        DeclareLaunchArgument('step_rate', default_value='4.0',
                              description='Animation steps per second'),
        DeclareLaunchArgument('loop', default_value='true'),
        DeclareLaunchArgument('show_labels', default_value='true'),
        DeclareLaunchArgument('label_stride', default_value='1'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Set to false to start the node only'),
    ]

    bvi_node = Node(
        package='bvi_nav',
        executable='bvi_node',
        name='bvi_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'seed': LaunchConfiguration('seed'),
            'grid_size': LaunchConfiguration('grid_size'),
            'obstacle_ratio': LaunchConfiguration('obstacle_ratio'),
            'start': LaunchConfiguration('start'),
            'goal': LaunchConfiguration('goal'),
            'step_rate': LaunchConfiguration('step_rate'),
            'loop': LaunchConfiguration('loop'),
            'show_labels': LaunchConfiguration('show_labels'),
            'label_stride': LaunchConfiguration('label_stride'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [bvi_node, rviz_node])

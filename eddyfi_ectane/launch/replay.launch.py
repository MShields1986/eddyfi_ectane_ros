"""Replays a recording in RViz, on the recording's clock."""

# TODO: Consider recording the pointcloud at the array head and doing the reconstruction after the face here

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('eddyfi_ectane')

    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='ectane_recording_01',
                              description='Bag to replay, relative to where the launch is run'),

        # ROSbag Play
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', LaunchConfiguration('bag'), '--loop', '--clock'],
            name='bag_player',
            output='screen',
        ),

        # Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', os.path.join(share, 'config', 'recon.rviz')],
            parameters=[{'use_sim_time': True}],
        ),
    ])

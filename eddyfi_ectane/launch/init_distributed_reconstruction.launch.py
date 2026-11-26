"""Robot side of a distributed reconstruction, the same as init.launch.py and taking the same arguments.

Run distributed_reconstruction.launch.py on the reconstruction machine with the same configuration.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('eddyfi_ectane'), 'launch', 'init.launch.py'))),
    ])

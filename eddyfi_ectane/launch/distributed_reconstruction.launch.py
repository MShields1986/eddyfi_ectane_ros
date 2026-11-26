"""Reconstruction of the clouds at the probe in the fixture frame, which can run on a separate machine.

The configuration must match the one given to init.launch.py, as the reconstruction reads its crosstalk
configuration to find how many frequencies to reconstruct.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context):
    configuration = LaunchConfiguration('configuration').perform(context)

    share = get_package_share_directory('eddyfi_ectane')
    settings = os.path.join(share, 'config',
                            'simulated_ectane_mode.yaml' if configuration == 'example' else 'settings.yaml')
    crosstalk = os.path.join(share, 'config', 'probes', f'crosstalk_{configuration}.yaml')
    if not os.path.isfile(crosstalk):
        raise RuntimeError(f"Unknown configuration '{configuration}', no {crosstalk}")

    container = ComposableNodeContainer(
        name='ectane_distributed_reconstruction_container',
        namespace='ectane',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='eddyfi_ectane',
                plugin='reconstruction_namespace::ReconstructionNode',
                name='distributed_reconstruction',
                namespace='ectane',
                parameters=[settings, crosstalk],
            ),
        ],
        output='screen',
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('configuration', default_value='lf_probe_axial_and_transversal_1_channel',
                              description='Probe configuration, the same as given to init.launch.py'),
        OpaqueFunction(function=launch_setup),
    ])

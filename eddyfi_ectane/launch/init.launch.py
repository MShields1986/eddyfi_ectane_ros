"""Eddyfi Ectane driver, with the eddy current data converted to clouds at the probe.

##########################################
#         Configuration Options          #
##########################################
example                                   - Intended for testing with simulated Ectane only
hf_probe_axial_1_channel
hf_probe_transversal_1_channel
hf_probe_axial_and_transversal_1_channel
hf_probe_axial_and_transversal_4_channels - Multi-frequency, publishes per frequency topics under /ectane/frequency_<n>/
lf_probe_axial_1_channel
lf_probe_transversal_1_channel
lf_probe_axial_and_transversal_1_channel
lf_probe_axial_and_transversal_4_channels - Multi-frequency, publishes per frequency topics under /ectane/frequency_<n>/
##########################################
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


# Composed nodes in the same container pass messages without copying them
INTRA_PROCESS = [{'use_intra_process_comms': True}]

# Coil geometry configurations for each configuration
GEOMETRY = {
    'example': ['example_geometry.yaml'],
    'hf_probe_axial_1_channel': ['hf_axial_geometry.yaml'],
    'hf_probe_transversal_1_channel': ['hf_transversal_geometry.yaml'],
    'hf_probe_axial_and_transversal_1_channel': ['hf_axial_geometry.yaml', 'hf_transversal_geometry.yaml'],
    'hf_probe_axial_and_transversal_4_channels': ['hf_axial_geometry.yaml', 'hf_transversal_geometry.yaml'],
    'lf_probe_axial_1_channel': ['lf_axial_geometry.yaml'],
    'lf_probe_transversal_1_channel': ['lf_transversal_geometry.yaml'],
    'lf_probe_axial_and_transversal_1_channel': ['lf_axial_geometry.yaml', 'lf_transversal_geometry.yaml'],
    'lf_probe_axial_and_transversal_4_channels': ['lf_axial_geometry.yaml', 'lf_transversal_geometry.yaml'],
}


def settings_file(share, configuration):
    """Basic settings, or the simulated Ectane's for the example configuration."""
    name = 'simulated_ectane_mode.yaml' if configuration == 'example' else 'settings.yaml'
    return os.path.join(share, 'config', name)


def crosstalk_file(share, configuration):
    """Crosstalk rectification configuration, mapping each channel to its time slot and input."""
    return os.path.join(share, 'config', 'probes', f'crosstalk_{configuration}.yaml')


def check_configuration(configuration):
    if configuration not in GEOMETRY:
        raise RuntimeError(f"Unknown configuration '{configuration}', options are: {', '.join(GEOMETRY)}")


def launch_setup(context):
    configuration = LaunchConfiguration('configuration').perform(context)
    check_configuration(configuration)
    no_robot_mode = LaunchConfiguration('no_robot_mode').perform(context).lower() in ('true', '1')

    share = get_package_share_directory('eddyfi_ectane')
    settings = settings_file(share, configuration)
    crosstalk = crosstalk_file(share, configuration)
    geometry = [os.path.join(share, 'config', 'probes', name) for name in GEOMETRY[configuration]]
    launch_settings = {'configuration': configuration, 'no_robot': no_robot_mode}
    reconstruction = LaunchConfiguration('reconstruction').perform(context).lower() in ('true', '1')

    composable_nodes = [
        # EC Data to Pointcloud at Array Head, loaded first so it is subscribed before data streams
        ComposableNode(
            package='eddyfi_ectane',
            plugin='ec_data_to_pointcloud_namespace::EcDataToPointCloudNode',
            name='ec_data_to_pointcloud',
            namespace='ectane',
            parameters=[settings, crosstalk, launch_settings],
            extra_arguments=INTRA_PROCESS,
        ),
    ]

    # Reconstruction, which otherwise runs separately with distributed_reconstruction.launch.py
    if reconstruction:
        composable_nodes.append(ComposableNode(
            package='eddyfi_ectane',
            plugin='reconstruction_namespace::ReconstructionNode',
            name='reconstruction',
            namespace='ectane',
            parameters=[settings, crosstalk],
            extra_arguments=INTRA_PROCESS,
        ))

    # Eddyfi Ectane, which waits for the C SDK to connect while loading
    composable_nodes.append(ComposableNode(
        package='eddyfi_ectane',
        plugin='ectane_namespace::EctaneNode',
        name='ectane_handler',
        namespace='ectane',
        parameters=[settings, crosstalk, *geometry, launch_settings],
        extra_arguments=INTRA_PROCESS,
    ))

    container = ComposableNodeContainer(
        name='ectane_container',
        namespace='ectane',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    # Recording
    topics = LaunchConfiguration('rosbag_topics').perform(context).strip('[]').replace(',', ' ').split()
    rosbag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--output', LaunchConfiguration('rosbag_name'), *topics],
        condition=IfCondition(LaunchConfiguration('logging')),
        output='screen',
    )

    # Rviz
    rviz_config = 'config_multi_frequency.rviz' if configuration.endswith('_4_channels') else 'config.rviz'
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(share, 'rviz', rviz_config)],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [container, rosbag, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('configuration', default_value='lf_probe_axial_and_transversal_1_channel',
                              description='Probe configuration, see the options at the top of this file'),
        DeclareLaunchArgument('no_robot_mode', default_value='false',
                              description='Broadcast a dummy transform for the probe rather than use a robot'),
        DeclareLaunchArgument('reconstruction', default_value='false',
                              description='Run the reconstruction in the same container, rather than with '
                                          'distributed_reconstruction.launch.py'),
        DeclareLaunchArgument('rviz', default_value=LaunchConfiguration('no_robot_mode'),
                              description='Open RViz, by default only in no robot mode'),
        DeclareLaunchArgument('logging', default_value='false', description='Record a bag'),
        DeclareLaunchArgument('rosbag_name', default_value='ectane_recording',
                              description='Bag directory, relative to where the launch is run'),
        DeclareLaunchArgument('rosbag_topics',
                              default_value='[/tf,/ectane/eddy_current_data,/ectane/axial_reconstruction,'
                                            '/ectane/transversal_reconstruction]',
                              description='Topics to record'),
        OpaqueFunction(function=launch_setup),
    ])

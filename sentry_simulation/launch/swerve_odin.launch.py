"""Independent four-swerve / four O1LITE Gazebo Harmonic platform."""
import os
from pathlib import Path
from sentry_simulation.gazebo_compat import gazebo_arguments, prepare_share, resource_paths
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from sentry_simulation.swerve_platform import build_assets


def _launch(context, share, worlds):
    resource_key, resource_value = resource_paths(share)
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    sensors = LaunchConfiguration('sensors').perform(context).lower() == 'true'
    publish_ground_truth_tf = LaunchConfiguration('publish_ground_truth_tf').perform(context).lower() == 'true'
    world_name = LaunchConfiguration('world').perform(context)
    world = worlds[world_name] if world_name != 'swerve_odin' else None
    directory = tempfile.TemporaryDirectory(prefix='swerve_odin_')
    paths = build_assets(config, share, directory.name, sensors=sensors,
                         world_path=share / world['world'] if world else None,
                         spawn=world['spawn'] if world else None,
                         publish_ground_truth_tf=publish_ground_truth_tf)
    actions = [SetEnvironmentVariable(resource_key, resource_value)]
    if (not context.environment.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            and context.environment.get('ROS_LOCALHOST_ONLY') != '1'):
        actions.append(SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                                              str(share / 'config/fastdds_shm.xml')))
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    actions.extend([
        # Keep generated files alive for the entire launch, then remove them.
        RegisterEventHandler(OnShutdown(on_shutdown=[
            OpaqueFunction(function=lambda _: directory.cleanup() or [])])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory('ros_gz_sim')) / 'launch/gz_sim.launch.py')),
            launch_arguments={'on_exit_shutdown': 'true',
                              **gazebo_arguments('-r ' + ('-s ' if headless else '') + str(paths['world']))}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='swerve_odin_bridge',
             parameters=[{'config_file': str(paths['bridge']), 'use_sim_time': True}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='swerve_state_publisher',
             parameters=[{'robot_description': paths['urdf'].read_text(), 'use_sim_time': True,
                          'publish_frequency': config['control']['publish_rate']}],
             remappings=[('joint_states', '/swerve/joint_states')], output='screen'),
    ])
    if LaunchConfiguration('rviz').perform(context).lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', name='swerve_rviz',
                            arguments=['-d', str(share / 'config/swerve_odin.rviz')],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    share = prepare_share(os.environ.get(
        'SENTRY_SIMULATION_SHARE', get_package_share_directory('sentry_simulation')))
    worlds = yaml.safe_load((share / 'config/worlds.yaml').read_text())
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/swerve_odin.yaml')),
        DeclareLaunchArgument('world', default_value='swerve_odin', choices=['swerve_odin', *worlds],
                              description='Independent platform world or a shared worlds.yaml field'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('sensors', default_value='true', choices=['true', 'false'],
                              description='Disable only for isolated chassis dynamics checks'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('publish_ground_truth_tf', default_value='true', choices=['true', 'false'],
                              description='Disable when localization owns the base_link transform'),
        OpaqueFunction(function=_launch, args=[share, worlds]),
    ])

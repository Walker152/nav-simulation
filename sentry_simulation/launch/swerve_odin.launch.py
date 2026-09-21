"""Independent four-swerve / four O1LITE Gazebo Fortress platform."""
from pathlib import Path
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


def _launch(context):
    share = Path(get_package_share_directory('sentry_simulation'))
    config = yaml.safe_load(Path(LaunchConfiguration('config').perform(context)).read_text())
    sensors = LaunchConfiguration('sensors').perform(context).lower() == 'true'
    directory = tempfile.TemporaryDirectory(prefix='swerve_odin_')
    paths = build_assets(config, share, directory.name, sensors=sensors)
    actions = []
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
            launch_arguments={'gz_version': '6', 'on_exit_shutdown': 'true',
                              'gz_args': '-r ' + ('-s ' if headless else '') + str(paths['world'])}.items()),
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
    share = Path(get_package_share_directory('sentry_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=str(share / 'config/swerve_odin.yaml')),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('sensors', default_value='true', choices=['true', 'false'],
                              description='Disable only for isolated chassis dynamics checks'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_launch),
    ])

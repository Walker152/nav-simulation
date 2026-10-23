"""Swerve dual MID360 simulation through the real fixed-map localization chain."""
import importlib.util
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node, RosTimer, SetUseSimTime
from launch_ros.descriptions import ComposableNode
from sentry_simulation.swerve_platform import mid360_pose


def _launch(context):
    value = lambda key: context.launch_configurations[key]
    share = Path(os.environ.get('SENTRY_SIMULATION_SHARE', get_package_share_directory('sentry_simulation')))
    worlds = yaml.safe_load((share / 'config/worlds.yaml').read_text())
    world = worlds[value('world')]
    prior_map = value('prior_map').strip()
    if not prior_map:
        default_prior = world.get('prior_map')
        icp = world.get('icp', {})
        # A zero PCD-to-map transform also makes the existing ICP asset a
        # compatible backend prior; nonzero transforms still need an override.
        if not default_prior and icp.get('pcd_to_map') == [0.0, 0.0, 0.0]:
            default_prior = icp.get('target_pcd')
        if not default_prior:
            raise ValueError('This world needs an explicit prior_map PCD in navigation map coordinates')
        prior_map = str(share / default_prior)
    files = {key: value(key) for key in
             ('params_file', 'mid360_config', 'pointlio_config', 'backend_config', 'platform_config')}
    files.update(prior_map=prior_map, map=value('map').strip() or str(share / world['map']))
    for key, filename in files.items():
        path = Path(filename).expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f'{key} does not exist: {path}')
        files[key] = str(path)
    for key in ('headless', 'rviz', 'autostart'):
        if value(key) not in ('true', 'false'):
            raise ValueError(f'{key} must be true or false')
    fusion = Path(get_package_share_directory('dual_mid360_fusion'))
    navigation = Path(get_package_share_directory('navi2'))
    spec = importlib.util.spec_from_file_location('mid360_sim_calibration', fusion / 'launch/calibration_common.py')
    calibration = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(calibration)
    cfg, *_ = calibration.load_calibration(files['params_file'], files['mid360_config'])
    vehicle = yaml.safe_load(Path(files['params_file']).read_text())
    if (value('model').strip() or vehicle['planner']['model']) != 'omni' or cfg['frames']['base'] != 'base_link':
        raise ValueError('Swerve simulation requires planner.model=omni and frames.base=base_link')
    sensors = yaml.safe_load(Path(files['mid360_config']).read_text())

    frequency = cfg['driver']['ros_parameters']['publish_freq']
    topics, frames = cfg['topics'], cfg['frames']
    inputs = {f'{side}_{hemisphere}_input_topic': f'/sim/mid360/{sensor}/{hemisphere}/points'
              for side, sensor in (('left', 'primary'), ('right', 'secondary'))
              for hemisphere in ('front', 'rear')}
    adapter = ComposableNodeContainer(
        package='rclcpp_components', executable='component_container',
        name='mid360_sim_input_container', namespace='', output='screen',
        composable_node_descriptions=[ComposableNode(
            package='sentry_simulation', plugin='sentry_simulation::PointCloudAdapter', name='mid360_sim_adapter',
            parameters=[{
                'use_sim_time': True, **inputs,
                'output_topic': topics['primary_lidar'], 'output_frame_id': frames['primary_lidar'],
                'secondary_output_topic': topics['secondary_lidar'],
                'secondary_output_frame_id': frames['secondary_lidar'],
                'left_sensor_pose': list(mid360_pose(sensors, 'primary_in_base')),
                'right_sensor_pose': list(mid360_pose(sensors, 'secondary_in_base')),
                'lidar_pose_in_model': [0.0] * 6,
                'ground_truth_topic': '/swerve/ground_truth/odometry',
                'pattern_file': str(share / 'resource/models/mid360/scan_mode/mid360-real-centr.csv'),
                'pattern_points_per_frame': round(200000 / frequency), 'scan_period': 1.0 / frequency,
                'sync_tolerance_ms': min(5.0, 1000.0 / frequency),
            }], extra_arguments=[{'use_intra_process_comms': True}],
        )], on_exit=lambda event, context: [] if context.is_shutdown else [
            Shutdown(reason='MID360 simulation input container exited')])
    return [
        SetUseSimTime(True),
        LogInfo(msg=f'Dual MID360 -> fusion -> Point-LIO -> backend global localization -> navigation; prior={files["prior_map"]}'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/swerve_odin.launch.py')),
            launch_arguments={
                'config': files['platform_config'], 'world': value('world'),
                'headless': value('headless'), 'rviz': 'false', 'sensors': 'true',
                'sensor_suite': 'mid360', 'mid360_config': files['mid360_config'],
                'publish_ground_truth_tf': 'false', 'ground_truth_frame': 'world',
            }.items()),
        adapter,
        # Gazebo is SI; emulate Livox raw g so the canonical hardware conversion
        # remains the only SI input used by Point-LIO.
        Node(package='dual_mid360_fusion', executable='imu_si_node', name='mid360_sim_raw_imu',
             output='screen', parameters=[{
                 'use_sim_time': True, 'input_topic': '/sim/mid360/primary/imu',
                 'output_topic': topics['primary_imu'], 'frame_id': frames['primary_imu'],
                 'accel_scale': 1.0 / 9.81, 'reliable_transport': True,
             }]),
        RosTimer(period=1.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(navigation / 'launch/mid360_navigation.launch.py')),
            launch_arguments={
                **{key: files[key] for key in
                   ('params_file', 'mid360_config', 'pointlio_config', 'backend_config', 'prior_map', 'map')},
                'model': value('model'), 'use_sim_time': 'true', 'start_driver': 'false',
                'rviz': value('rviz'), 'autostart': value('autostart'),
            }.items())]),
        Node(package='sentry_simulation', executable='sentry_sim_cmd_adapter',
             name='swerve_navigation_command', output='screen', parameters=[{
                 'use_sim_time': True, 'chassis_type': 'swerve', 'input_topic': '/cmd_vel',
                 'output_topic': '/kinco_swerve/cmd_vel/selected',
             }]),
    ]


def generate_launch_description():
    share = Path(os.environ.get('SENTRY_SIMULATION_SHARE', get_package_share_directory('sentry_simulation')))
    config = Path(get_package_share_directory('perception_config'))
    defaults = {
        'world': 'home_large', 'headless': 'false', 'rviz': 'true', 'autostart': 'true',
        'params_file': str(Path(get_package_share_directory('navi2')) / 'params/navigation.yaml'),
        'mid360_config': str(config / 'mid360.yaml'),
        'pointlio_config': str(config / 'pointlio.yaml'),
        'backend_config': str(config / 'localization_room.yaml'),
        'platform_config': str(share / 'config/swerve_odin.yaml'),
        'prior_map': '', 'map': '', 'model': '',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=val) for key, val in defaults.items()],
        OpaqueFunction(function=_launch),
    ])

#!/usr/bin/env python3

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import ReplaceString


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, package_share):
    world_name = LaunchConfiguration("world").perform(context)
    chassis_type = LaunchConfiguration("chassis_type").perform(context)
    headless = _as_bool(LaunchConfiguration("headless").perform(context))

    if chassis_type not in ("omni", "diff"):
        raise RuntimeError("chassis_type must be 'omni' or 'diff'")

    worlds_config_path = os.path.join(package_share, "config", "worlds.yaml")
    with open(worlds_config_path, encoding="utf-8") as stream:
        worlds = yaml.safe_load(stream)
    if world_name not in worlds:
        raise RuntimeError(
            f"unknown world '{world_name}', choose one of: {', '.join(sorted(worlds))}"
        )

    world_config = worlds[world_name]
    spawn = world_config["spawn"]
    world_path = os.path.join(package_share, world_config["world"])
    map_path = os.path.join(package_share, world_config["map"])
    model_path = os.path.join(
        package_share, "resource", "models", f"sentry_{chassis_type}", "model.sdf"
    )
    bridge_path = os.path.join(package_share, "config", "ros_gz_bridge.yaml")
    point_lio_path = os.path.join(package_share, "config", "point_lio_sim.yaml")
    nav2_params_path = os.path.join(package_share, "config", "nav2_sim.yaml")
    configured_nav2_params = ReplaceString(
        source_file=nav2_params_path,
        replacements={"<simulation_share>": package_share},
    )

    x = float(spawn["x"])
    y = float(spawn["y"])
    z = float(spawn["z"])
    yaw = float(spawn["yaw"])
    lidar_x = x + 0.20 * math.sin(yaw)
    lidar_y = y - 0.20 * math.cos(yaw)
    lidar_z = z + 0.58

    gz_args = f"-r {'-s ' if headless else ''}{world_path}"
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_version": "6", "gz_args": gz_args}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_sentry",
        output="screen",
        arguments=[
            "-file", model_path, "-name", "sentry",
            "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="simulation_bridge",
        output="screen",
        parameters=[{"config_file": bridge_path, "use_sim_time": True}],
    )

    point_lio_container = ComposableNodeContainer(
        name="livox_pointlio_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="sentry_simulation",
                plugin="sentry_simulation::PointCloudAdapter",
                name="pointcloud_adapter",
                parameters=[{
                    "use_sim_time": True,
                    "input_topic": "/sim/lidar/points",
                    "output_topic": "/velodyne_points",
                    "n_scan": 32,
                    "vertical_min_deg": -7.0,
                    "vertical_max_deg": 52.0,
                    "scan_period": 0.05,
                }],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="point_lio",
                plugin="point_lio::LaserMappingNode",
                name="laserMapping",
                parameters=[point_lio_path],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("navi2"), "launch", "navigation_launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": configured_nav2_params,
            "autostart": "true",
            "use_composition": "False",
            "planner_container_name": "livox_pointlio_container",
            "log_level": LaunchConfiguration("log_level"),
        }.items(),
    )

    return [
        gazebo,
        spawn_robot,
        bridge,
        point_lio_container,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_camera_init",
            output="screen",
            arguments=[
                str(lidar_x), str(lidar_y), str(lidar_z),
                str(yaw), "0", "0", "map", "camera_init",
            ],
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_path, "use_sim_time": True}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ),
        navigation,
        Node(
            package="sentry_simulation",
            executable="sentry_sim_cmd_adapter",
            name="sentry_sim_cmd_adapter",
            output="screen",
            parameters=[{"use_sim_time": True, "chassis_type": chassis_type}],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=[
                "-d", os.path.join(get_package_share_directory("navi2"), "config", "our.rviz")
            ],
            parameters=[{"use_sim_time": True}],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("sentry_simulation")
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="rmuc_2025",
            description="rmuc_2024, rmul_2024, rmuc_2025, or rmul_2025",
        ),
        DeclareLaunchArgument(
            "chassis_type", default_value="omni", description="omni or diff"
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_launch_setup, args=[package_share]),
    ])

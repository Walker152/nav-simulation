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
    use_icp = _as_bool(LaunchConfiguration("use_icp").perform(context))

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
    gicp_path = os.path.join(package_share, "config", "gicp_sim.yaml")
    mid360_pattern_path = os.path.join(
        package_share,
        "resource",
        "models",
        "mid360",
        "scan_mode",
        "mid360-real-centr.csv",
    )
    nav2_params_path = os.path.join(package_share, "config", "nav2_sim.yaml")
    configured_nav2_params = ReplaceString(
        source_file=nav2_params_path,
        replacements={"<simulation_share>": package_share},
    )

    x = float(spawn["x"])
    y = float(spawn["y"])
    z = float(spawn["z"])
    yaw = float(spawn["yaw"])
    fusion_x = x + 0.2 * math.sin(yaw)
    fusion_y = y - 0.2 * math.cos(yaw)

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

    imu_filter = Node(
        package="sentry_simulation",
        executable="sentry_sim_imu_filter",
        name="sentry_sim_imu_filter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_topic": "/sim/imu_raw",
            "output_topic": "/sim/imu",
            "acceleration_limit": 29.43,
            "angular_velocity_limit": 35.0,
            "dynamic_acceleration_limit": 4.0,
            "vertical_dynamic_acceleration_limit": 0.0,
        }],
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
                    "left_front_input_topic": "/sim/lidar/left/front/points",
                    "left_rear_input_topic": "/sim/lidar/left/rear/points",
                    "right_front_input_topic": "/sim/lidar/right/front/points",
                    "right_rear_input_topic": "/sim/lidar/right/rear/points",
                    "output_topic": "/livox/lidar",
                    "output_frame_id": "sim_lidar",
            "left_sensor_pose": [-0.0496, 0.352530918687, 0.0, -0.5835987756, 0.0, 0.0],
            "right_sensor_pose": [-0.0496, 0.047469081313, 0.0, 0.5835987756, 0.0, 0.0],
                    "pattern_file": mid360_pattern_path,
                    "pattern_points_per_frame": 20000,
                    "sync_tolerance_ms": 5.0,
                    "vertical_min_deg": -7.3,
                    "vertical_max_deg": 52.3,
                    "scan_period": 0.1,
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

    localization_actions = []
    icp_config = world_config.get("icp", {})
    if use_icp and bool(icp_config.get("enable", False)):
        pcd_path = os.path.join(package_share, icp_config["target_pcd"])
        if not os.path.isfile(pcd_path):
            raise RuntimeError(f"model PCD does not exist: {pcd_path}")
        pcd_to_map_x, pcd_to_map_y, pcd_to_map_yaw = [
            float(value) for value in icp_config["pcd_to_map"]
        ]
        delta_x = fusion_x - pcd_to_map_x
        delta_y = fusion_y - pcd_to_map_y
        cos_yaw = math.cos(pcd_to_map_yaw)
        sin_yaw = math.sin(pcd_to_map_yaw)
        initial_pose = [
            cos_yaw * delta_x + sin_yaw * delta_y,
            -sin_yaw * delta_x + cos_yaw * delta_y,
            0.0,
            0.0,
            0.0,
            yaw - pcd_to_map_yaw,
        ]
        localization_actions.extend([
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_pcd_map",
                output="screen",
                arguments=[
                    str(pcd_to_map_x), str(pcd_to_map_y), "0",
                    str(pcd_to_map_yaw), "0", "0", "map", "pcd_map",
                ],
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="icp_relocalization",
                executable="gicp_node",
                name="gicp_relocalization_node",
                output="screen",
                parameters=[
                    gicp_path,
                    {
                        "use_sim_time": True,
                        "target_pcd_file": pcd_path,
                        "initial_pose": initial_pose,
                        "default_pose_on_timeout": initial_pose,
                        "gicp.min_inlier_ratio": float(
                            icp_config["min_inlier_ratio"]
                        ),
                        "gicp.min_overlap_ratio": float(
                            icp_config["min_overlap_ratio"]
                        ),
                    },
                ],
            ),
        ])
    else:
        localization_actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_camera_init",
            output="screen",
            arguments=[
                str(fusion_x), str(fusion_y), "0", str(yaw), "0", "0", "map", "camera_init"
            ],
            parameters=[{"use_sim_time": True}],
        ))

    return [
        gazebo,
        spawn_robot,
        bridge,
        imu_filter,
        point_lio_container,
        *localization_actions,
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
    installed_package_share = get_package_share_directory("sentry_simulation")
    package_share = os.environ.get(
        "SENTRY_SIMULATION_SHARE", installed_package_share
    )
    if not os.path.isfile(os.path.join(package_share, "package.xml")):
        raise RuntimeError(
            f"sentry_simulation share directory is invalid: {package_share}"
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            "world", default_value="rmuc_2025",
            description="rmuc_2024, rmul_2024, rmuc_2025, rmuc_2026, or rmul_2025",
        ),
        DeclareLaunchArgument(
            "chassis_type", default_value="omni", description="omni or diff"
        ),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_icp", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_launch_setup, args=[package_share]),
    ])

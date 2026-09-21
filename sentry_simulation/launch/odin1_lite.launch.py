"""Standalone O1LITE sensor demonstration; no localization/navigation nodes."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _configure_transport(context, share):
    # Match the main simulation launch, including empty environment values.
    if (not context.environment.get("FASTRTPS_DEFAULT_PROFILES_FILE")
            and context.environment.get("ROS_LOCALHOST_ONLY") != "1"):
        return [SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE", str(share / "config/fastdds_shm.xml"))]
    return []


def generate_launch_description():
    share = Path(get_package_share_directory("sentry_simulation"))
    world = share / "resource/worlds/odin1_lite_demo.sdf"
    model = ET.parse(share / "resource/models/odin1_lite/model.sdf").getroot().find("model")
    mounted = ET.parse(world).getroot().find("world/include")
    transforms = [("world", "odin1_lite_body", mounted.findtext("pose"))]
    transforms.extend((frame.get("attached_to"), frame.get("name"), frame.findtext("pose"))
                      for frame in model.findall("frame"))
    actions = [
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=_configure_transport, kwargs={"share": share}),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(
                Path(get_package_share_directory("ros_gz_sim")) / "launch/gz_sim.launch.py")),
            launch_arguments={
                "gz_version": "6",
                "on_exit_shutdown": "true",
                "gz_args": ["-r ", PythonExpression([
                    "'-s ' if '", LaunchConfiguration("headless"), "'.lower() == 'true' else ''"
                ]), str(world)],
            }.items(),
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="odin1_lite_bridge",
             parameters=[{"config_file": str(share / "config/odin1_lite_bridge.yaml"),
                          "use_sim_time": True}], output="screen"),
    ]
    # Derive TF from the same SDF used by Gazebo, avoiding duplicated extrinsics.
    for parent, child, pose in transforms:
        x, y, z, roll, pitch, yaw = pose.split()
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher", name=f"tf_{child}",
            arguments=["--x", x, "--y", y, "--z", z, "--roll", roll, "--pitch", pitch,
                       "--yaw", yaw, "--frame-id", parent, "--child-frame-id", child],
            parameters=[{"use_sim_time": True}], output="screen",
        ))
    return LaunchDescription(actions)

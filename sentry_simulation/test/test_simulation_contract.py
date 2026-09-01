#!/usr/bin/env python3

from pathlib import Path
import importlib.util
import math
import os
import signal
import struct
import subprocess
from tempfile import TemporaryDirectory
import time
import unittest
import xml.etree.ElementTree as ET

from launch import LaunchContext
import yaml


REPO_ROOT = Path(__file__).resolve().parents[4]
SIM_ROOT = REPO_ROOT / "src" / "simulation"
PACKAGE_ROOT = SIM_ROOT / "sentry_simulation"


def read_pgm(path):
    with path.open("rb") as stream:
        def token():
            value = bytearray()
            while True:
                char = stream.read(1)
                if not char:
                    raise ValueError(f"incomplete PGM header: {path}")
                if char == b"#":
                    stream.readline()
                    continue
                if not char.isspace():
                    value.extend(char)
                    break
            while True:
                char = stream.read(1)
                if not char or char.isspace():
                    return bytes(value)
                value.extend(char)

        magic = token()
        width = int(token())
        height = int(token())
        max_value = int(token())
        if magic != b"P5" or max_value != 255:
            raise ValueError(f"unsupported PGM format: {path}")
        pixels = stream.read(width * height)
        if len(pixels) != width * height:
            raise ValueError(f"incomplete PGM payload: {path}")
        return width, height, pixels


class SimulationContractTest(unittest.TestCase):
    def test_command_adapter_exits_cleanly_on_sigint(self):
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join(
            filter(
                None,
                [str(PACKAGE_ROOT), environment.get("PYTHONPATH", "")],
            )
        )
        process = subprocess.Popen(
            [
                "python3",
                str(PACKAGE_ROOT / "sentry_simulation" / "cmd_vel_adapter.py"),
            ],
            cwd=REPO_ROOT,
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        try:
            time.sleep(1.0)
            process.send_signal(signal.SIGINT)
            output, _ = process.communicate(timeout=10)
        finally:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
        self.assertEqual(process.returncode, 0, output)

    def test_shell_preflight_runs_with_ros_setup_variables_unset(self):
        environment = os.environ.copy()
        environment.pop("AMENT_TRACE_SETUP_FILES", None)
        result = subprocess.run(
            [
                str(REPO_ROOT / "simlation.bash"),
                "omni",
                "rmuc_2025",
                "--check",
            ],
            cwd=REPO_ROOT,
            env=environment,
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_shell_preflight_accepts_options_before_positionals(self):
        result = subprocess.run(
            [
                str(REPO_ROOT / "simlation.bash"),
                "--check",
                "--headless",
                "--no-rviz",
            ],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn(
            "Simulation preflight passed for omni in rmuc_2025.",
            result.stdout,
        )

    def test_required_entrypoints_exist(self):
        required = [
            REPO_ROOT / "simlation.bash",
            SIM_ROOT / "README.md",
            PACKAGE_ROOT / "package.xml",
            PACKAGE_ROOT / "CMakeLists.txt",
            PACKAGE_ROOT / "launch" / "simulation.launch.py",
            PACKAGE_ROOT / "config" / "ros_gz_bridge.yaml",
            PACKAGE_ROOT / "config" / "point_lio_sim.yaml",
            PACKAGE_ROOT / "config" / "worlds.yaml",
        ]
        missing = [str(path.relative_to(REPO_ROOT)) for path in required if not path.is_file()]
        self.assertEqual(missing, [], f"missing simulation entrypoints: {missing}")

    def test_shell_entrypoint_checks_the_complete_runtime_overlay(self):
        script = (REPO_ROOT / "simlation.bash").read_text(encoding="utf-8")
        for package in (
            "sentry_simulation",
            "ros_gz_sim",
            "ros_gz_bridge",
            "navi2",
            "point_lio",
            "icp_relocalization",
            "minco_planner",
            "minco_controller",
            "rog_map",
        ):
            self.assertIn(package, script)
        self.assertIn("simulation_source_models", script)
        self.assertIn("SENTRY_SIMULATION_SHARE", script)
        self.assertIn("IGN_GAZEBO_RESOURCE_PATH", script)

    def test_manifest_declares_the_complete_navigation_runtime(self):
        root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {
            node.text.strip()
            for node in root
            if node.tag in ("depend", "exec_depend") and node.text
        }
        self.assertTrue(
            {
                "point_lio",
                "icp_relocalization",
                "navi2",
                "minco_planner",
                "minco_controller",
                "rog_map",
                "nav2_bringup",
                "spatio_temporal_voxel_layer",
            }
            <= dependencies
        )

    def test_robot_models_parse_and_select_expected_drive_plugins(self):
        models = {
            "omni": "MecanumDrive2",
            "diff": "DiffDrive",
        }
        for chassis_type, plugin_name in models.items():
            model_path = (
                PACKAGE_ROOT
                / "resource"
                / "models"
                / f"sentry_{chassis_type}"
                / "model.sdf"
            )
            self.assertTrue(model_path.is_file(), f"missing {chassis_type} model")
            root = ET.parse(model_path).getroot()
            plugins = " ".join(
                f"{node.attrib.get('filename', '')} {node.attrib.get('name', '')}"
                for node in root.findall(".//plugin")
            )
            self.assertIn(plugin_name, plugins)
            sensor_types = {node.attrib.get("type") for node in root.findall(".//sensor")}
            self.assertIn("gpu_lidar", sensor_types)
            self.assertIn("imu", sensor_types)

    def test_mecanum_drive2_plugin_is_built_and_packaged(self):
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("find_package(ignition-gazebo6 REQUIRED)", cmake)
        self.assertIn("add_library(MecanumDrive2 SHARED", cmake)
        self.assertIn("install(TARGETS MecanumDrive2 LIBRARY DESTINATION plugins)", cmake)

    def test_omni_wheels_form_a_square_and_use_chassis_wrench_drive(self):
        root = ET.parse(
            PACKAGE_ROOT / "resource" / "models" / "sentry_omni" / "model.sdf"
        ).getroot()
        model = root.find("model")
        wheel_links = {
            link.attrib["name"]: link
            for link in model.findall("link")
            if link.attrib["name"].endswith("_wheel")
        }
        self.assertEqual(
            set(wheel_links),
            {
                "front_left_wheel",
                "front_right_wheel",
                "rear_left_wheel",
                "rear_right_wheel",
            },
        )
        wheel_xy = {
            name: tuple(float(value) for value in link.findtext("pose").split()[:2])
            for name, link in wheel_links.items()
        }
        x_span = max(value[0] for value in wheel_xy.values()) - min(
            value[0] for value in wheel_xy.values()
        )
        y_span = max(value[1] for value in wheel_xy.values()) - min(
            value[1] for value in wheel_xy.values()
        )
        self.assertAlmostEqual(x_span, y_span, places=6)

        plugins = model.findall("plugin")
        mecanum_plugins = [
            plugin
            for plugin in plugins
            if "MecanumDrive" in plugin.attrib.get("name", "")
        ]
        self.assertEqual(len(mecanum_plugins), 1)
        self.assertEqual(
            mecanum_plugins[0].attrib.get("filename"),
            "MecanumDrive2",
        )
        self.assertEqual(
            mecanum_plugins[0].attrib.get("name"),
            "ignition::gazebo::systems::MecanumDrive2",
        )
        self.assertEqual(mecanum_plugins[0].findtext("chassis_link"), "base_link")
        for joint_name in (
            "front_left_joint",
            "front_right_joint",
            "rear_left_joint",
            "rear_right_joint",
        ):
            self.assertEqual(mecanum_plugins[0].findtext(joint_name), joint_name)

        ground_truth_publishers = [
            plugin
            for plugin in plugins
            if "OdometryPublisher" in plugin.attrib.get("name", "")
        ]
        self.assertEqual(len(ground_truth_publishers), 1)
        self.assertEqual(
            ground_truth_publishers[0].findtext("odom_topic"),
            "/sentry/ground_truth_odometry",
        )

        friction_directions = {
            link.findtext("collision/surface/friction/ode/fdir1")
            for link in wheel_links.values()
        }
        self.assertNotIn(None, friction_directions)
        self.assertEqual(friction_directions, {"1 1 0", "1 -1 0"})
        transverse_friction = {
            float(link.findtext("collision/surface/friction/ode/mu2"))
            for link in wheel_links.values()
        }
        self.assertTrue(
            all(value >= 0.2 for value in transverse_friction),
            "omni wheels need transverse grip to hold a diagonal ramp at low speed",
        )

    def test_diff_robot_is_a_radius_point_three_two_wheel_base(self):
        root = ET.parse(
            PACKAGE_ROOT / "resource" / "models" / "sentry_diff" / "model.sdf"
        ).getroot()
        model = root.find("model")
        wheel_links = {
            link.attrib["name"]
            for link in model.findall("link")
            if link.attrib["name"].endswith("_wheel")
        }
        self.assertEqual(wheel_links, {"left_wheel", "right_wheel"})
        self.assertAlmostEqual(
            float(model.findtext("link[@name='base_link']/collision/geometry/cylinder/radius")),
            0.3,
            places=6,
        )

        for position in ("front", "rear"):
            caster = model.find(f"link[@name='{position}_caster_ball']")
            self.assertIsNotNone(caster)
            self.assertIsNotNone(caster.find("collision/geometry/sphere"))
            caster_joint = model.find(f"joint[@name='{position}_caster_joint']")
            self.assertIsNotNone(caster_joint)
            self.assertEqual(caster_joint.attrib.get("type"), "ball")

        diff_plugins = [
            plugin
            for plugin in model.findall("plugin")
            if "DiffDrive" in plugin.attrib.get("name", "")
        ]
        self.assertEqual(len(diff_plugins), 1)
        plugin = diff_plugins[0]
        self.assertEqual(
            plugin.attrib.get("filename"), "ignition-gazebo-diff-drive-system"
        )
        self.assertEqual(
            [node.text for node in plugin.findall("left_joint")], ["left_joint"]
        )
        self.assertEqual(
            [node.text for node in plugin.findall("right_joint")], ["right_joint"]
        )
        self.assertAlmostEqual(float(plugin.findtext("wheel_separation")), 0.6)
        self.assertAlmostEqual(float(plugin.findtext("wheel_radius")), 0.076)
        self.assertEqual(plugin.findtext("odom_topic"), "/sentry/wheel_odometry")

        odometry_publishers = [
            plugin
            for plugin in model.findall("plugin")
            if "OdometryPublisher" in plugin.attrib.get("name", "")
        ]
        self.assertEqual(len(odometry_publishers), 1)
        self.assertEqual(
            odometry_publishers[0].findtext("odom_topic"),
            "/sentry/ground_truth_odometry",
        )

    def test_wheel_joint_axes_match_the_cylinder_spin_axes(self):
        for chassis_type in ("omni", "diff"):
            root = ET.parse(
                PACKAGE_ROOT
                / "resource"
                / "models"
                / f"sentry_{chassis_type}"
                / "model.sdf"
            ).getroot()
            model = root.find("model")
            joints_by_child = {
                joint.findtext("child"): joint for joint in model.findall("joint")
            }
            for link in model.findall("link"):
                if not link.attrib["name"].endswith("_wheel"):
                    continue
                roll = float(link.findtext("pose").split()[3])
                cylinder_axis_y = -math.sin(roll)
                cylinder_axis_z = math.cos(roll)
                self.assertGreater(cylinder_axis_y, 0.999, link.attrib["name"])
                self.assertAlmostEqual(cylinder_axis_z, 0.0, places=4)
                joint_axis = [
                    float(value)
                    for value in joints_by_child[link.attrib["name"]]
                    .findtext("axis/xyz")
                    .split()
                ]
                self.assertEqual(joint_axis, [0.0, 0.0, 1.0])

    def test_mid360_dense_grid_preserves_angular_resolution(self):
        for chassis_type in ("omni", "diff"):
            root = ET.parse(
                PACKAGE_ROOT
                / "resource"
                / "models"
                / f"sentry_{chassis_type}"
                / "model.sdf"
            ).getroot()
            lidars = root.findall(".//sensor[@type='gpu_lidar']")
            self.assertEqual(
                {lidar.attrib["name"] for lidar in lidars},
                {
                    "sim_lidar_left_front",
                    "sim_lidar_left_rear",
                    "sim_lidar_right_front",
                    "sim_lidar_right_rear",
                },
            )
            points_per_second = 0.0
            for lidar in lidars:
                horizontal = lidar.find("lidar/scan/horizontal")
                horizontal_samples = int(horizontal.findtext("samples"))
                vertical_samples = int(lidar.findtext("lidar/scan/vertical/samples"))
                update_rate = float(lidar.findtext("update_rate"))
                self.assertEqual(horizontal_samples, 360)
                self.assertEqual(vertical_samples, 120)
                self.assertEqual(update_rate, 10.0)
                horizontal_fov = float(horizontal.findtext("max_angle")) - float(
                    horizontal.findtext("min_angle")
                )
                self.assertLessEqual(horizontal_fov, math.pi + 1e-6)
                points_per_second += horizontal_samples * vertical_samples * update_rate

                noise = lidar.find("lidar/noise")
                self.assertIsNotNone(noise)
                self.assertEqual(noise.findtext("type"), "gaussian")
                self.assertAlmostEqual(float(noise.findtext("stddev")), 0.0)
            self.assertGreaterEqual(points_per_second, 1_600_000)
            self.assertLessEqual(points_per_second, 2_000_000)

            for side in ("left", "right"):
                mesh_uri = root.findtext(
                    f".//link[@name='sim_lidar_{side}']/visual/geometry/mesh/uri"
                )
                self.assertEqual(mesh_uri, "model://mid360/meshes/mid360.dae")

    def test_mid360_scan_pattern_and_adapter_are_packaged(self):
        model_dir = PACKAGE_ROOT / "resource" / "models" / "mid360"
        self.assertTrue((model_dir / "model.config").is_file())
        self.assertGreater((model_dir / "meshes" / "mid360.dae").stat().st_size, 3_000_000)
        pattern_path = model_dir / "scan_mode" / "mid360-real-centr.csv"
        self.assertTrue(pattern_path.is_file())
        with pattern_path.open(encoding="utf-8") as stream:
            self.assertGreaterEqual(sum(1 for _ in stream), 100_000)

        launch_source = (
            PACKAGE_ROOT / "launch" / "simulation.launch.py"
        ).read_text(encoding="utf-8")
        for token in (
            "pattern_file",
            "mid360-real-centr.csv",
            '"pattern_points_per_frame": 20000',
            '"sync_tolerance_ms": 5.0',
            '"scan_period": 0.1',
        ):
            self.assertIn(token, launch_source)

        adapter_source = (
            PACKAGE_ROOT / "src" / "pointcloud_adapter.cpp"
        ).read_text(encoding="utf-8")
        for token in ("PatternRay", "pattern_file", "pattern_points_per_frame"):
            self.assertIn(token, adapter_source)

    def test_compact_chassis_and_dual_mid360_mounts_follow_description(self):
        visual_dir = PACKAGE_ROOT / "resource" / "models" / "pb2025_visuals" / "meshes"
        expected_assets = {
            "chassis_base.dae": 7_000_000,
        }
        for name, minimum_size in expected_assets.items():
            self.assertGreater((visual_dir / name).stat().st_size, minimum_size)

        models = {}
        for chassis_type in ("omni", "diff"):
            model = ET.parse(
                PACKAGE_ROOT
                / "resource"
                / "models"
                / f"sentry_{chassis_type}"
                / "model.sdf"
            ).getroot().find("model")
            models[chassis_type] = model
            visual_uris = {
                node.text for node in model.findall(".//visual/geometry/mesh/uri")
            }
            self.assertNotIn("model://pb2025_visuals/meshes/gimbal_yaw.dae", visual_uris)
            self.assertNotIn("model://pb2025_visuals/meshes/gimbal_pitch.dae", visual_uris)

            common_pose = [
                float(value)
                for value in model.find("link[@name='sim_lidar']").findtext("pose").split()
            ]
            self.assertEqual(common_pose, [0.0, -0.2, 0.3, 0.0, 0.0, 0.0])
            self.assertIsNotNone(model.find("link[@name='sim_lidar']/sensor[@type='imu']"))

            left_pose = [
                float(value)
                for value in model.find("link[@name='sim_lidar_left']").findtext("pose").split()
            ]
            right_pose = [
                float(value)
                for value in model.find("link[@name='sim_lidar_right']").findtext("pose").split()
            ]
            self.assertEqual(left_pose[:3], [-0.0496, 0.136, 0.274965449328])
            self.assertEqual(right_pose[:3], [-0.0496, -0.136, 0.274965449328])
            self.assertAlmostEqual(left_pose[3], -right_pose[3], places=9)
            self.assertAlmostEqual(abs(left_pose[3]), math.pi / 6.0 + 0.06, places=9)
            self.assertEqual(left_pose[4:], [0.0, 0.0])
            self.assertEqual(right_pose[4:], [0.0, 0.0])

            # PB's MID360 optical origin is 30 mm above the mesh origin.  Keep
            # the optical origins at 300 mm while the 65 mm housing clears the
            # 240 mm chassis; otherwise shallow downward rays hit the robot.
            for side, link_pose in (("left", left_pose), ("right", right_pose)):
                link = model.find(f"link[@name='sim_lidar_{side}']")
                front_pose = [
                    float(value)
                    for value in next(
                        sensor
                        for sensor in link.findall("sensor")
                        if sensor.attrib["name"].endswith("front")
                    ).findtext("pose").split()
                ]
                rear_pose = [
                    float(value)
                    for value in next(
                        sensor
                        for sensor in link.findall("sensor")
                        if sensor.attrib["name"].endswith("rear")
                    ).findtext("pose").split()
                ]
                self.assertEqual(front_pose, [0.0, 0.0, 0.03, 0.0, 0.0, 0.0])
                self.assertEqual(rear_pose[:3], [0.0, 0.0, 0.03])
                self.assertAlmostEqual(abs(rear_pose[5]), math.pi, places=9)
                optical_z = link_pose[2] + 0.03 * math.cos(link_pose[3])
                self.assertAlmostEqual(optical_z, 0.3, places=9)
                self.assertGreater(link_pose[2] - 0.03243692, 0.24)
                collision = link.find("collision")
                collision_pose = [
                    float(value)
                    for value in (collision.findtext("pose") or "0 0 0 0 0 0").split()
                ]
                collision_size = [
                    float(value)
                    for value in collision.findtext("geometry/box/size").split()
                ]
                self.assertEqual(collision_pose, [0.0] * 6)
                self.assertEqual(collision_size, [0.065, 0.06, 0.065])

        omni_visual_uris = {
            node.text
            for node in models["omni"].findall(".//visual/geometry/mesh/uri")
        }
        self.assertIn(
            "model://pb2025_visuals/meshes/chassis_base.dae", omni_visual_uris
        )
        omni_base_visual = models["omni"].find(
            "link[@name='base_link']/visual[@name='base_visual']"
        )
        scale = [
            float(value)
            for value in omni_base_visual.findtext("geometry/mesh/scale").split()
        ]
        self.assertEqual(scale[:2], [1.0, 1.0])
        self.assertAlmostEqual(scale[2], 0.852565282, places=9)
        base_z = float(models["omni"].find("link[@name='base_link']").findtext("pose").split()[2])
        visual_z = float(omni_base_visual.findtext("pose").split()[2])
        self.assertAlmostEqual(
            base_z + visual_z + 0.253044 * scale[2],
            0.24,
            places=6,
        )

        catalog = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "worlds.yaml").read_text(encoding="utf-8")
        )
        self.assertTrue(
            all(item["spawn_clearance"] == 0.0 for item in catalog.values())
        )
        launch_source = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text(
            encoding="utf-8"
        )
        for token in (
            '"left_sensor_pose": [-0.0496, 0.352530918687, 0.0, -0.5835987756',
            '"right_sensor_pose": [-0.0496, 0.047469081313, 0.0, 0.5835987756',
            '"output_frame_id": "sim_lidar"',
            "fusion_x = x + 0.2 * math.sin(yaw)",
            "fusion_y = y - 0.2 * math.cos(yaw)",
        ):
            self.assertIn(token, launch_source)

    def test_all_world_files_are_valid_sdf(self):
        worlds_dir = PACKAGE_ROOT / "resource" / "worlds"
        expected = {"rmuc_2024", "rmul_2024", "rmuc_2025", "rmuc_2026", "rmul_2025"}
        actual = {path.stem.removesuffix("_world") for path in worlds_dir.glob("*_world.sdf")}
        self.assertEqual(actual, expected)
        for world in worlds_dir.glob("*_world.sdf"):
            root = ET.parse(world).getroot()
            self.assertEqual(root.tag, "sdf")
            gravity = root.findtext("world/gravity")
            self.assertIsNotNone(gravity, world.name)
            self.assertEqual(
                [float(value) for value in gravity.split()],
                [0.0, 0.0, -9.81],
                world.name,
            )

    def test_2025_arena_floor_is_on_world_zero_plane(self):
        for arena in ("rmuc_2025", "rmul_2025"):
            model = ET.parse(
                PACKAGE_ROOT / "resource" / "models" / arena / "model.sdf"
            ).getroot().find("model")
            self.assertIsNotNone(model, arena)
            link_pose = [
                float(value) for value in model.find("link").findtext("pose").split()
            ]
            self.assertAlmostEqual(link_pose[2], 0.0, msg=arena)
            visual_uri = model.findtext("link/visual/geometry/mesh/uri")
            collision_uri = model.findtext("link/collision/geometry/mesh/uri")
            self.assertEqual(collision_uri, visual_uri, arena)

            world = ET.parse(
                PACKAGE_ROOT / "resource" / "worlds" / f"{arena}_world.sdf"
            ).getroot().find("world")
            included_uris = {node.text for node in world.findall(".//include/uri")}
            self.assertNotIn(f"model://{arena}_collision", included_uris)

    def test_rmuc_2025_uses_the_complete_original_mesh_for_physics(self):
        world = ET.parse(
            PACKAGE_ROOT / "resource" / "worlds" / "rmuc_2025_world.sdf"
        ).getroot().find("world")
        self.assertIsNone(world.find("model[@name='ground_plane']"))

        mesh_path = (
            PACKAGE_ROOT
            / "resource/models/rmuc_2025/meshes/rmuc_2025.stl"
        )
        payload = mesh_path.read_bytes()
        triangle_count = struct.unpack_from("<I", payload, 80)[0]
        self.assertEqual(triangle_count, 114234)
        giant_floor_triangles = 0
        for index in range(triangle_count):
            values = struct.unpack_from("<12fH", payload, 84 + index * 50)
            vertices = (values[3:6], values[6:9], values[9:12])
            x_span = max(vertex[0] for vertex in vertices) - min(
                vertex[0] for vertex in vertices
            )
            y_span = max(vertex[1] for vertex in vertices) - min(
                vertex[1] for vertex in vertices
            )
            on_zero_plane = all(abs(vertex[2]) < 1.0e-4 for vertex in vertices)
            giant_floor_triangles += int(
                x_span > 20.0 and y_span > 10.0 and on_zero_plane
            )
        self.assertEqual(giant_floor_triangles, 2)

    def test_bridge_covers_clock_sensors_command_and_ground_truth(self):
        bridge_path = PACKAGE_ROOT / "config" / "ros_gz_bridge.yaml"
        self.assertTrue(bridge_path.is_file(), "bridge configuration is missing")
        entries = yaml.safe_load(bridge_path.read_text(encoding="utf-8"))
        ros_topics = {entry["ros_topic_name"] for entry in entries}
        self.assertTrue(
            {
                "/clock",
                "/sim/lidar/left/front/points",
                "/sim/lidar/left/rear/points",
                "/sim/lidar/right/front/points",
                "/sim/lidar/right/rear/points",
                "/sim/imu_raw",
                "/sim/cmd_vel",
                "/sim/ground_truth/odom",
            }
            <= ros_topics
        )
        ground_truth = next(
            entry
            for entry in entries
            if entry["ros_topic_name"] == "/sim/ground_truth/odom"
        )
        self.assertEqual(
            ground_truth["gz_topic_name"], "/sentry/ground_truth_odometry"
        )

    def test_gazebo_imu_exposes_per_axis_saturation_to_point_lio(self):
        launch_source = (
            PACKAGE_ROOT / "launch" / "simulation.launch.py"
        ).read_text(encoding="utf-8")
        for token in (
            'executable="sentry_sim_imu_filter"',
            '"input_topic": "/sim/imu_raw"',
            '"output_topic": "/sim/imu"',
            '"acceleration_limit": 29.43',
            '"angular_velocity_limit": 35.0',
            '"dynamic_acceleration_limit": 4.0',
            '"vertical_dynamic_acceleration_limit": 0.0',
        ):
            self.assertIn(token, launch_source)
        self.assertNotIn('"acceleration_max_step"', launch_source)

        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("sentry_simulation/imu_filter.py", cmake)
        self.assertIn("RENAME sentry_sim_imu_filter", cmake)

    def test_point_lio_simulation_uses_livox_custom_msg_and_sim_time(self):
        config_path = PACKAGE_ROOT / "config" / "point_lio_sim.yaml"
        self.assertTrue(config_path.is_file(), "Point-LIO simulation config is missing")
        params = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
        self.assertTrue(params["use_sim_time"])
        self.assertEqual(params["common"]["lid_topic"], "/livox/lidar")
        self.assertEqual(params["common"]["imu_topic"], "/sim/imu")
        self.assertEqual(params["preprocess"]["lidar_type"], 1)
        self.assertEqual(params["preprocess"]["scan_line"], 4)
        self.assertEqual(params["preprocess"]["timestamp_unit"], 3)
        self.assertEqual(params["preprocess"]["blind_center"], [0.0, 0.2, 0.0])
        self.assertAlmostEqual(params["mapping"]["lidar_meas_cov"], 0.01)
        self.assertAlmostEqual(params["mapping"]["lidar_time_inte"], 0.1)

    def test_world_catalog_references_packaged_worlds_and_maps(self):
        catalog = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "worlds.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            set(catalog),
            {"rmuc_2024", "rmul_2024", "rmuc_2025", "rmuc_2026", "rmul_2025"},
        )
        for item in catalog.values():
            self.assertTrue((PACKAGE_ROOT / item["world"]).is_file())
            self.assertTrue((PACKAGE_ROOT / item["map"]).is_file())

    def test_each_spawn_is_in_free_map_space(self):
        catalog = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "worlds.yaml").read_text(encoding="utf-8")
        )
        for name, item in catalog.items():
            map_path = PACKAGE_ROOT / item["map"]
            metadata = yaml.safe_load(map_path.read_text(encoding="utf-8"))
            width, height, pixels = read_pgm(map_path.parent / metadata["image"])
            resolution = float(metadata["resolution"])
            origin_x, origin_y, _ = metadata["origin"]
            x = float(item["spawn"]["x"])
            y = float(item["spawn"]["y"])
            column = int((x - origin_x) / resolution)
            row = height - 1 - int((y - origin_y) / resolution)
            self.assertGreaterEqual(column, 0, name)
            self.assertLess(column, width, name)
            self.assertGreaterEqual(row, 0, name)
            self.assertLess(row, height, name)
            self.assertGreaterEqual(pixels[row * width + column], 250, name)
            clearance_cells = math.ceil(0.25 / resolution)
            for delta_row in range(-clearance_cells, clearance_cells + 1):
                for delta_column in range(-clearance_cells, clearance_cells + 1):
                    if math.hypot(delta_row, delta_column) > clearance_cells:
                        continue
                    check_row = row + delta_row
                    check_column = column + delta_column
                    self.assertGreaterEqual(check_row, 0, name)
                    self.assertLess(check_row, height, name)
                    self.assertGreaterEqual(check_column, 0, name)
                    self.assertLess(check_column, width, name)
                    self.assertGreaterEqual(
                        pixels[check_row * width + check_column], 250, name
                    )

    def test_maps_use_zero_origin_and_real_icp_spawn_pose(self):
        catalog = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "worlds.yaml").read_text(encoding="utf-8")
        )
        for name, item in catalog.items():
            metadata = yaml.safe_load(
                (PACKAGE_ROOT / item["map"]).read_text(encoding="utf-8")
            )
            self.assertEqual(metadata["origin"], [0, 0, 0], name)
            expected = (
                {"x": 4.234, "y": 7.3, "z": 0.0, "yaw": 0.04}
                if name.startswith("rmuc_")
                else {"x": 2.0, "y": 6.0, "z": 0.0, "yaw": 0.4}
            )
            self.assertEqual(item["spawn"], expected, name)

    def test_2025_arenas_use_the_visual_mesh_as_physical_collision(self):
        for arena in ("rmuc_2025", "rmul_2025"):
            arena_model = ET.parse(
                PACKAGE_ROOT / "resource" / "models" / arena / "model.sdf"
            ).getroot().find("model")
            visual_uri = arena_model.findtext("link/visual/geometry/mesh/uri")
            collision_uri = arena_model.findtext("link/collision/geometry/mesh/uri")
            self.assertEqual(collision_uri, visual_uri, arena)

            world = ET.parse(
                PACKAGE_ROOT / "resource" / "worlds" / f"{arena}_world.sdf"
            ).getroot()
            included_models = {node.text for node in world.findall(".//include/uri")}
            self.assertNotIn(f"model://{arena}_collision", included_models)

    def test_rmuc_map_does_not_extend_the_wall_past_the_visual_corner(self):
        map_path = PACKAGE_ROOT / "maps" / "rmuc_2025.pgm"
        width, height, pixels = read_pgm(map_path)
        resolution = 0.05

        # The visual wall ends near x=7.568.  The stable primitive collision is
        # generated from this map, so extending the wall farther creates a fake
        # corner that the controller can hit even though the planned opening is free.
        for x in (7.70, 7.85, 8.05):
            column = round(x / resolution)
            occupied_past_corner = any(
                pixels[row * width + column] < 65
                for row in range(
                    height - 1 - round(3.90 / resolution),
                    height - round(3.80 / resolution),
                )
            )
            self.assertFalse(occupied_past_corner, f"fake RMUC wall cell near x={x}")

    def test_simulation_icp_uses_packaged_model_clouds(self):
        config_path = PACKAGE_ROOT / "config" / "gicp_sim.yaml"
        self.assertTrue(config_path.is_file())
        params = yaml.safe_load(config_path.read_text(encoding="utf-8"))[
            "gicp_relocalization_node"
        ]["ros__parameters"]
        self.assertEqual(params["map_frame"], "pcd_map")
        self.assertEqual(params["source_cloud_topic"], "/cloud_registered_full")
        self.assertFalse(params["enable_continuous_relocalization"])
        self.assertFalse(params["results"]["enable"])
        # Raw score scales with the dual-lidar point count. Keep it disabled and
        # accept alignment using normalized score, inlier ratio and overlap ratio.
        self.assertLessEqual(params["gicp"]["score_threshold"], 0.0)
        self.assertLessEqual(params["gicp"]["normalized_score_threshold"], 0.2)
        self.assertAlmostEqual(params["gicp"]["min_inlier_ratio"], 0.4)
        self.assertGreaterEqual(params["gicp"]["min_overlap_ratio"], 0.5)

        pcd_dir = PACKAGE_ROOT / "resource" / "maps" / "pcd"
        for filename in ("rmuc_model.pcd", "rmuc_2026.pcd", "rmul_model.pcd"):
            path = pcd_dir / filename
            self.assertTrue(path.is_file(), filename)
            with path.open("rb") as stream:
                header = stream.read(512)
            self.assertIn(b"FIELDS", header)
            self.assertIn(b"POINTS", header)

        catalog = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "worlds.yaml").read_text(encoding="utf-8")
        )
        for name in ("rmuc_2025", "rmuc_2026", "rmul_2025"):
            self.assertTrue(catalog[name]["icp"]["enable"])
            target = catalog[name]["icp"]["target_pcd"]
            self.assertNotIn("red", target)
            self.assertNotIn("blue", target)
            self.assertIn("min_inlier_ratio", catalog[name]["icp"])
            self.assertIn("min_overlap_ratio", catalog[name]["icp"])

        self.assertEqual(catalog["rmuc_2025"]["icp"]["min_inlier_ratio"], 0.4)
        self.assertEqual(catalog["rmuc_2025"]["icp"]["min_overlap_ratio"], 0.5)
        self.assertEqual(catalog["rmuc_2026"]["icp"]["min_inlier_ratio"], 0.4)
        self.assertEqual(catalog["rmuc_2026"]["icp"]["min_overlap_ratio"], 0.5)
        self.assertEqual(catalog["rmul_2025"]["icp"]["min_inlier_ratio"], 0.35)
        self.assertEqual(catalog["rmul_2025"]["icp"]["min_overlap_ratio"], 0.42)

        launch_source = (PACKAGE_ROOT / "launch" / "simulation.launch.py").read_text(
            encoding="utf-8"
        )
        self.assertIn('package="icp_relocalization"', launch_source)
        self.assertIn('executable="gicp_node"', launch_source)
        self.assertIn('"map", "pcd_map"', launch_source)
        self.assertIn('"gicp.min_inlier_ratio"', launch_source)
        self.assertIn('"gicp.min_overlap_ratio"', launch_source)

    def test_rmuc_and_rmul_model_cloud_alignment_is_explicit(self):
        catalog = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "worlds.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            catalog["rmuc_2025"]["icp"]["pcd_to_map"],
            [-0.18495, 0.38684, -0.022672],
        )
        self.assertEqual(catalog["rmuc_2026"]["icp"]["pcd_to_map"], [0.0, 0.0, 0.0])
        self.assertEqual(
            catalog["rmul_2025"]["icp"]["pcd_to_map"],
            [-0.059784, -0.060093, -0.002569],
        )

        width, height, _ = read_pgm(PACKAGE_ROOT / "maps" / "rmul_2025.pgm")
        self.assertEqual((width, height), (253, 173))

        world = ET.parse(
            PACKAGE_ROOT / "resource" / "worlds" / "rmul_2025_world.sdf"
        ).getroot()
        model = world.find(".//model[@name='rmul_2025']")
        self.assertIsNotNone(model)
        self.assertEqual(
            [float(value) for value in model.findtext("pose").split()[:3]],
            [6.3, 4.3, 0.0],
        )

    def test_navigation_simulation_parameters_use_clock_and_online_rogmap(self):
        config_path = PACKAGE_ROOT / "config" / "nav2_sim.yaml"
        text = config_path.read_text(encoding="utf-8")
        self.assertNotIn("use_sim_time: False", text)
        params = yaml.safe_load(text)
        planner = params["planner_server"]["ros__parameters"]["MincoPlanner"]
        self.assertFalse(planner["rog_map"]["projection"]["prior_map"]["enable"])
        local_stvl = params["local_costmap"]["local_costmap"]["ros__parameters"][
            "stvl_layer"
        ]
        self.assertEqual(local_stvl["mid360"]["topic"], "/cloud_registered")
        controller = params["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertAlmostEqual(controller["lidar_offset_x"], 0.0)
        self.assertAlmostEqual(controller["lidar_offset_y"], 0.0)
        self.assertAlmostEqual(controller["lidar_roll_offset"], 0.0)
        planner = params["planner_server"]["ros__parameters"]["MincoPlanner"]
        self.assertAlmostEqual(planner["lidar_offset_x"], 0.0)
        self.assertAlmostEqual(planner["lidar_offset_y"], 0.0)

        local = params["local_costmap"]["local_costmap"]["ros__parameters"]
        global_costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertEqual(
            local["global_frame"],
            "map",
            "simulation goal checking must not timestamp-transform map paths through dynamic GICP TF",
        )
        expected_footprint = (
            '[[0.30, 0.00], [0.296, 0.22], [0.274, 0.274], [0.22, 0.296], '
            '[0.00, 0.32], [-0.22, 0.296], [-0.274, 0.274], [-0.296, 0.22], '
            '[-0.30, 0.00], [-0.296, -0.22], [-0.274, -0.274], [-0.22, -0.296], '
            '[0.00, -0.32], [0.22, -0.296], [0.274, -0.274], [0.296, -0.22]]'
        )
        self.assertEqual(local["footprint"], expected_footprint)
        self.assertEqual(global_costmap["footprint"], expected_footprint)
        self.assertGreaterEqual(local["inflation_layer"]["inflation_radius"], 0.50)
        self.assertAlmostEqual(planner["corridor"]["robot_radius"], 0.42)
        self.assertAlmostEqual(planner["minco_optimizer"]["safe_dist"], 0.45)

        projection = planner["rog_map"]["projection"]
        self.assertAlmostEqual(projection["surface_height_delta_max"], 0.25)
        self.assertAlmostEqual(projection["wall_height_delta_min"], 0.50)
        self.assertAlmostEqual(projection["tunnel_height_delta_min"], 0.26)
        self.assertAlmostEqual(projection["obstacle_hold_time"], 0.0)

    def test_behavior_trees_are_packaged_and_do_not_depend_on_source_cwd(self):
        config_path = PACKAGE_ROOT / "config" / "nav2_sim.yaml"
        params = yaml.safe_load(config_path.read_text(encoding="utf-8"))
        bt_params = params["bt_navigator"]["ros__parameters"]
        for key in (
            "default_nav_through_poses_bt_xml",
            "default_nav_to_pose_bt_xml",
        ):
            configured_path = bt_params[key]
            self.assertTrue(configured_path.startswith("<simulation_share>/behavior_tree/"))
            packaged_path = PACKAGE_ROOT / configured_path.split("/behavior_tree/", 1)[1]
            packaged_path = PACKAGE_ROOT / "behavior_tree" / packaged_path.name
            self.assertTrue(packaged_path.is_file())

        launch_text = (
            PACKAGE_ROOT / "launch" / "simulation.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn("ReplaceString", launch_text)
        self.assertIn('"<simulation_share>": package_share', launch_text)

    def test_launch_runs_full_pipeline_without_communication(self):
        launch_path = PACKAGE_ROOT / "launch" / "simulation.launch.py"
        self.assertTrue(launch_path.is_file(), "simulation launch is missing")
        launch_text = launch_path.read_text(encoding="utf-8")
        self.assertIn("point_lio::LaserMappingNode", launch_text)
        self.assertIn("navigation_launch.py", launch_text)
        self.assertIn("sentry_sim_cmd_adapter", launch_text)
        self.assertIn('"use_composition": "False"', launch_text)
        self.assertNotIn("package=\"communication\"", launch_text)
        self.assertNotIn("package='communication'", launch_text)

    def test_launch_setup_instantiates_for_every_chassis_and_world(self):
        launch_path = PACKAGE_ROOT / "launch" / "simulation.launch.py"
        spec = importlib.util.spec_from_file_location("sentry_simulation_launch", launch_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        with TemporaryDirectory() as temporary_directory:
            fake_ros_gz_share = Path(temporary_directory)
            (fake_ros_gz_share / "launch").mkdir()
            (fake_ros_gz_share / "launch" / "gz_sim.launch.py").write_text(
                "", encoding="utf-8"
            )
            shares = {
                "ros_gz_sim": str(fake_ros_gz_share),
                "navi2": str(REPO_ROOT / "src" / "navigation" / "navi2_bringup"),
            }
            module.get_package_share_directory = lambda name: shares[name]

            for chassis_type in ("omni", "diff"):
                for world in (
                    "rmuc_2024",
                    "rmul_2024",
                    "rmuc_2025",
                    "rmuc_2026",
                    "rmul_2025",
                ):
                    context = LaunchContext()
                    context.launch_configurations.update(
                        {
                            "world": world,
                            "chassis_type": chassis_type,
                            "headless": "true",
                            "rviz": "false",
                            "use_icp": "true",
                            "log_level": "info",
                        }
                    )
                    actions = module._launch_setup(context, str(PACKAGE_ROOT))
                    self.assertGreaterEqual(len(actions), 10)

    def test_pointcloud_adapter_preserves_simulation_contract(self):
        source_path = PACKAGE_ROOT / "src" / "pointcloud_adapter.cpp"
        self.assertTrue(source_path.is_file(), "pointcloud adapter source is missing")
        source = source_path.read_text(encoding="utf-8")
        for token in [
            "livox_ros_driver2::msg::CustomMsg",
            "livox_ros_driver2::msg::CustomPoint",
            "offset_time",
            "line",
            "tag",
            "SensorDataQoS",
            "UniquePtr",
            "left_front_cloud_callback",
            "left_rear_cloud_callback",
            "right_front_cloud_callback",
            "right_rear_cloud_callback",
            "transform_point",
            "sync_tolerance_ms",
        ]:
            self.assertIn(token, source)

        manifest = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
        dependencies = {
            node.text.strip()
            for node in manifest
            if node.tag in ("depend", "exec_depend") and node.text
        }
        self.assertIn("livox_ros_driver2", dependencies)
        cmake = (PACKAGE_ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("find_package(livox_ros_driver2 REQUIRED)", cmake)

    def test_pointcloud_adapter_uses_instantaneous_gazebo_snapshot_time(self):
        source = (PACKAGE_ROOT / "src" / "pointcloud_adapter.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn("snapshot_stamp", source)
        self.assertIn("output->header.stamp = snapshot_stamp", source)
        self.assertIn(
            "output->timebase = static_cast<std::uint64_t>(snapshot_stamp.nanoseconds())",
            source,
        )
        self.assertIn("constexpr std::uint32_t snapshot_offset_time = 0", source)
        self.assertNotIn("scan_start_stamp", source)
        self.assertNotIn(
            "scan_period_ * static_cast<double>(offset) /",
            source,
            "an instantaneous Gazebo depth snapshot must not be assigned a fake scan sweep",
        )


if __name__ == "__main__":
    unittest.main()

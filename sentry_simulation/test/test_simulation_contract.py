#!/usr/bin/env python3

from pathlib import Path
import importlib.util
import math
import os
import signal
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
            "minco_planner",
            "minco_controller",
            "rog_map",
        ):
            self.assertIn(package, script)

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
            "omni": "MecanumDrive",
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

    def test_lidar_point_rate_stays_within_the_realtime_budget(self):
        for chassis_type in ("omni", "diff"):
            root = ET.parse(
                PACKAGE_ROOT
                / "resource"
                / "models"
                / f"sentry_{chassis_type}"
                / "model.sdf"
            ).getroot()
            lidar = root.find(".//sensor[@type='gpu_lidar']")
            horizontal_samples = int(lidar.findtext("lidar/scan/horizontal/samples"))
            vertical_samples = int(lidar.findtext("lidar/scan/vertical/samples"))
            update_rate = float(lidar.findtext("update_rate"))
            points_per_second = horizontal_samples * vertical_samples * update_rate
            self.assertGreaterEqual(points_per_second, 100_000)
            self.assertLessEqual(points_per_second, 300_000)

    def test_all_world_files_are_valid_sdf(self):
        worlds_dir = PACKAGE_ROOT / "resource" / "worlds"
        expected = {"rmuc_2024", "rmul_2024", "rmuc_2025", "rmul_2025"}
        actual = {path.stem.removesuffix("_world") for path in worlds_dir.glob("*_world.sdf")}
        self.assertEqual(actual, expected)
        for world in worlds_dir.glob("*_world.sdf"):
            root = ET.parse(world).getroot()
            self.assertEqual(root.tag, "sdf")

    def test_bridge_covers_clock_sensors_command_and_ground_truth(self):
        bridge_path = PACKAGE_ROOT / "config" / "ros_gz_bridge.yaml"
        self.assertTrue(bridge_path.is_file(), "bridge configuration is missing")
        entries = yaml.safe_load(bridge_path.read_text(encoding="utf-8"))
        ros_topics = {entry["ros_topic_name"] for entry in entries}
        self.assertTrue(
            {"/clock", "/sim/lidar/points", "/sim/imu", "/sim/cmd_vel", "/sim/ground_truth/odom"}
            <= ros_topics
        )

    def test_point_lio_simulation_uses_standard_cloud_and_sim_time(self):
        config_path = PACKAGE_ROOT / "config" / "point_lio_sim.yaml"
        self.assertTrue(config_path.is_file(), "Point-LIO simulation config is missing")
        params = yaml.safe_load(config_path.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
        self.assertTrue(params["use_sim_time"])
        self.assertEqual(params["common"]["lid_topic"], "/velodyne_points")
        self.assertEqual(params["common"]["imu_topic"], "/sim/imu")
        self.assertEqual(params["preprocess"]["lidar_type"], 2)
        self.assertEqual(params["preprocess"]["timestamp_unit"], 0)

    def test_world_catalog_references_packaged_worlds_and_maps(self):
        catalog = yaml.safe_load(
            (PACKAGE_ROOT / "config" / "worlds.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            set(catalog), {"rmuc_2024", "rmul_2024", "rmuc_2025", "rmul_2025"}
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
        self.assertEqual(controller["lidar_roll_offset"], 0.0)

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
                for world in ("rmuc_2024", "rmul_2024", "rmuc_2025", "rmul_2025"):
                    context = LaunchContext()
                    context.launch_configurations.update(
                        {
                            "world": world,
                            "chassis_type": chassis_type,
                            "headless": "true",
                            "rviz": "false",
                            "log_level": "info",
                        }
                    )
                    actions = module._launch_setup(context, str(PACKAGE_ROOT))
                    self.assertGreaterEqual(len(actions), 10)

    def test_pointcloud_adapter_preserves_simulation_contract(self):
        source_path = PACKAGE_ROOT / "src" / "pointcloud_adapter.cpp"
        self.assertTrue(source_path.is_file(), "pointcloud adapter source is missing")
        source = source_path.read_text(encoding="utf-8")
        for token in ["PointXYZIRT", "ring", "time", "SensorDataQoS", "UniquePtr"]:
            self.assertIn(token, source)


if __name__ == "__main__":
    unittest.main()

"""Static physical contracts for the standalone Ackermann asset (no ROS nodes)."""

from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET


MODELS = Path(__file__).resolve().parents[1] / "resource" / "models"


def numbers(element, path):
    return tuple(float(value) for value in element.findtext(path).split())


class AckermannModelTest(unittest.TestCase):
    def setUp(self):
        path = MODELS / "sentry_ackermann" / "model.sdf"
        self.assertTrue(path.is_file(), "standalone Ackermann model asset is missing")
        self.model = ET.parse(path).getroot().find("model")
        self.links = {link.get("name"): link for link in self.model.findall("link")}
        self.joints = {joint.get("name"): joint for joint in self.model.findall("joint")}

    def test_model_config_resolves_the_sdf_asset(self):
        config = ET.parse(MODELS / "sentry_ackermann" / "model.config").getroot()
        self.assertEqual(config.findtext("sdf"), "model.sdf")
        self.assertEqual(self.model.get("name"), "sentry")
        self.assertEqual(self.model.findtext("self_collide"), "false")

    def test_rear_axle_origin_matches_plugin_geometry_and_round_wheels(self):
        self.assertEqual(numbers(self.links["base_link"], "pose")[:2], (0.0, 0.0))
        for name, xy in {
            "rear_left_wheel": (0.0, 0.22), "rear_right_wheel": (0.0, -0.22),
            "front_left_wheel": (0.44, 0.22), "front_right_wheel": (0.44, -0.22),
        }.items():
            with self.subTest(wheel=name):
                wheel = self.links[name]
                pose = numbers(wheel, "pose")
                self.assertEqual(pose[:2], xy)
                self.assertAlmostEqual(pose[2], 0.076)
                self.assertAlmostEqual(pose[3], -math.pi / 2, places=8)
                cylinder = wheel.find("collision/geometry/cylinder")
                self.assertIsNotNone(cylinder, "a no-slip wheel cannot reuse the omni sphere")
                self.assertAlmostEqual(float(cylinder.findtext("radius")), 0.076)
                self.assertGreater(float(cylinder.findtext("length")), 0.0)
                self.assertIsNone(wheel.find("collision/surface/friction/ode/fdir1"))
        base = self.links["base_link"]
        for path in ("inertial/pose", "collision/pose", "visual/pose"):
            self.assertAlmostEqual(numbers(base, path)[0], 0.22)
        drive = self.model.find("plugin[@name='ignition::gazebo::systems::AckermannSteering']")
        self.assertIsNotNone(drive)
        for key in ("wheel_base", "wheel_separation", "kingpin_width"):
            self.assertAlmostEqual(float(drive.findtext(key)), 0.44)
        self.assertAlmostEqual(float(drive.findtext("wheel_radius")), 0.076)

    def test_front_wheels_roll_about_independent_steering_knuckles(self):
        for side in ("left", "right"):
            steer = self.joints[f"front_{side}_steering_joint"]
            roll = self.joints[f"front_{side}_joint"]
            self.assertEqual(steer.get("type"), "revolute")
            self.assertEqual(steer.findtext("parent"), "base_link")
            self.assertEqual(steer.findtext("child"), f"front_{side}_knuckle")
            self.assertEqual(numbers(steer, "axis/xyz"), (0.0, 0.0, 1.0))
            self.assertEqual(numbers(self.links[f"front_{side}_knuckle"], "pose")[3:], (0.0, 0.0, 0.0))
            self.assertEqual(roll.get("type"), "revolute")
            self.assertEqual(roll.findtext("parent"), f"front_{side}_knuckle")
            self.assertEqual(roll.findtext("child"), f"front_{side}_wheel")
            self.assertEqual(numbers(roll, "axis/xyz"), (0.0, 0.0, 1.0))
        for joint in self.joints.values():
            self.assertIn(joint.findtext("parent"), self.links)
            self.assertIn(joint.findtext("child"), self.links)

    def test_steering_limits_cover_inner_angle_without_a_planner_rate_bottleneck(self):
        drive = self.model.find("plugin[@name='ignition::gazebo::systems::AckermannSteering']")
        effective_center_max = math.atan(math.sin(float(drive.findtext("steering_limit"))))
        self.assertAlmostEqual(effective_center_max, 0.4, places=10)
        # Independent planar Ackermann geometry; these limits concern physical joints.
        t = math.tan(0.4)
        inner_angle = math.atan(t / (1.0 - 0.5 * t))
        for side in ("left", "right"):
            limit = self.joints[f"front_{side}_steering_joint"].find("axis/limit")
            lower, upper = float(limit.findtext("lower")), float(limit.findtext("upper"))
            velocity, effort = float(limit.findtext("velocity")), float(limit.findtext("effort"))
            self.assertLess(lower, -inner_angle)
            self.assertGreater(upper, inner_angle)
            self.assertLessEqual(upper, inner_angle + 0.05)
            self.assertAlmostEqual(lower, -upper)
            # This prototype has no independently calibrated steering actuator.
            # Keep the joint at least as responsive as Gazebo's official
            # Ackermann example instead of recreating a planner steering-rate
            # limit in the SDF and making commanded yaw lag for several seconds.
            self.assertGreaterEqual(velocity, 1.0)
            self.assertTrue(math.isfinite(effort) and effort > 0.0)

    def test_only_official_ackermann_owns_rear_drive_and_front_steering(self):
        actuator_plugins = [p for p in self.model.findall("plugin") if any(
            name in p.get("name", "") for name in
            ("Drive", "AckermannSteering", "JointController", "JointPositionController")
        )]
        self.assertEqual(len(actuator_plugins), 1, "each joint must have a single command owner")
        drive = actuator_plugins[0]
        self.assertEqual(drive.get("name"), "ignition::gazebo::systems::AckermannSteering")
        self.assertEqual(drive.findtext("left_joint"), "rear_left_joint")
        self.assertEqual(drive.findtext("right_joint"), "rear_right_joint")
        self.assertEqual(drive.findtext("left_steering_joint"), "front_left_steering_joint")
        self.assertEqual(drive.findtext("right_steering_joint"), "front_right_steering_joint")
        self.assertEqual(drive.findtext("topic"), "/sentry/cmd_vel")
        state = self.model.find("plugin[@name='ignition::gazebo::systems::JointStatePublisher']")
        self.assertIsNotNone(state)
        self.assertEqual({node.text for node in state.findall("joint_name")}, {
            "front_left_steering_joint", "front_right_steering_joint",
        })
        self.assertEqual(state.findtext("topic"), "/sentry/steering_joint_state")
        self.assertIsNone(state.find("update_rate"), "installed Gazebo 6.18 does not read update_rate")

    def test_sensor_geometry_and_imu_contract_match_omni(self):
        omni = ET.parse(MODELS / "sentry_omni" / "model.sdf").getroot().find("model")
        for name in ("sim_lidar", "sim_lidar_left", "sim_lidar_right"):
            original = omni.find(f"link[@name='{name}']")
            # Only the GPU sampling cadence/grid differs. Compare every other
            # sensor, mount, collision, FOV and range value, including the IMU.
            for link in (self.links[name], original):
                for sensor in link.findall("sensor[@type='gpu_lidar']"):
                    sensor.remove(sensor.find("update_rate"))
                    for axis in ("horizontal", "vertical"):
                        scan = sensor.find(f"lidar/scan/{axis}")
                        scan.remove(scan.find("samples"))
            self.assertEqual(ET.tostring(self.links[name]), ET.tostring(original))
        self.assertEqual(numbers(self.links["sim_lidar"], "pose")[:2], (0.0, -0.2))
        odometry = self.model.find("plugin[@name='ignition::gazebo::systems::OdometryPublisher']")
        self.assertIsNotNone(odometry)
        self.assertEqual(odometry.findtext("odom_topic"), "/sentry/ground_truth_odometry")
        self.assertEqual(odometry.findtext("robot_base_frame"), "sentry/base_link")
        self.assertEqual(odometry.findtext("dimensions"), "3")

    def test_ack_snapshot_cadence_keeps_the_omni_ray_budget(self):
        for model, expected in ((self.model, (10, 360, 120)), (
                ET.parse(MODELS / "sentry_omni" / "model.sdf").getroot().find("model"),
                (10, 360, 120))):
            sensors = model.findall(".//sensor[@type='gpu_lidar']")
            self.assertEqual(len(sensors), 4)
            rays_per_second = 0
            for sensor in sensors:
                with self.subTest(sensor=sensor.get("name"), cadence=expected[0]):
                    sampling = tuple(int(sensor.findtext(path)) for path in (
                        "update_rate", "lidar/scan/horizontal/samples",
                        "lidar/scan/vertical/samples"))
                    rays_per_second += math.prod(sampling)
                    self.assertEqual(sampling, expected)
                    for axis in ("horizontal", "vertical"):
                        self.assertEqual(float(sensor.findtext(f"lidar/scan/{axis}/resolution")), 1.0)
            self.assertEqual(rays_per_second, 1_728_000)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

from pathlib import Path
import unittest


SRC_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = SRC_ROOT / "scripts" / "simlation.bash"
PACKAGE_ROOT = SRC_ROOT / "simulation" / "sentry_simulation"


class SimulationShutdownContractTest(unittest.TestCase):
    def test_shell_entrypoint_forwards_one_shutdown_signal_to_launch(self):
        script = SCRIPT.read_text(encoding="utf-8")
        self.assertIn("launch_pid=", script)
        self.assertIn("trap forward_shutdown INT TERM", script)
        self.assertIn('kill -INT -- "-${launch_pid}"', script)
        self.assertIn('wait "${launch_pid}"', script)

    def test_python_nodes_treat_ros_context_shutdown_as_normal(self):
        for relative_path in (
            "sentry_simulation/imu_filter.py",
            "sentry_simulation/cmd_vel_adapter.py",
        ):
            source = (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
            self.assertIn("ExternalShutdownException", source)
            self.assertIn("RCLError", source)

if __name__ == "__main__":
    unittest.main()

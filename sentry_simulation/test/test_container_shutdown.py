"""Exercise production container-exit wiring with real launch processes."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from ament_index_python.packages import get_package_prefix
import unittest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_ROOT / 'launch/simulation.launch.py'


class ContainerShutdownTest(unittest.TestCase):
    def test_ordered_container_handles_startup_and_idle_signals(self):
        executable = (Path(get_package_prefix('sentry_simulation')) /
                      'lib/sentry_simulation/simulation_container')
        environment = dict(os.environ, ROS_DOMAIN_ID='195')
        for delay in (0, 0.15, 0.3):
            for shutdown_signal in (signal.SIGINT, signal.SIGTERM):
                with self.subTest(delay=delay, shutdown_signal=shutdown_signal):
                    process = subprocess.Popen([str(executable), '--ros-args', '-p', 'thread_num:=2'],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=environment)
                    try:
                        deadline = time.monotonic() + 3
                        mask = 0
                        expected = (1 << (signal.SIGINT - 1)) | (1 << (signal.SIGTERM - 1))
                        while time.monotonic() < deadline and process.poll() is None:
                            status = Path(f'/proc/{process.pid}/status').read_text()
                            mask = int(next(line.split()[1] for line in status.splitlines()
                                            if line.startswith('SigBlk:')), 16)
                            if mask & expected == expected:
                                break
                            time.sleep(0.005)
                        self.assertEqual(mask & expected, expected)
                        time.sleep(delay)
                        process.send_signal(shutdown_signal)
                        output, _ = process.communicate(timeout=3)
                        self.assertEqual(process.returncode, 0, output.decode())
                    finally:
                        if process.poll() is None:
                            process.kill()
                        process.communicate()

    def test_unexpected_container_exit_stops_other_processes(self):
        for returncode in (0, 23):
            with self.subTest(returncode=returncode), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                runner = root / 'runner.py'
                runner.write_text('''import runpy, sys
from pathlib import Path
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, RegisterEventHandler
setup = runpy.run_path(sys.argv[1])['_launch_setup']
container = ExecuteProcess(cmd=[sys.executable, '-c', 'import time;time.sleep(.3);raise SystemExit(' + sys.argv[2] + ')'])
sibling = ExecuteProcess(cmd=[sys.executable, '-c', 'import time;time.sleep(120)'])
# Substitute only the heavy executable; the real setup creates/registers its handler.
setup.__globals__['ComposableNodeContainer'] = lambda **kwargs: container
get_share = setup.__globals__['get_package_share_directory']
source = Path(sys.argv[3]).parents[1]
setup.__globals__['get_package_share_directory'] = lambda name: (
    str(source / 'navigation/navi2_bringup') if name == 'navi2' else get_share(name))
context = LaunchContext()
context.launch_configurations.update(world='home_indoor', chassis_type='ackermann',
    headless='true', use_icp='false', params_file='', rviz='false', log_level='info')
actions = setup(context, sys.argv[3])
handlers = [action for action in actions if isinstance(action, RegisterEventHandler)]
service = LaunchService(noninteractive=True)
service.include_launch_description(LaunchDescription([*handlers, sibling, container]))
raise SystemExit(service.run())
''')
                with (root/'output.log').open('w') as output:
                    process = subprocess.Popen([sys.executable, str(runner), str(LAUNCH_FILE), str(returncode), str(PACKAGE_ROOT)],
                        stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
                    try:
                        self.assertEqual(process.wait(timeout=8), 0,
                                         (root/'output.log').read_text())
                        self.assertIn('Simulation container exited unexpectedly',
                                      (root/'output.log').read_text())
                    finally:
                        try: os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError: pass
                        process.wait()


if __name__ == '__main__':
    unittest.main()

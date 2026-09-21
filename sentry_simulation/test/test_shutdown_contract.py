#!/usr/bin/env python3
"""Run the real shell entrypoint with a controlled ROS backend.

These tests catch inherited SIGINT ignoring and launchers that leave live
children behind. No Gazebo, ROS graph or installed overlay is modified.
"""
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import unittest

SRC_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = SRC_ROOT / 'scripts' / 'simlation.bash'
PACKAGE_ROOT = SRC_ROOT / 'simulation' / 'sentry_simulation'


class SimulationShutdownContractTest(unittest.TestCase):
    def run_wrapper(self, mode):
        with tempfile.TemporaryDirectory(prefix='simulation_shutdown_test_') as directory:
            root = Path(directory)
            (root / 'src/scripts').mkdir(parents=True)
            (root / 'src/navigation/navi2_bringup/launch').mkdir(parents=True)
            (root / 'src/navigation/navi2_bringup/params').mkdir(parents=True)
            (root / 'install').mkdir()
            (root / 'bin').mkdir()
            shutil.copy2(SCRIPT, root / 'src/scripts/simlation.bash')
            for relative in ('launch/navigation_parameters.py', 'params/navigation.yaml'):
                shutil.copy2(SRC_ROOT / 'navigation/navi2_bringup' / relative,
                             root / 'src/navigation/navi2_bringup' / relative)
            (root / 'install/setup.bash').write_text(
                'export PATH="' + str(root / 'bin') + ':$PATH"\n')
            backend = root / 'bin/ros2'
            backend.write_text('''#!/usr/bin/python3
import os, signal, subprocess, sys, time
from pathlib import Path
if sys.argv[1:3] == ['pkg', 'prefix']:
    sys.exit(0)
root = Path(os.environ['PROBE_ROOT'])
mode = os.environ['PROBE_MODE']
(root/'launcher.pid').write_text(str(os.getpid()))
(root/'ignored').write_text(str(signal.getsignal(signal.SIGINT) == signal.SIG_IGN))
if mode != 'normal':
    child = subprocess.Popen([sys.executable, '-c',
        "import signal,time;signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);print('ready',flush=True);time.sleep(120)"], stdout=subprocess.PIPE)
    child.stdout.readline()
    (root/'child.pid').write_text(str(child.pid))
if signal.getsignal(signal.SIGINT) != signal.SIG_IGN:
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
(root/'ready').touch()
if mode == 'orphan':
    sys.exit(23)
while True:
    time.sleep(.1)
''')
            backend.chmod(0o755)
            env = os.environ.copy()
            env.update(PROBE_ROOT=str(root), PROBE_MODE=mode)
            with (root / 'output.log').open('w') as output:
                wrapper = subprocess.Popen(['bash', str(root / 'src/scripts/simlation.bash'),
                    'omni', 'home_indoor'], env=env, stdout=output,
                    stderr=subprocess.STDOUT, start_new_session=True)
                launcher_pid = None
                try:
                    deadline = time.monotonic() + 8
                    while not (root/'ready').exists() and time.monotonic() < deadline:
                        if wrapper.poll() is not None:
                            break
                        time.sleep(.05)
                    self.assertTrue((root/'ready').exists(), (root/'output.log').read_text())
                    launcher_pid = int((root/'launcher.pid').read_text())
                    if mode != 'orphan':
                        os.kill(wrapper.pid, signal.SIGINT)
                    started = time.monotonic()
                    try:
                        status = wrapper.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        self.fail('simulation entrypoint did not exit within 12 seconds')
                    self.assertLess(time.monotonic()-started, 12)
                    self.assertEqual((root/'ignored').read_text(), 'False',
                                     'launch inherited ignored SIGINT')
                    if mode == 'orphan':
                        self.assertEqual(status, 23, 'launch failure status was lost')
                    for file in ('launcher.pid', 'child.pid'):
                        if (root/file).exists():
                            pid=int((root/file).read_text())
                            stat=Path(f'/proc/{pid}/stat')
                            self.assertTrue(not stat.exists() or
                                stat.read_text().rsplit(')',1)[1].split()[0] == 'Z',
                                f'live process {pid} remains after wrapper exit')
                finally:
                    if launcher_pid:
                        try: os.killpg(launcher_pid,signal.SIGKILL)
                        except ProcessLookupError: pass
                    if wrapper.poll() is None:
                        os.killpg(wrapper.pid,signal.SIGKILL)
                    wrapper.wait()

    def test_ctrl_c_reaches_launch(self):
        self.run_wrapper('normal')

    def test_stubborn_child_is_reaped_after_ctrl_c(self):
        self.run_wrapper('stubborn')

    def test_failed_launcher_cleans_remaining_children(self):
        self.run_wrapper('orphan')



if __name__ == '__main__':
    unittest.main()

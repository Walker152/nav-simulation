#!/usr/bin/env python3
"""Isolated real-Gazebo response regression; run explicitly, not during pytest.

Requires Gazebo Fortress's ign CLI only (Python standard library).
The input SDF is unmodified except removal of visuals/sensors and adding rear
joint observations. This catches slow steering, wrong reverse signs and a
wheel-only fix whose actual body motion still violates the bicycle contract.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid
import xml.etree.ElementTree as ET


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def prepare(model_path, directory):
    model = ET.parse(model_path).getroot().find('model')
    for parent in model.iter():
        for child in list(parent):
            if child.tag in ('visual', 'sensor'):
                parent.remove(child)
    publisher = next(p for p in model.findall('plugin')
                     if 'JointStatePublisher' in p.get('name', ''))
    for name in ('rear_left_joint', 'rear_right_joint'):
        ET.SubElement(publisher, 'joint_name').text = name
    root = ET.fromstring('''<sdf version="1.7"><world name="ack_regression">
      <gravity>0 0 -9.81</gravity><physics name="bench" type="ignored">
      <max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
      <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
      <model name="ground"><static>true</static><link name="ground"><collision name="ground">
      <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
      <surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface>
      </collision></link></model></world></sdf>''')
    root.find('world').append(model)
    ET.ElementTree(root).write(directory / 'world.sdf', encoding='unicode')


def analyze(directory, rows, events):
    failures, results = [], []
    joints, truth = rows['joints'], rows['truth']
    for index, event in enumerate(events):
        end = events[index + 1]['before'] if index + 1 < len(events) else joints[-1][0]
        sample = [r for r in joints if event['before'] <= r[0] < end]
        if not sample:
            raise RuntimeError('missing physical samples: ' + event['name'])
        # Detect execution from the first changed rear velocity, not CLI start.
        previous = [r for r in joints if r[0] < event['before']][-1]
        target_rear = event['rear']
        jump = max(abs(target_rear[i] - previous[3 + i]) for i in (0, 1))
        changed = [r for r in sample if max(abs(r[3+i] - previous[3+i])
                   for i in (0, 1)) > max(.02, .1 * jump)]
        t0 = changed[0][0] if jump > .05 and changed else event['after']
        if jump > .05 and not changed:
            raise RuntimeError('command produced no rear response: ' + event['name'])
        near = min(sample, key=lambda r: abs(r[0] - (t0 + .05)))
        wheel_error = max(abs(near[i+1] - event['angles'][i]) for i in (0, 1))
        body = [r for r in truth if t0 <= r[0] < end]
        if not body:
            raise RuntimeError('missing groundtruth: ' + event['name'])
        response = min(body, key=lambda r: abs(r[0] - (t0 + .15)))
        yaw_error = abs(response[3] - event['w'])
        steady = [r for r in body if end - .5 <= r[0]]
        vx = sum(r[1] for r in steady) / len(steady)
        vy = max(abs(r[2]) for r in steady)
        checks = {'wheel_50ms': wheel_error <= .005,
                  'body_150ms': yaw_error <= max(.03, .15 * abs(event['w'])),
                  'direction_150ms': abs(event['w']) < .03 or response[3] * event['w'] > 0,
                  'steady_speed': abs(vx-event['v']) <= max(.015, .05*abs(event['v'])),
                  'steady_lateral': vy <= .02}
        a = min(body, key=lambda r: abs(r[0] - (end - 1.1)))
        b = min(body, key=lambda r: abs(r[0] - (a[0] + 1)))
        dt, v, w = b[0]-a[0], event['v'], event['w']
        dx = v*dt if abs(w) < 1e-10 else v*math.sin(w*dt)/w
        dy = 0 if abs(w) < 1e-10 else v*(1-math.cos(w*dt))/w
        x = a[4] + math.cos(a[6])*dx - math.sin(a[6])*dy
        y = a[5] + math.sin(a[6])*dx + math.cos(a[6])*dy
        position_error = math.hypot(b[4]-x, b[5]-y)
        heading_error = abs(math.remainder(b[6]-a[6]-w*dt, 2*math.pi))
        checks['arc_position'] = position_error <= .04
        checks['arc_heading'] = heading_error <= .05
        result = dict(event, execution=t0, wheel_error_50ms=wheel_error,
                      yaw_error_150ms=yaw_error, steady_vx=vx, steady_max_vy=vy,
                      arc_position_error=position_error, arc_heading_error=heading_error,
                      checks=checks)
        results.append(result)
        failures.extend(event['name'] + ': ' + name for name, ok in checks.items() if not ok)
    report = {'results': results, 'failures': failures}
    (directory/'summary.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=Path(__file__).resolve().parents[1] /
                        'resource/models/sentry_ackermann/model.sdf')
    parser.add_argument('--output', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--smoke', action='store_true', help='only verify straight driving and transport')
    mode.add_argument('--nonfinite', action='store_true',
                      help='separate moving-to-NaN/Inf stop cases; do not run on the old plugin')
    args = parser.parse_args()
    directory = args.output or Path(tempfile.mkdtemp(prefix='ack_physics_'))
    directory.mkdir(parents=True, exist_ok=True)
    partition = 'ack_regression_' + uuid.uuid4().hex
    env = dict(os.environ, IGN_PARTITION=partition)
    prepare(args.model, directory)
    (directory/'manifest.json').write_text(json.dumps({'partition': partition,
        'model': str(args.model.resolve()), 'physics_step': .001}, indent=2))
    rows = {'joints': [], 'truth': []}
    lock, processes, threads, files = threading.Lock(), [], [], []

    def consume(name, topic):
        stderr = open(directory/(name+'.stderr'), 'w')
        files.append(stderr)
        process = subprocess.Popen(['stdbuf', '-oL', 'ign', 'topic', '-t', topic,
            '-e', '--json-output'], env=env, stdout=subprocess.PIPE, stderr=stderr,
            text=True, start_new_session=True)
        processes.append(process)

        def read():
            with open(directory/(name+'.csv'), 'w') as output:
                writer = csv.writer(output)
                writer.writerow(['sim', 'left', 'right', 'rear_left_rate', 'rear_right_rate']
                                if name == 'joints' else ['sim', 'vx', 'vy', 'w', 'x', 'y', 'yaw'])
                for line in process.stdout:
                    try:
                        message = json.loads(line)
                    except ValueError:
                        continue
                    stamp = message['header']['stamp']
                    t = int(stamp.get('sec', 0)) + int(stamp.get('nsec', 0))*1e-9
                    if name == 'joints':
                        j = {x['name']: x['axis1'] for x in message['joint']}
                        row = [t, j['front_left_steering_joint'].get('position', 0),
                               j['front_right_steering_joint'].get('position', 0),
                               j['rear_left_joint'].get('velocity', 0),
                               j['rear_right_joint'].get('velocity', 0)]
                    else:
                        pose, twist = message['pose'], message['twist']
                        q = pose['orientation']
                        yaw = math.atan2(2*(q.get('w', 0)*q.get('z', 0)+q.get('x', 0)*q.get('y', 0)),
                                         1-2*(q.get('y', 0)**2+q.get('z', 0)**2))
                        row = [t, twist['linear'].get('x', 0), twist['linear'].get('y', 0),
                               twist['angular'].get('z', 0), pose['position'].get('x', 0),
                               pose['position'].get('y', 0), yaw]
                    with lock:
                        rows[name].append(row)
                    writer.writerow(row)
        thread = threading.Thread(target=read, daemon=True)
        threads.append(thread)
        thread.start()

    def simtime():
        with lock:
            return rows['joints'][-1][0] if rows['joints'] else None

    def wait_sim(duration):
        begin = simtime()
        deadline = time.monotonic() + duration*4 + 10
        while simtime()-begin < duration:
            if time.monotonic() > deadline:
                raise RuntimeError('simulation no longer advancing')
            time.sleep(.02)

    events = []
    try:
        log = open(directory/'gazebo.log', 'w')
        files.append(log)
        server = subprocess.Popen(['ign', 'gazebo', '-s', '-r', '-v', '2', str(directory/'world.sdf')],
            env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(server)
        consume('joints', '/sentry/steering_joint_state')
        consume('truth', '/sentry/ground_truth_odometry')
        deadline = time.monotonic()+15
        while simtime() is None:
            if time.monotonic() > deadline:
                raise RuntimeError('no joint observations; inspect gazebo.log (not behavioral RED)')
            time.sleep(.05)
        wait_sim(.2)
        stages = [('straight', .6, 0, 2), ('forward_left', .6, .4, 6),
                  ('forward_weak_right', .6, -.05, 3), ('forward_left_again', .6, .4, 6),
                  ('forward_strong_right', .6, -.4, 3), ('stop_before_reverse', 0, 0, 2),
                  ('reverse_straight', -.6, 0, 3),
                  ('reverse_left', -.6, .4, 6), ('reverse_weak_right', -.6, -.05, 3),
                  ('reverse_left_again', -.6, .4, 6), ('reverse_strong_right', -.6, -.4, 3),
                  ('stop', 0, 0, 2), ('pure_yaw', 0, 0, 2), ('near_zero', 1e-6, .4, 2)]
        if args.smoke:
            stages = stages[:1]
        elif args.nonfinite:
            stages = [('before_nan', .6, .4, 2), ('nan_stop', 0, 0, 2),
                      ('before_inf', -.6, -.4, 2), ('inf_stop', 0, 0, 2)]
        for name, v, delta, duration in stages:
            w = v*math.tan(delta)/.44
            before = simtime()
            command_w = .4 if name == 'pure_yaw' else w
            command_v = 'nan' if name == 'nan_stop' else v
            if name == 'inf_stop':
                command_w = 'inf'
            tangent = math.tan(delta)
            rear = [v*(1-.5*tangent)/.076, v*(1+.5*tangent)/.076]
            # Fortress CLI is one-shot; retry only after the previous writer
            # exits. Rear response acknowledges delivery independently of the
            # front-angle assertion, so the old slow plant can reach RED.
            for attempt in range(3):
                publisher = subprocess.Popen(['ign', 'topic', '-t', '/sentry/cmd_vel',
                    '-m', 'ignition.msgs.Twist', '-p',
                    f'linear: {{x: {command_v}}} angular: {{z: {command_w}}}'], env=env,
                    stdout=subprocess.DEVNULL, stderr=log, start_new_session=True)
                processes.append(publisher)
                publisher.wait(timeout=5)
                if publisher.returncode:
                    raise RuntimeError('ign publisher failed; not behavioral RED')
                wait_sim(.1)
                with lock:
                    measured = rows['joints'][-1][3:5]
                if max(abs(measured[i]-rear[i]) for i in (0, 1)) < .03:
                    break
            else:
                raise RuntimeError('no expected rear response after three publishes')
            events.append({'name': name, 'v': v, 'w': w, 'sent_v': command_v,
                'sent_w': command_w, 'before': before, 'after': simtime(),
                'angles': [math.atan(tangent/(1-.5*tangent)), math.atan(tangent/(1+.5*tangent))],
                'rear': rear})
            (directory/'events.json').write_text(json.dumps(events, indent=2))
            print('stage', name, 'sim', simtime(), flush=True)
            wait_sim(duration)
            stop(publisher)
    finally:
        for process in reversed(processes):
            stop(process)
        for thread in threads:
            thread.join(timeout=3)
        for file in files:
            file.close()
        (directory/'cleanup.json').write_text(json.dumps([
            {'pid': p.pid, 'returncode': p.poll()} for p in processes], indent=2))
    return analyze(directory, rows, events)


if __name__ == '__main__':
    raise SystemExit(main())

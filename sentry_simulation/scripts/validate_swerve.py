#!/usr/bin/env python3
"""Manual integration probe for an already running, isolated swerve_odin launch.

Run with matching ROS_DOMAIN_ID. Motion mode takes about one simulated minute;
use the default YAML and sensors:=false for repeatable dynamics measurements.
Never use on hardware.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, PointCloud2, Imu, Image, CameraInfo
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


def stamp(msg):
    t = msg.clock if isinstance(msg, Clock) else msg.header.stamp
    return t.sec + t.nanosec * 1e-9


def twist(msg):
    t = msg.twist.twist if isinstance(msg, Odometry) else msg.twist
    return (t.linear.x, t.linear.y, t.angular.z)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('motion', 'sensors'), default='sensors')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('swerve_validation')
    latest, history, frames = {}, {}, {}
    def receive(key, msg):
        latest[key] = msg
        history.setdefault(key, []).append((stamp(msg), twist(msg) if isinstance(msg, (Odometry, TwistStamped))
                                               else (list(msg.velocity) if isinstance(msg, JointState) else None)))
    def subscribe(key, cls, topic):
        node.create_subscription(cls, topic, lambda msg: receive(key, msg), qos_profile_sensor_data)
    subscribe('clock', Clock, '/clock')
    subscribe('truth', Odometry, '/swerve/ground_truth/odometry')
    subscribe('wheel', Odometry, '/swerve/wheel_odometry')
    subscribe('actual_joints', JointState, '/swerve/joint_states')
    subscribe('targets', JointState, '/swerve/joint_targets')
    subscribe('reference', TwistStamped, '/swerve/reference_twist')
    def tf_callback(msg):
        frames.update({t.child_frame_id: t for t in msg.transforms})
    node.create_subscription(TFMessage, '/tf', tf_callback, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf_static', tf_callback,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin_until(predicate, wall_timeout=30):
        end = time.monotonic() + wall_timeout
        while time.monotonic() < end and not predicate():
            rclpy.spin_once(node, timeout_sec=.02)
        assert predicate(), f'Timed out: received {list(latest)}'
    report = {'mode': args.mode}
    try:
        if args.mode == 'sensors':
            for direction in ('front', 'rear', 'left', 'right'):
                for key, cls, suffix in (('points', PointCloud2, 'points'), ('imu', Imu, 'imu'),
                                        ('rgb', Image, 'rgb/image'), ('info', CameraInfo, 'rgb/camera_info')):
                    subscribe(direction+'_'+key, cls, '/sim/odin_'+direction+'/'+suffix)
            expected = {d+'_'+k for d in ('front', 'rear', 'left', 'right') for k in ('points', 'imu', 'rgb', 'info')}
            spin_until(lambda: expected <= latest.keys(), 90)
            start = stamp(latest['clock'])
            spin_until(lambda: stamp(latest['clock']) - start >= 1, 90)
            report['sensors'] = {}
            for direction in ('front', 'rear', 'left', 'right'):
                values = {}
                for key in ('points', 'imu', 'rgb', 'info'):
                    full = direction+'_'+key
                    msg = latest[full]
                    times = [t for t, _ in history[full]]
                    values[key] = {'frame': msg.header.frame_id, 'count': len(times),
                                   'median_sim_dt': float(np.median(np.diff(times))) if len(times)>1 else None}
                    wanted = 'body' if key == 'imu' else ('lidar' if key == 'points' else 'camera_optical')
                    assert msg.header.frame_id == 'odin_'+direction+'_'+wanted
                    assert msg.header.frame_id in frames
                cloud = latest[direction+'_points']
                offsets = {f.name: f.offset for f in cloud.fields}
                points = np.stack([np.ndarray((cloud.height, cloud.width), dtype='<f4', buffer=cloud.data,
                    offset=offsets[a], strides=(cloud.row_step, cloud.point_step)) for a in ('x','y','z')], axis=-1)
                values['points'].update(size=[cloud.width, cloud.height], finite_fraction=float(np.isfinite(points).all(axis=-1).mean()))
                assert (cloud.width, cloud.height) == (240, 180)
                assert values['points']['finite_fraction'] > .05
                rgb, info = latest[direction+'_rgb'], latest[direction+'_info']
                values['rgb']['size'] = [rgb.width, rgb.height]
                assert (rgb.width, rgb.height) == (1280, 1088)
                assert (info.width, info.height) == (rgb.width, rgb.height)
                assert info.k[0] > 0 and len(rgb.data) >= rgb.step*rgb.height
                values['imu']['acceleration'] = [getattr(latest[direction+'_imu'].linear_acceleration,a) for a in ('x','y','z')]
                report['sensors'][direction] = values
            report['tf'] = {k: v.header.frame_id for k,v in frames.items()}
            assert frames['base_link'].header.frame_id == 'odom'
        else:
            spin_until(lambda: {'clock','truth','actual_joints','targets','reference','wheel'} <= latest.keys())
            # Guard this motion probe against accidental hardware use.
            assert len(latest['actual_joints'].name) == 8 and 'front_left_steer_joint' in latest['actual_joints'].name
            command_topic = '/kinco_swerve/cmd_vel/selected'
            assert node.count_publishers(command_topic) == 0, 'selected already has a command publisher'
            truth_owners = node.get_publishers_info_by_topic('/swerve/ground_truth/odometry')
            assert len(truth_owners) == 1 and truth_owners[0].node_name == 'swerve_odin_bridge', \
                'Expected the independent swerve_odin simulation bridge'
            publisher = node.create_publisher(TwistStamped, command_topic, 10)
            def segment(name, command, duration, invalid=False, publish=True):
                begin = stamp(latest['clock'])
                first = len(history['truth'])
                last_publish = 0
                deadline = time.monotonic() + max(20, duration*6)
                while stamp(latest['clock']) - begin < duration:
                    assert time.monotonic() < deadline, name+' simulation did not advance'
                    assert node.count_publishers(command_topic) <= 1, 'A second selected command publisher appeared'
                    if publish and time.monotonic()-last_publish >= .02:
                        msg = TwistStamped()
                        if not invalid:
                            msg.header.stamp = latest['clock'].clock
                        msg.header.frame_id = 'base_link'
                        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z = command
                        publisher.publish(msg)
                        last_publish = time.monotonic()
                    rclpy.spin_once(node, timeout_sec=.005)
                data = [(t, v) for t,v in history['truth'][first:] if t-begin > duration-1]
                measured = np.mean([v for _,v in data],axis=0)
                ref = np.array(twist(latest['reference']))
                result = {'start_sim_time':begin, 'end_sim_time':stamp(latest['clock']),
                          'normal_command': publish and not invalid, 'command':command, 'mean_actual_last_second':measured.tolist(), 'last_reference':ref.tolist(),
                          'z':latest['truth'].pose.pose.position.z,
                          'actual_joints':dict(zip(latest['actual_joints'].name, latest['actual_joints'].velocity)),
                          'target_velocity':dict(zip(latest['targets'].name, latest['targets'].velocity))}
                print(name, json.dumps(result), flush=True)
                report.setdefault('segments',{})[name] = result
                return measured
            segment('settle', (0.,0.,0.), 2)
            cases = [('forward_max',(2.,0.,0.),5), ('reverse',(-.5,0.,0.),4),
                     ('left',(0.,.5,0.),4), ('right',(0.,-.5,0.),4),
                     ('diagonal',(.4,.3,0.),4), ('yaw_max',(0.,0.,2.),5),
                     ('combined',(.5,.2,.6),5)]
            errors = []
            for name, cmd, duration in cases:
                actual = segment(name, cmd, duration)
                if np.linalg.norm(actual[:2]-cmd[:2]) > .2 or abs(actual[2]-cmd[2]) > .3:
                    errors.append(name+' velocity tracking')
                stop = segment(name+'_normal_stop', (0.,0.,0.), 3)
                if np.linalg.norm(stop) > .12:
                    errors.append(name+' normal stop')
            segment('before_invalid', (.6,0.,0.),3)
            invalid_stop = segment('invalid_zero', (0.,0.,0.),2,invalid=True)
            segment('before_timeout', (.6,0.,0.),3)
            timeout_stop = segment('timeout', (0.,0.,0.),2,publish=False)
            if np.linalg.norm(invalid_stop) > .12 or np.linalg.norm(timeout_stop) > .12:
                errors.append('invalid/timeout stopping')
            report['errors'] = errors
            truth_samples = history['truth']
            velocities = np.array([v for _,v in truth_samples]); times = np.array([t for t,_ in truth_samples])
            mask = np.diff(times)>1e-6
            acceleration = np.diff(velocities,axis=0)[mask]/np.diff(times)[mask,None]
            report['observed_max_actual'] = {'linear_speed':float(np.linalg.norm(velocities[:,:2],axis=1).max()),
                'yaw_speed':float(abs(velocities[:,2]).max()),
                'linear_acceleration':float(np.linalg.norm(acceleration[:,:2],axis=1).max()),
                'yaw_acceleration':float(abs(acceleration[:,2]).max())}
            normal_intervals = [(s['start_sim_time'], s['end_sim_time']) for s in report['segments'].values()
                                if s['normal_command']]
            # Only ordinary fresh commands are subject to smooth reference limits;
            # invalidation intentionally stops targets immediately.
            for key, indices in (('reference', (0, 1, 2)), ('targets', (0, 2, 4, 6))):
                samples = history[key]
                ts = np.array([t for t, _ in samples])
                values = np.array([v for _, v in samples])[:, indices]
                delta = np.diff(ts)
                ordinary = np.array([any(a <= t0 and t1 <= b for a, b in normal_intervals)
                                     for t0, t1 in zip(ts[:-1], ts[1:])]) & (delta > 1e-6)
                slopes = np.diff(values, axis=0)[ordinary] / delta[ordinary, None]
                if key == 'reference':
                    report['observed_reference_limits'] = {
                        'linear_speed':float(np.linalg.norm(values[:, :2],axis=1).max()),
                        'yaw_speed':float(abs(values[:, 2]).max()),
                        'normal_linear_acceleration':float(np.linalg.norm(slopes[:, :2],axis=1).max()),
                        'normal_yaw_acceleration':float(abs(slopes[:, 2]).max())}
                    assert report['observed_reference_limits']['normal_linear_acceleration'] <= 1.00001
                    assert report['observed_reference_limits']['normal_yaw_acceleration'] <= 2.00001
                else:
                    report['observed_steering_reference'] = {
                        'max_velocity':float(abs(values).max()),
                        'normal_max_acceleration':float(abs(slopes).max())}
                    assert report['observed_steering_reference']['max_velocity'] <= 2*math.pi+1e-5
                    assert report['observed_steering_reference']['normal_max_acceleration'] <= 4*math.pi+1e-5
            for key in ('actual_joints','truth','reference'):
                report[key+'_median_sim_dt'] = float(np.median(np.diff([t for t,_ in history[key]])))
            assert not errors, errors
        report['result'] = 'PASS'
    except Exception as error:
        report['result'] = 'FAIL'
        report['error'] = str(error)
        raise
    finally:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(report,indent=2))
        node.destroy_node()
        rclpy.shutdown()
        print('Report:',args.output,report['result'],flush=True)


if __name__ == '__main__':
    main()

"""Generate one coherent SDF/URDF/bridge set from the platform parameters.

The canonical O1LITE asset owns the Odin optics, extrinsics and mass. Optional
MID360 mounts and sampling frequency come from the hardware input configuration.
"""
from sentry_simulation.gazebo_compat import gazebo_text
from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

MODULES = ('front_left', 'front_right', 'rear_left', 'rear_right')
DIRECTIONS = ('front', 'rear', 'left', 'right')


def element(parent, tag, text=None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    if text is not None:
        child.text = str(text)
    return child


def numbers(values):
    return ' '.join(f'{v:.12g}' for v in values)


def box(size):
    geometry = ET.Element('geometry')
    element(element(geometry, 'box'), 'size', numbers(size))
    return geometry


def cylinder(radius, length):
    geometry = ET.Element('geometry')
    shape = element(geometry, 'cylinder')
    element(shape, 'radius', radius)
    element(shape, 'length', length)
    return geometry


def visual(link, name, geometry, pose, color):
    node = element(link, 'visual', name=name)
    element(node, 'pose', numbers(pose))
    node.append(deepcopy(geometry))
    material = element(node, 'material')
    for tag in ('ambient', 'diffuse'):
        element(material, tag, color)


def physical_link(model, name, parent, pose, mass, moments, shape, shape_pose, color,
                  dynamics):
    link = element(model, 'link', name=name)
    element(link, 'pose', numbers(pose), relative_to=parent)
    inertial = element(link, 'inertial')
    # Moments are expressed in the link frame, with the COM at shape translation.
    element(inertial, 'pose', numbers((*shape_pose[:3], 0, 0, 0)))
    element(inertial, 'mass', mass)
    inertia = element(inertial, 'inertia')
    for key, value in zip(('ixx', 'iyy', 'izz'), moments):
        element(inertia, key, value)
    collision = element(link, 'collision', name=name + '_collision')
    element(collision, 'pose', numbers(shape_pose))
    collision.append(deepcopy(shape))
    surface = element(collision, 'surface')
    friction = element(element(surface, 'friction'), 'ode')
    element(friction, 'mu', dynamics['friction'])
    element(friction, 'mu2', dynamics['friction'])
    contact = element(element(surface, 'contact'), 'ode')
    element(contact, 'kp', dynamics['contact_stiffness'])
    element(contact, 'kd', dynamics['contact_damping'])
    visual(link, name + '_visual', shape, shape_pose, color)
    return link


def box_inertia(mass, size):
    x, y, z = size
    return (mass * (y*y + z*z) / 12, mass * (x*x + z*z) / 12,
            mass * (x*x + y*y) / 12)


def joint(model, name, parent, child, axis=None, lower=None, upper=None,
          velocity=None, effort=None, damping=0):
    node = element(model, 'joint', name=name, type='revolute' if axis else 'fixed')
    element(node, 'parent', parent)
    element(node, 'child', child)
    if axis:
        a = element(node, 'axis')
        element(a, 'xyz', axis)
        limit = element(a, 'limit')
        # SDF uses unlimited revolute; URDF below exposes it as continuous.
        if lower is not None:
            element(limit, 'lower', lower)
            element(limit, 'upper', upper)
        element(limit, 'velocity', velocity)
        element(limit, 'effort', effort)
        element(element(a, 'dynamics'), 'damping', damping)
    return node


def to_urdf(model):
    """Translate our explicit parent-relative poses; never infer another geometry."""
    robot = ET.Element('robot', name=model.get('name'))
    links = {link.get('name'): link for link in model.findall('link')}
    for name, source in links.items():
        link = element(robot, 'link', name=name)
        for source_visual in source.findall('visual'):
            v = element(link, 'visual', name=source_visual.get('name'))
            pose = source_visual.findtext('pose', '0 0 0 0 0 0').split()
            element(v, 'origin', xyz=' '.join(pose[:3]), rpy=' '.join(pose[3:]))
            geometry = element(v, 'geometry')
            shape = source_visual.find('geometry')[0]
            if shape.tag == 'box':
                element(geometry, 'box', size=shape.findtext('size'))
            elif shape.tag == 'cylinder':
                element(geometry, 'cylinder', radius=shape.findtext('radius'), length=shape.findtext('length'))
            elif shape.tag == 'mesh':
                uri = shape.findtext('uri').replace('model://', 'package://sentry_simulation/resource/models/')
                element(geometry, 'mesh', filename=uri)
            material = element(v, 'material', name=source_visual.get('name') + '_material')
            element(material, 'color', rgba=source_visual.findtext('material/diffuse', '0.5 0.5 0.5 1'))
    for source in model.findall('joint'):
        child = source.findtext('child')
        unlimited = source.find('axis') is not None and source.find('axis/limit/lower') is None
        j = element(robot, 'joint', name=source.get('name'),
                    type='continuous' if unlimited else source.get('type'))
        element(j, 'parent', link=source.findtext('parent'))
        element(j, 'child', link=child)
        pose = links[child].findtext('pose', '0 0 0 0 0 0').split()
        element(j, 'origin', xyz=' '.join(pose[:3]), rpy=' '.join(pose[3:]))
        if source.find('axis') is not None:
            element(j, 'axis', xyz=source.findtext('axis/xyz'))
            limit = source.find('axis/limit')
            element(j, 'limit', **{e.tag: e.text for e in limit})
    for frame in model.findall('frame'):
        name = frame.get('name')
        element(robot, 'link', name=name)
        j = element(robot, 'joint', name=name + '_fixed', type='fixed')
        element(j, 'parent', link=frame.get('attached_to'))
        element(j, 'child', link=name)
        pose = frame.findtext('pose').split()
        element(j, 'origin', xyz=' '.join(pose[:3]), rpy=' '.join(pose[3:]))
    return robot


def mid360_pose(config, key):
    """Read a canonical T_parent_child pose, converting degrees to radians."""
    try:
        position, angles = (config[key][name] for name in ('xyz_m', 'rpy_deg'))
        values = (*position, *angles)
        if len(position) != 3 or len(angles) != 3 or not all(
                type(v) in (int, float) and math.isfinite(v) for v in values):
            raise ValueError('expected three finite xyz_m and rpy_deg values')
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f'Invalid MID360 {key}: {error}') from error
    return (*position, *(math.radians(v) for v in angles))


def inverse_pose(pose):
    """Invert an XYZ/RPY rigid transform using R=Rz(yaw)Ry(pitch)Rx(roll)."""
    x, y, z, roll, pitch, yaw = pose
    sr, cr, sp, cp, sy, cy = (math.sin(roll), math.cos(roll), math.sin(pitch),
                             math.cos(pitch), math.sin(yaw), math.cos(yaw))
    rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
                (-sp, cp*sr, cp*cr))
    inverse = tuple(zip(*rotation))
    translation = tuple(-sum(row[i]*v for i, v in enumerate((x, y, z))) for row in inverse)
    horizontal = math.hypot(inverse[0][0], inverse[1][0])
    inverse_pitch = math.atan2(-inverse[2][0], horizontal)
    if horizontal > 1e-12:
        inverse_roll = math.atan2(inverse[2][1], inverse[2][2])
        inverse_yaw = math.atan2(inverse[1][0], inverse[0][0])
    else:
        inverse_roll = math.atan2(-inverse[1][2], inverse[1][1])
        inverse_yaw = 0.0
    return (*translation, inverse_roll, inverse_pitch, inverse_yaw)


def add_mid360(model, role, pose, frequency, dynamics, sensors, imu_pose=None):
    """Mount one physical device; both scan halves share the calibrated light origin."""
    name = 'sim_mid360_' + role
    size, mass = (.065, .060, .065), .265  # Existing MID360 simulation assumptions.
    link = physical_link(model, name, 'base_link', pose, mass, box_inertia(mass, size),
                         box(size), (0, 0, -.03, 0, 0, 0), '0.12 0.14 0.16 1', dynamics)
    link.remove(link.find('visual'))
    mesh = ET.Element('geometry')
    element(element(mesh, 'mesh'), 'uri', 'model://mid360/meshes/mid360.dae')
    visual(link, name + '_visual', mesh, (0, 0, -.03, 0, 0, math.pi), '0.12 0.14 0.16 1')
    joint(model, name + '_mount', 'base_link', name)
    if imu_pose is not None:
        frame = element(model, 'frame', name=name + '_imu', attached_to=name)
        element(frame, 'pose', numbers(imu_pose), relative_to=name)
    if not sensors:
        return
    for half, yaw in (('front', 0), ('rear', math.pi)):
        sensor = element(link, 'sensor', name=name + '_' + half, type='gpu_lidar')
        element(sensor, 'pose', numbers((0, 0, 0, 0, 0, yaw)))
        element(sensor, 'topic', f'/sim/mid360/{role}/{half}')
        element(sensor, 'update_rate', frequency)
        element(sensor, 'always_on', 'true')
        element(sensor, 'visualize', 'false')
        element(sensor, 'gz_frame_id', name)
        lidar = element(sensor, 'lidar')
        scan = element(lidar, 'scan')
        for axis, count, minimum, maximum in (
                ('horizontal', 360, -math.pi/2, math.pi/2),
                ('vertical', 120, math.radians(-7.3), math.radians(52.3))):
            axis_node = element(scan, axis)
            for tag, value in (('samples', count), ('resolution', 1),
                               ('min_angle', minimum), ('max_angle', maximum)):
                element(axis_node, tag, value)
        limits = element(lidar, 'range')
        for tag, value in (('min', .1), ('max', 40.0), ('resolution', .01)):
            element(limits, tag, value)
        noise = element(lidar, 'noise')
        for tag, value in (('type', 'gaussian'), ('mean', 0.0), ('stddev', 0.0)):
            element(noise, tag, value)
    if imu_pose is not None:
        imu = element(link, 'sensor', name=name + '_imu', type='imu')
        element(imu, 'pose', numbers(imu_pose))
        element(imu, 'topic', '/sim/mid360/primary/imu')
        element(imu, 'update_rate', 200)
        element(imu, 'always_on', 'true')
        element(imu, 'gz_frame_id', name + '_imu')


def bridge_config(sensors, publish_ground_truth_tf=True, mid360=False):
    entries = []
    def bridge(ros, gz, ros_type, gz_type, direction='GZ_TO_ROS'):
        entries.append(dict(ros_topic_name=ros, gz_topic_name=gz,
                            ros_type_name=ros_type, gz_type_name='gz.msgs.' + gz_type,
                            direction=direction))
    bridge('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'Clock')
    bridge('/kinco_swerve/cmd_vel/selected', '/kinco_swerve/cmd_vel/selected',
           'geometry_msgs/msg/TwistStamped', 'Twist', 'ROS_TO_GZ')
    for name in ('joint_states', 'joint_targets'):
        bridge('/swerve/' + name, '/swerve/' + name, 'sensor_msgs/msg/JointState', 'Model')
    bridge('/swerve/reference_twist', '/swerve/reference_twist', 'geometry_msgs/msg/TwistStamped', 'Twist')
    for name in ('wheel_odometry', 'ground_truth/odometry'):
        bridge('/swerve/' + name, '/swerve/' + name, 'nav_msgs/msg/Odometry', 'Odometry')
    if publish_ground_truth_tf:
        bridge('/tf', '/swerve/ground_truth/tf', 'tf2_msgs/msg/TFMessage', 'Pose_V')
    if sensors and mid360:
        for role in ('primary', 'secondary'):
            for half in ('front', 'rear'):
                topic = f'/sim/mid360/{role}/{half}/points'
                bridge(topic, topic, 'sensor_msgs/msg/PointCloud2', 'PointCloudPacked')
        bridge('/sim/mid360/primary/imu', '/sim/mid360/primary/imu', 'sensor_msgs/msg/Imu', 'IMU')
    elif sensors:
        for direction in DIRECTIONS:
            prefix = '/sim/odin_' + direction
            for ros, gz, ros_type, gz_type in (
                ('/points', '/lidar/points', 'PointCloud2', 'PointCloudPacked'),
                ('/imu', '/imu', 'Imu', 'IMU'),
                ('/rgb/image', '/rgb/image', 'Image', 'Image'),
                ('/rgb/camera_info', '/rgb/camera_info', 'CameraInfo', 'CameraInfo')):
                bridge(prefix + ros, prefix + gz, 'sensor_msgs/msg/' + ros_type, gz_type)
                # Unused RGB streams otherwise keep four full-resolution cameras rendering.
                if ros_type in ('Image', 'CameraInfo'):
                    entries[-1]['lazy'] = True
    return entries


def build_assets(config, share, output, sensors=True, world_path=None, spawn=None,
                 publish_ground_truth_tf=True, ground_truth_frame='odom', mid360_config=None):
    """Generate resources; spawn.z is ground height, before wheel clearance.

    Localization may own the ROS base transform while ground-truth odometry
    remains available for simulated device inputs and diagnostics. Navigation
    uses a distinct world label for these poses when odom has a local origin.
    """
    if (not isinstance(ground_truth_frame, str) or not ground_truth_frame.strip()
            or ground_truth_frame == 'base_link'):
        raise ValueError('ground_truth_frame must be nonempty and distinct from base_link')
    g, d, c, p = (config[k] for k in ('geometry', 'dynamics', 'control', 'physics'))
    signed = {'body_center_z', 'wheel_center_z', 'sensor_height', 'spindle_center_z'}
    for section in (g, d, c, p):
        for key, value in section.items():
            if not isinstance(value, (int, float)) or not math.isfinite(value) or (key not in signed and value <= 0):
                raise ValueError(f'{key} must be finite' + ('' if key in signed else ' and positive'))
    if p['step'] > .1:
        raise ValueError('physics.step must be <= 0.1 s, the controller dt limit')
    if not 0 < g['steering_limit'] < g['steering_hard_limit'] < math.pi:
        raise ValueError('steering targets must lie within physical endpoints below pi')
    if g['wheelbase'] >= g['length'] or g['track'] >= g['width']:
        raise ValueError('steering axes must be inside the chassis perimeter')
    if spawn is None:
        spawn = dict(x=0, y=0, z=0, yaw=0)
    for key in ('x', 'y', 'z', 'yaw'):
        value = spawn[key]
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'spawn.{key} must be finite')
    if mid360_config is None:
        odin = ET.parse(Path(share) / 'resource/models/odin1_lite/model.sdf').getroot().find('model')
        sensor_mass = 4 * float(odin.findtext('link/inertial/mass'))
    else:
        primary = mid360_pose(mid360_config, 'primary_in_base')
        secondary = mid360_pose(mid360_config, 'secondary_in_base')
        imu_pose = inverse_pose(mid360_pose(mid360_config, 'primary_lidar_in_imu'))
        try:
            frequency = mid360_config['driver']['ros_parameters']['publish_freq']
            if type(frequency) not in (int, float) or not math.isfinite(frequency) or not .5 <= frequency <= 100:
                raise ValueError('must be a finite number within [0.5, 100] Hz')
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f'Invalid MID360 publish_freq: {error}') from error
        sensor_mass = 2 * .265
    body_mass = d['total_mass'] - 4 * (d['steer_mass'] + d['wheel_mass']) - sensor_mass
    if body_mass <= 0:
        raise ValueError('total_mass must exceed the four modules and sensor bodies')
    sdf = ET.Element('sdf', version='1.9')
    model = element(sdf, 'model', name='swerve_odin', canonical_link='base_link')
    element(model, 'self_collide', 'false')
    size = (g['length'], g['width'], g['body_height'])
    body = physical_link(model, 'base_link', '__model__', (0,)*6, body_mass,
                         box_inertia(body_mass, size), box(size), (0, 0, g['body_center_z'], 0, 0, 0),
                         '0.12 0.17 0.22 1', d)
    # Decorative top plate, perimeter accents and service cover; mass is included
    # in the single chassis inertia assumption, no duplicate collision geometry.
    top = g['body_center_z'] + g['body_height']/2
    visual(body, 'aluminium_deck', box((g['length']-.018, g['width']-.018, .008)),
           (0, 0, top+.004, 0, 0, 0), '0.58 0.64 0.69 1')
    visual(body, 'service_cover', box((.34, .26, .018)), (0, 0, top+.017, 0, 0, 0), '0.07 0.10 0.13 1')
    for sign in (-1, 1):
        visual(body, f'side_accent_{sign}', box((g['length']-.08, .005, .018)),
               (0, sign*(g['width']/2+.001), g['body_center_z'], 0, 0, 0), '0.05 0.65 0.63 1')
    for prefix, sx, sy in zip(MODULES, (1, 1, -1, -1), (1, -1, 1, -1)):
        steer, wheel = prefix+'_steer_link', prefix+'_wheel_link'
        # Upright steering spindle and a Y-axis wheel with no caster offset.
        knuckle_size = (g['spindle_length'], g['spindle_width'], g['spindle_height'])
        physical_link(model, steer, 'base_link',
                      (sx*g['wheelbase']/2, sy*g['track']/2, g['wheel_center_z'], 0, 0, 0),
                      d['steer_mass'], box_inertia(d['steer_mass'], knuckle_size), box(knuckle_size),
                      (0, 0, g['spindle_center_z'], 0, 0, 0), '0.26 0.30 0.34 1', d)
        r, w, m = g['wheel_radius'], g['wheel_contact_width'], d['wheel_mass']
        radial, axial = m*(3*r*r+w*w)/12, m*r*r/2
        tire = physical_link(model, wheel, steer, (0,)*6, m, (radial, axial, radial),
                             cylinder(r, w), (0, 0, 0, math.pi/2, 0, 0), '0.025 0.03 0.035 1', d)
        visual(tire, prefix+'_hub', cylinder(r*.53, w+.002),
               (0, 0, 0, math.pi/2, 0, 0), '0.65 0.69 0.72 1')
        joint(model, prefix+'_steer_joint', 'base_link', steer, '0 0 1',
              -g['steering_hard_limit'], g['steering_hard_limit'],
              c['max_steering_velocity'], c['steering_torque'], d['steering_damping'])
        joint(model, prefix+'_wheel_joint', steer, wheel, '0 1 0',
              velocity=c['max_wheel_speed']/r, effort=c['wheel_torque'], damping=d['wheel_damping'])
    if mid360_config is not None:
        add_mid360(model, 'primary', primary, frequency, d, sensors, imu_pose=imu_pose)
        add_mid360(model, 'secondary', secondary, frequency, d, sensors)
    else:
        # Place the outer housing faces flush with the measured chassis perimeter.
        # The body/IMU origin is the canonical asset's housing centre.
        housing_depth = float(odin.findtext('link/collision/geometry/box/size').split()[0])
        mount_x = (g['length'] - housing_depth) / 2
        mount_y = (g['width'] - housing_depth) / 2
        mounts = ((mount_x, 0, 0), (-mount_x, 0, math.pi),
                  (0, mount_y, math.pi/2), (0, -mount_y, -math.pi/2))
        for direction, (x, y, yaw) in zip(DIRECTIONS, mounts):
            prefix = 'odin_' + direction
            for original in odin:
                if original.tag not in ('link', 'frame'):
                    continue
                instance = deepcopy(original)
                for node in instance.iter():
                    for key, value in list(node.attrib.items()):
                        node.set(key, value.replace('odin1_lite', prefix))
                    # Mesh URI is deliberately shared, all transport/frame values differ.
                    if node.text and node.tag != 'uri':
                        node.text = node.text.replace('odin1_lite', prefix)
                if instance.tag == 'link':
                    element(instance, 'pose', numbers((x, y, g['sensor_height'], 0, 0, yaw)), relative_to='base_link')
                    if not sensors:
                        for sensor in instance.findall('sensor'):
                            instance.remove(sensor)
                model.append(instance)
            joint(model, prefix+'_mount', 'base_link', prefix+'_body')
    drive = element(model, 'plugin', filename='libSwerveDrive.so', name='sentry_simulation::SwerveDrive')
    for key in ('wheel_radius', 'wheelbase', 'track', 'steering_limit', 'steering_hard_limit'):
        element(drive, key, g[key])
    for key, value in c.items():
        element(drive, key, value)
    odometry = element(model, 'plugin', filename='gz-sim-odometry-publisher-system',
                       name='gz::sim::systems::OdometryPublisher')
    for key, value in dict(odom_frame=ground_truth_frame, robot_base_frame='base_link', dimensions=3,
                           odom_publish_frequency=c['publish_rate'],
                           odom_topic='/swerve/ground_truth/odometry', tf_topic='/swerve/ground_truth/tf').items():
        element(odometry, key, value)
    if world_path is None:
        world_path = Path(share) / 'resource/worlds/swerve_odin.sdf'
    world = ET.parse(world_path).getroot()
    world_node = world.find('world')
    physics = world_node.find('physics')
    if physics is None:
        physics = element(world_node, 'physics', name='default', type='ignored')
    for key, value in (('max_step_size', p['step']), ('real_time_factor', p['real_time_factor'])):
        setting = physics.find(key)
        if setting is None:
            setting = element(physics, key)
        setting.text = str(value)
    placed = deepcopy(model)
    base_z = spawn['z'] + g['wheel_radius'] - g['wheel_center_z'] + .005
    element(placed, 'pose', numbers((spawn['x'], spawn['y'], base_z, 0, 0, spawn['yaw'])))
    world_node.append(placed)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    paths = {key: output/name for key, name in dict(model='model.sdf', world='world.sdf',
             urdf='robot.urdf', bridge='bridge.yaml').items()}
    for key, root in (('model', sdf), ('world', world), ('urdf', to_urdf(model))):
        ET.indent(root)
        ET.ElementTree(root).write(paths[key], encoding='utf-8', xml_declaration=True)
    paths['bridge'].write_text(yaml.safe_dump(
        bridge_config(sensors, publish_ground_truth_tf, mid360=mid360_config is not None), sort_keys=False))
    for key in ('model', 'world', 'bridge'):
        paths[key].write_text(gazebo_text(paths[key].read_text()))
    return paths

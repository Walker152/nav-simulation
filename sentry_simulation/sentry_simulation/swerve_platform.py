"""Generate one coherent SDF/URDF/bridge set from the platform parameters.

The canonical O1LITE asset remains the source for optics, extrinsics and mass.
Only instance names, topics and mounting poses change here.
"""
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


def bridge_config(sensors):
    entries = []
    def bridge(ros, gz, ros_type, gz_type, direction='GZ_TO_ROS'):
        entries.append(dict(ros_topic_name=ros, gz_topic_name=gz,
                            ros_type_name=ros_type, gz_type_name='ignition.msgs.' + gz_type,
                            direction=direction))
    bridge('/clock', '/clock', 'rosgraph_msgs/msg/Clock', 'Clock')
    bridge('/kinco_swerve/cmd_vel/selected', '/kinco_swerve/cmd_vel/selected',
           'geometry_msgs/msg/TwistStamped', 'Twist', 'ROS_TO_GZ')
    for name in ('joint_states', 'joint_targets'):
        bridge('/swerve/' + name, '/swerve/' + name, 'sensor_msgs/msg/JointState', 'Model')
    bridge('/swerve/reference_twist', '/swerve/reference_twist', 'geometry_msgs/msg/TwistStamped', 'Twist')
    for name in ('wheel_odometry', 'ground_truth/odometry'):
        bridge('/swerve/' + name, '/swerve/' + name, 'nav_msgs/msg/Odometry', 'Odometry')
    bridge('/tf', '/swerve/ground_truth/tf', 'tf2_msgs/msg/TFMessage', 'Pose_V')
    if sensors:
        for direction in DIRECTIONS:
            prefix = '/sim/odin_' + direction
            for ros, gz, ros_type, gz_type in (
                ('/points', '/lidar/points', 'PointCloud2', 'PointCloudPacked'),
                ('/imu', '/imu', 'Imu', 'IMU'),
                ('/rgb/image', '/rgb/image', 'Image', 'Image'),
                ('/rgb/camera_info', '/rgb/camera_info', 'CameraInfo', 'CameraInfo')):
                bridge(prefix + ros, prefix + gz, 'sensor_msgs/msg/' + ros_type, gz_type)
    return entries


def build_assets(config, share, output, sensors=True):
    """Validate parameters, generate install-independent resources and return paths."""
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
    odin = ET.parse(Path(share) / 'resource/models/odin1_lite/model.sdf').getroot().find('model')
    sensor_mass = float(odin.findtext('link/inertial/mass'))
    body_mass = d['total_mass'] - 4 * (d['steer_mass'] + d['wheel_mass'] + sensor_mass)
    if body_mass <= 0:
        raise ValueError('total_mass must exceed the four modules and Odin bodies')
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
    mounts = ((g['length']/2, 0, 0), (-g['length']/2, 0, math.pi),
              (0, g['width']/2, math.pi/2), (0, -g['width']/2, -math.pi/2))
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
    odometry = element(model, 'plugin', filename='ignition-gazebo-odometry-publisher-system',
                       name='ignition::gazebo::systems::OdometryPublisher')
    for key, value in dict(odom_frame='odom', robot_base_frame='base_link', dimensions=3,
                           odom_publish_frequency=c['publish_rate'],
                           odom_topic='/swerve/ground_truth/odometry', tf_topic='/swerve/ground_truth/tf').items():
        element(odometry, key, value)
    world = ET.parse(Path(share) / 'resource/worlds/swerve_odin.sdf').getroot()
    world_node = world.find('world')
    world_node.find('physics/max_step_size').text = str(p['step'])
    world_node.find('physics/real_time_factor').text = str(p['real_time_factor'])
    placed = deepcopy(model)
    element(placed, 'pose', numbers((0, 0, r-g['wheel_center_z']+.005, 0, 0, 0)))
    world_node.append(placed)
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    paths = {key: output/name for key, name in dict(model='model.sdf', world='world.sdf',
             urdf='robot.urdf', bridge='bridge.yaml').items()}
    for key, root in (('model', sdf), ('world', world), ('urdf', to_urdf(model))):
        ET.indent(root)
        ET.ElementTree(root).write(paths[key], encoding='utf-8', xml_declaration=True)
    paths['bridge'].write_text(yaml.safe_dump(bridge_config(sensors), sort_keys=False))
    return paths

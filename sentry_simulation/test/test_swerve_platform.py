"""Integration contracts for generated physics and the four independent sensors."""
from pathlib import Path
import importlib.util
import math
import xml.etree.ElementTree as ET

import pytest
import yaml

SHARE = Path(__file__).resolve().parents[1]


def generate(tmp_path, change=None, sensors=True):
    source = SHARE / 'sentry_simulation/swerve_platform.py'
    assert source.exists(), 'The platform generator has not been implemented'
    spec = importlib.util.spec_from_file_location('swerve_platform', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = yaml.safe_load((SHARE / 'config/swerve_odin.yaml').read_text())
    if change:
        change(config)
    paths = module.build_assets(config, SHARE, tmp_path, sensors=sensors)
    return config, paths


def test_adjusted_geometry_propagates_to_mounts_and_tf(tmp_path):
    def change(c):
        c['geometry']['length'] = 1.2
        c['geometry']['width'] = .8
    _, paths = generate(tmp_path, change)
    model = ET.parse(paths['model']).getroot().find('model')
    robot = ET.parse(paths['urdf']).getroot()
    for direction, expected in {'front': (.6, 0, 0), 'rear': (-.6, 0, math.pi),
                                'left': (0, .4, math.pi / 2), 'right': (0, -.4, -math.pi / 2)}.items():
        name = f'odin_{direction}_body'
        pose = list(map(float, model.find(f"link[@name='{name}']/pose").text.split()))
        joint = next(
            j for j in robot.findall('joint') if j.find('child').get('link') == name)
        origin = joint.find('origin')
        assert pose[0] == expected[0] and pose[1] == expected[1]
        assert pose[5] == pytest.approx(expected[2])
        assert list(map(float, origin.get('xyz').split())) == pytest.approx(pose[:3])
        assert list(map(float, origin.get('rpy').split())) == pytest.approx(pose[3:])


def test_mass_and_eight_axis_physical_tree(tmp_path):
    config, paths = generate(tmp_path)
    model = ET.parse(paths['model']).getroot().find('model')
    links = model.findall('link')
    assert sum(float(l.findtext('inertial/mass')) for l in links) == pytest.approx(config['dynamics']['total_mass'])
    for link in links:
        assert float(link.findtext('inertial/mass')) > 0
        moments = [float(link.findtext(f'inertial/inertia/{axis}')) for axis in ('ixx', 'iyy', 'izz')]
        assert min(moments) > 0 and max(moments) <= sum(moments) - max(moments) + 1e-10
        assert link.find('collision') is not None
    active = [j for j in model.findall('joint') if j.get('type') != 'fixed']
    assert len(active) == 8
    for prefix in ('front_left', 'front_right', 'rear_left', 'rear_right'):
        steer = model.find(f"joint[@name='{prefix}_steer_joint']")
        wheel = model.find(f"joint[@name='{prefix}_wheel_joint']")
        assert steer.findtext('parent') == 'base_link'
        assert wheel.findtext('parent') == steer.findtext('child')
        assert wheel.findtext('axis/xyz') == '0 1 0'
        assert steer.findtext('axis/xyz') == '0 0 1'


def test_sensor_instances_have_unique_topics_and_resolvable_frames(tmp_path):
    _, paths = generate(tmp_path)
    model = ET.parse(paths['model']).getroot().find('model')
    robot = ET.parse(paths['urdf']).getroot()
    frame_names = {e.get('name') for e in robot.findall('link')}
    sensors = model.findall('link/sensor')
    assert len(sensors) == 12
    assert len({s.findtext('topic') for s in sensors}) == 12
    for sensor in sensors:
        assert sensor.findtext('ignition_frame_id') in frame_names
        assert 'odin1_lite' not in sensor.findtext('topic')
    bridge = yaml.safe_load(Path(paths['bridge']).read_text())
    assert len({b['ros_topic_name'] for b in bridge}) == len(bridge)
    children = [j.find('child').get('link') for j in robot.findall('joint')]
    assert len(children) == len(set(children))
    assert set(children) == frame_names - {'base_link'}


def test_disabled_sensors_preserve_mass_and_visuals(tmp_path):
    _, paths = generate(tmp_path, sensors=False)
    model = ET.parse(paths['model']).getroot().find('model')
    assert not model.findall('link/sensor')
    assert len(model.findall('link')) == 13
    assert len(model.findall("link[@name='odin_front_body']/visual")) >= 20


@pytest.mark.parametrize('section,key,value', [('control', 'max_wheel_speed', 0),
    ('physics', 'step', .2), ('dynamics', 'total_mass', 1), ('geometry', 'wheel_radius', float('nan'))])
def test_invalid_config_fails_before_launch(tmp_path, section, key, value):
    with pytest.raises(ValueError):
        generate(tmp_path, lambda c: c[section].update({key: value}))

#!/usr/bin/env python3
"""Sample the published static home collision surfaces into a binary XYZ PCD.

This is a synthetic geometry prior, including hidden/overlapping surfaces, not
an accumulated lidar scan. Coordinates include model/link/collision poses; the
home_large world currently includes this model with an identity pose. Only the
standalone static SDF subset used by home_large is supported. Unknown geometry,
frames, pose encodings and nested models fail rather than dropping geometry.
"""

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _numbers(text, count, label, positive=False):
    try:
        values = np.array([float(v) for v in (text or '').split()])
    except ValueError as error:
        raise ValueError(f'{label}: expected {count} finite numbers') from error
    if len(values) != count or not np.isfinite(values).all():
        raise ValueError(f'{label}: expected {count} finite numbers')
    if positive and (values <= 0).any():
        raise ValueError(f'{label}: dimensions must be positive')
    return values


def _children(element, allowed):
    attributes = {'name'} if element.tag in {'model', 'link', 'collision'} else set()
    if set(element.attrib) - attributes:
        raise ValueError(f'{element.tag}: unsupported attributes {element.attrib}')
    unknown = {child.tag for child in element} - allowed
    if unknown:
        raise ValueError(f'{element.tag}: unsupported elements {sorted(unknown)}')
    for tag in allowed - {'link', 'collision', 'visual'}:
        if len(element.findall(tag)) > 1:
            raise ValueError(f'{element.tag}: duplicate {tag}')


def _pose(element):
    """Parent-from-child matrix for SDF's default xyz + Euler RPY pose."""
    pose = element.find('pose')
    matrix = np.eye(4)
    if pose is None:
        return matrix
    if pose.attrib:
        raise ValueError(f'{element.tag}: pose attributes are unsupported: {pose.attrib}')
    x, y, z, roll, pitch, yaw = _numbers(pose.text, 6, f'{element.tag} pose')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    matrix[:3, :3] = ((cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr),
                     (sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr),
                     (-sp, cp*sr, cp*cr))
    matrix[:3, 3] = (x, y, z)
    return matrix


def _axis(length, spacing):
    return np.linspace(-length / 2, length / 2, math.ceil(length / spacing) + 1)


def _ring(radius, z, spacing):
    # Multiples of four include the cardinal extrema without exceeding spacing.
    count = max(4, 4 * math.ceil(2 * math.pi * radius / (4 * spacing)))
    angle = np.arange(count) * (2 * math.pi / count)
    return np.column_stack((radius * np.cos(angle), radius * np.sin(angle),
                            np.full(count, z)))


def _surface(geometry, spacing):
    if geometry is None or len(geometry) != 1:
        raise ValueError('collision must have exactly one geometry shape')
    shape = geometry[0]
    if shape.tag == 'box':
        _children(shape, {'size'})
        lengths = _numbers(shape.findtext('size'), 3, 'box size', positive=True)
        axes = [_axis(length, spacing) for length in lengths]
        faces = []
        for fixed in range(3):
            other = [i for i in range(3) if i != fixed]
            grid = np.meshgrid(axes[other[0]], axes[other[1]], indexing='ij')
            for side in (-lengths[fixed] / 2, lengths[fixed] / 2):
                points = np.empty((grid[0].size, 3))
                points[:, fixed] = side
                points[:, other[0]] = grid[0].ravel()
                points[:, other[1]] = grid[1].ravel()
                faces.append(points)
        return np.concatenate(faces), shape.tag
    if shape.tag == 'cylinder':
        _children(shape, {'radius', 'length'})
        radius = _numbers(shape.findtext('radius'), 1, 'cylinder radius', True)[0]
        length = _numbers(shape.findtext('length'), 1, 'cylinder length', True)[0]
        surfaces = [_ring(radius, z, spacing) for z in _axis(length, spacing)]
        radii = np.linspace(0, radius, math.ceil(radius / spacing) + 1)
        for z in (-length / 2, length / 2):
            surfaces.append(np.array([[0., 0., z]]))
            surfaces.extend(_ring(r, z, spacing) for r in radii[1:-1])
        return np.concatenate(surfaces), shape.tag
    if shape.tag == 'sphere':
        _children(shape, {'radius'})
        radius = _numbers(shape.findtext('radius'), 1, 'sphere radius', True)[0]
        steps = max(2, 2 * math.ceil(math.pi * radius / (2 * spacing)))
        surfaces = [np.array([[0., 0., -radius], [0., 0., radius]])]
        for latitude in np.linspace(-math.pi / 2, math.pi / 2, steps + 1)[1:-1]:
            surfaces.append(_ring(radius * math.cos(latitude),
                                  radius * math.sin(latitude), spacing))
        return np.concatenate(surfaces), shape.tag
    raise ValueError(f'unsupported collision geometry: {shape.tag}')


def export_model(model_path, output_path, spacing=0.1):
    """Write a PCL binary float32 XYZ PCD and return source/output statistics.

    The caller supplies the standalone model SDF, not a world. Duplicate sample
    keys rounded to 1 micrometre retain the first original surface coordinate;
    this removes shared edges/coincident samples without voxel-centroid shifts.
    """
    if not math.isfinite(spacing) or spacing <= 0:
        raise ValueError('spacing must be finite and positive, in metres')
    model_path, output_path = Path(model_path), Path(output_path)
    if model_path.resolve() == output_path.resolve():
        raise ValueError('output must not overwrite the source SDF')
    source = model_path.read_bytes()
    try:
        root = ET.fromstring(source)
    except ET.ParseError as error:
        raise ValueError(f'invalid SDF: {error}') from error
    if root.tag != 'sdf' or len(root) != 1 or root[0].tag != 'model':
        raise ValueError('expected one standalone SDF model; worlds/includes are unsupported')
    model = root[0]
    _children(model, {'static', 'pose', 'link'})
    if model.findtext('static', '').strip().lower() not in ('true', '1'):
        raise ValueError('model must explicitly be static')
    model_transform = _pose(model)
    surfaces, counts = [], Counter()
    for link in model.findall('link'):
        _children(link, {'pose', 'collision', 'visual'})
        transform = model_transform @ _pose(link)
        for collision in link.findall('collision'):
            _children(collision, {'pose', 'geometry', 'surface', 'max_contacts', 'laser_retro'})
            points, kind = _surface(collision.find('geometry'), spacing)
            collision_transform = transform @ _pose(collision)
            surfaces.append(points @ collision_transform[:3, :3].T + collision_transform[:3, 3])
            counts[kind] += 1
    if not surfaces:
        raise ValueError('model has no collision surfaces')
    points = np.concatenate(surfaces)
    if not np.isfinite(points).all():
        raise ValueError('transformed surface coordinates are nonfinite')
    _, first = np.unique(np.round(points, decimals=6), axis=0, return_index=True)
    points = points[np.sort(first)].astype('<f4')
    if not np.isfinite(points).all():
        raise ValueError('surface coordinates exceed float32 range')
    # Float32 can merge nearby large coordinates even after micrometre dedup.
    _, first = np.unique(points, axis=0, return_index=True)
    points = points[np.sort(first)]
    header = (f'# .PCD v0.7 - Point Cloud Data file format\n'
              f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(points)}\nDATA binary\n').encode('ascii')
    payload = header + points.tobytes(order='C')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(payload)
    return {'model': str(model_path), 'output': str(output_path), 'spacing': spacing,
            'collision_count': sum(counts.values()), 'primitive_counts': dict(counts),
            'point_count': len(points), 'bounds_min': points.min(axis=0).tolist(),
            'bounds_max': points.max(axis=0).tolist(), 'size_bytes': len(payload),
            'model_sha256': hashlib.sha256(source).hexdigest(),
            'output_sha256': hashlib.sha256(payload).hexdigest()}


def main():
    # Keep the logical __file__ location so installed share/scripts symlinks use
    # the resources from that same share, rather than following the source link.
    share = Path(__file__).absolute().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path,
                        default=share / 'resource/models/home_large/model.sdf')
    parser.add_argument('--output', type=Path,
                        default=share / 'resource/maps/pcd/home_large.pcd')
    parser.add_argument('--spacing', type=float, default=0.1,
                        help='maximum axis/arc sampling step in metres (default: 0.1)')
    args = parser.parse_args()
    try:
        summary = export_model(args.model, args.output, args.spacing)
    except (OSError, ValueError) as error:
        parser.exit(1, f'export failed: {error}\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

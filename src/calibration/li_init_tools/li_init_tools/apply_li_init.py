#!/usr/bin/env python3
"""Parse a LI-Init result and write a calibrated FAST-LIO YAML.

Accepted result formats
-----------------------
1) LI-Init text output, for example::

       Rotation LiDAR to IMU:
        0.9999  0.0010 -0.0020
       -0.0010  0.9999  0.0005
        0.0020 -0.0005  0.9999
       Translation LiDAR to IMU:
        0.012  -0.003  0.045
       Time offset: -0.0021

2) A YAML file with keys such as ``extrinsic_R``, ``extrinsic_T`` and
   ``time_offset_lidar_to_imu`` (a full FAST-LIO config also works).

3) Explicit command-line values:
   ``--rotation 9_floats --translation 3_floats --time-offset value``.

The output is a copy of the FAST-LIO config with:
  common.time_offset_lidar_to_imu
  mapping.extrinsic_T
  mapping.extrinsic_R
  mapping.extrinsic_est_en: false
"""

import argparse
import math
import os
import re

import numpy as np
import yaml

DEFAULT_INPUT = os.path.expanduser(
    '~/nav_system_ws/src/mapping/FAST_LIO/config/mid360.yaml')
DEFAULT_OUTPUT = os.path.expanduser(
    '~/nav_system_ws/src/mapping/FAST_LIO/config/mid360_calibrated.yaml')

_NUM_RE = re.compile(r'[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?')


def _floats(text):
    return [float(v) for v in _NUM_RE.findall(text)]


def _flatten(values):
    if isinstance(values, (list, tuple)):
        out = []
        for v in values:
            out.extend(_flatten(v))
        return out
    if isinstance(values, (int, float)):
        return [float(values)]
    return []


def _quat_to_matrix(q, order='xyzw'):
    if order == 'wxyz':
        w, x, y, z = q
    else:
        x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        raise ValueError('zero quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _block_after(lines, keywords, count):
    for i, line in enumerate(lines):
        low = line.lower()
        if any(k in low for k in keywords):
            values = []
            j = i
            while j < len(lines) and len(values) < count:
                values.extend(_floats(lines[j]))
                j += 1
            if len(values) >= count:
                return values[:count]
    return None


def _walk_yaml(node, wanted):
    """Find a key in a nested dict and return its numeric values."""
    if isinstance(node, dict):
        for key, value in node.items():
            if str(key).lower() in wanted:
                values = _flatten(value)
                if values:
                    return values
            found = _walk_yaml(value, wanted)
            if found:
                return found
    elif isinstance(node, list):
        for value in node:
            found = _walk_yaml(value, wanted)
            if found:
                return found
    return None


def parse_result(path, quat_order='xyzw'):
    text = open(path, 'r', encoding='utf-8', errors='ignore').read()
    if path.lower().endswith(('.yaml', '.yml')):
        data = yaml.safe_load(text)
        rot = _walk_yaml(data, {'extrinsic_r', 'rotation', 'rot', 'r_lidar_to_imu'})
        trans = _walk_yaml(data, {'extrinsic_t', 'translation', 'trans', 't_lidar_to_imu'})
        dt = _walk_yaml(data, {'time_offset_lidar_to_imu', 'time_offset',
                               'time_delay', 'dt'})
        if not (rot and trans and dt):
            raise ValueError('YAML result is missing rotation/translation/time offset')
        return _make_rotation(rot, quat_order), _make_translation(trans), float(dt[0])

    lines = text.splitlines()
    rot = _block_after(lines, ('rotation', 'rot_lidar', 'r_lidar', 'r:'), 9)
    if rot is None:
        rot = _block_after(lines, ('rotation', 'rot_lidar', 'r_lidar', 'r:'), 4)
    trans = _block_after(lines, ('translation', 'trans_lidar', 't_lidar', 't:'), 3)
    dt = _block_after(lines, ('time offset', 'time_offset', 'offset', 'dt'), 1)
    if rot is None or trans is None or dt is None:
        raise ValueError('cannot find rotation/translation/time offset in %s' % path)
    return _make_rotation(rot, quat_order), _make_translation(trans), float(dt[0])


def _make_rotation(values, quat_order='xyzw'):
    values = [float(v) for v in values]
    if len(values) == 4:
        R = _quat_to_matrix(values, quat_order)
    elif len(values) >= 9:
        R = np.array(values[:9], dtype=np.float64).reshape(3, 3)
    else:
        raise ValueError('rotation needs 4 (quaternion) or 9 values, got %d'
                         % len(values))
    return _orthonormalize(R)


def _orthonormalize(R):
    det = float(np.linalg.det(R))
    if det < 0.5:
        raise ValueError('rotation determinant %.3f looks invalid' % det)
    u, _, vt = np.linalg.svd(R)
    Rn = u @ vt
    if np.linalg.det(Rn) < 0:
        u[:, -1] *= -1.0
        Rn = u @ vt
    err = float(np.linalg.norm(Rn - R))
    if err > 1e-3:
        print('[apply_li_init] warning: rotation was not orthonormal, '
              'projected to SO(3) (delta=%.5f)' % err)
    return Rn


def _make_translation(values):
    values = [float(v) for v in values]
    if len(values) < 3:
        raise ValueError('translation needs 3 values')
    return values[:3]


def write_config(input_path, output_path, R, T, dt):
    with open(input_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError('unexpected FAST-LIO YAML structure')

    if '/**' in data:
        params = data['/**'].setdefault('ros__parameters', {})
    elif 'ros__parameters' in data:
        params = data['ros__parameters']
    else:
        params = data

    common = params.setdefault('common', {})
    common['time_offset_lidar_to_imu'] = float(dt)

    mapping = params.setdefault('mapping', {})
    mapping['extrinsic_T'] = [float(v) for v in T]
    mapping['extrinsic_R'] = [float(v) for v in np.asarray(R).reshape(-1)]
    mapping['extrinsic_est_en'] = False

    out_dir = os.path.dirname(os.path.abspath(output_path))
    os.makedirs(out_dir, exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=False)
    return output_path


def main():
    parser = argparse.ArgumentParser(
        description='Convert LI-Init output to a FAST-LIO calibrated YAML')
    parser.add_argument('--result', help='LI-Init txt/yaml result file')
    parser.add_argument('--input', default=DEFAULT_INPUT,
                        help='source FAST-LIO yaml')
    parser.add_argument('--output', default=DEFAULT_OUTPUT,
                        help='calibrated FAST-LIO yaml to write')
    parser.add_argument('--rotation', nargs='+', type=float,
                        help='9 row-major values or 4 quaternion values')
    parser.add_argument('--translation', nargs=3, type=float)
    parser.add_argument('--time-offset', type=float)
    parser.add_argument('--quat-order', choices=['xyzw', 'wxyz'],
                        default='xyzw')
    args = parser.parse_args()

    if args.rotation and args.translation is not None and args.time_offset is not None:
        R = _make_rotation(args.rotation, args.quat_order)
        T = _make_translation(args.translation)
        dt = float(args.time_offset)
    else:
        if not args.result:
            parser.error('provide --result or all of --rotation/--translation/'
                         '--time-offset')
        R, T, dt = parse_result(args.result, args.quat_order)

    out = write_config(args.input, args.output, R, T, dt)
    print('rotation (LiDAR -> IMU):')
    print(np.array2string(R, precision=6, suppress_small=True))
    print('translation (LiDAR -> IMU):', ['%.6f' % v for v in T])
    print('time offset lidar->imu:', dt)
    print('written:', out)
    print('launch with: lio_config:=%s' % out)


if __name__ == '__main__':
    main()

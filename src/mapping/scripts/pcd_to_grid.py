#!/usr/bin/env python3
"""Offline PCD -> 2D occupancy map (pgm + yaml) for nav2_map_server.

This is the "batch" counterpart of grid_mapping/pointcloud_to_grid.py: use it
when you already have the accumulated FAST-LIO cloud (PCD) and want a clean
static map for global planning.

Example
  python3 pcd_to_grid.py ~/nav_system_ws/src/mapping/FAST_LIO/PCD/scans.pcd \
      --resolution 0.05 --z-min 0.20 --z-max 2.0 --inflation 0.25
"""

import argparse
import os

import numpy as np

try:
    import open3d as o3d
except ImportError:      # open3d is optional; a plain .npy/.txt cloud still works
    o3d = None

import cv2


def load_points(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == '.npy':
        return np.load(path)[:, :3]
    if ext in ('.txt', '.xyz', '.csv'):
        return np.loadtxt(path, delimiter=',' if ext == '.csv' else None)[:, :3]
    if o3d is None:
        raise RuntimeError('open3d is required to read %s' % path)
    cloud = o3d.io.read_point_cloud(path)
    return np.asarray(cloud.points)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('pcd', help='input .pcd/.npy/.txt point cloud')
    parser.add_argument('-o', '--output-dir', default='~/nav_system_ws/src/mapping/maps')
    parser.add_argument('-n', '--name', default='occupancy_map')
    parser.add_argument('-r', '--resolution', type=float, default=0.05)
    parser.add_argument('--z-min', type=float, default=0.20)
    parser.add_argument('--z-max', type=float, default=2.0)
    parser.add_argument('--inflation', type=float, default=0.25,
                        help='obstacle dilation radius in meters')
    parser.add_argument('--margin', type=float, default=1.0,
                        help='free border around the cloud in meters')
    parser.add_argument('--voxel', type=float, default=0.05)
    parser.add_argument('--no-outlier', action='store_true',
                        help='skip statistical outlier removal')
    parser.add_argument('--unknown-outside', action='store_true',
                        help='leave cells without points unknown instead of free')
    return parser.parse_args()


def main():
    args = parse_args()
    points = load_points(args.pcd)
    print('points: %d' % len(points))

    mask = np.isfinite(points).all(axis=1)
    points = points[mask]

    if not args.no_outlier and o3d is not None and len(points) > 100:
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
        points = np.asarray(cloud.points)
        print('after outlier removal: %d' % len(points))

    if args.voxel > 0 and len(points) > 0:
        keys = np.floor(points / args.voxel).astype(np.int64)
        _, idx = np.unique(keys, axis=0, return_index=True)
        points = points[idx]
        print('after voxel downsample: %d' % len(points))

    z = points[:, 2]
    band = points[(z > args.z_min) & (z < args.z_max)]
    print('height band [%.2f, %.2f]: %d points' % (args.z_min, args.z_max, len(band)))
    if len(band) == 0:
        raise SystemExit('no points in the height band, adjust --z-min/--z-max')

    res = args.resolution
    x_min = band[:, 0].min() - args.margin
    y_min = band[:, 1].min() - args.margin
    x_max = band[:, 0].max() + args.margin
    y_max = band[:, 1].max() + args.margin
    width = int(np.ceil((x_max - x_min) / res))
    height = int(np.ceil((y_max - y_min) / res))

    occupied = np.zeros((height, width), dtype=bool)
    ix = np.floor((band[:, 0] - x_min) / res).astype(np.int64)
    iy = np.floor((band[:, 1] - y_min) / res).astype(np.int64)
    ok = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    occupied[iy[ok], ix[ok]] = True

    if args.unknown_outside:
        grid = np.full((height, width), -1, dtype=np.int8)
        grid[occupied] = 100
    else:
        # everything inside the cloud bounding box is considered free,
        # obstacles are stamped on top (typical for a fully explored room)
        grid = np.zeros((height, width), dtype=np.int8)
        grid[occupied] = 100

    if args.inflation > 0:
        from scipy import ndimage
        radius_cells = int(round(args.inflation / res))
        if radius_cells > 0:
            yy, xx = np.mgrid[-radius_cells:radius_cells + 1,
                              -radius_cells:radius_cells + 1]
            disk = (xx * xx + yy * yy) <= radius_cells * radius_cells
            occupied = ndimage.binary_dilation(occupied, structure=disk)
            grid[occupied] = 100

    img = np.full((height, width), 205, dtype=np.uint8)   # unknown
    img[grid == 0] = 254                                  # free
    img[grid == 100] = 0                                  # occupied
    img = np.flipud(img)                                  # PGM row 0 = max y

    out_dir = os.path.expanduser(args.output_dir)
    os.makedirs(out_dir, exist_ok=True)
    pgm = os.path.join(out_dir, args.name + '.pgm')
    yaml = os.path.join(out_dir, args.name + '.yaml')
    cv2.imwrite(pgm, img)
    with open(yaml, 'w', encoding='utf-8') as f:
        f.write('image: %s\n' % os.path.basename(pgm))
        f.write('resolution: %.4f\n' % res)
        f.write('origin: [%.4f, %.4f, 0.0]\n' % (x_min, y_min))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

    print('saved %s (%dx%d @ %.3f m)' % (yaml, width, height, res))
    print('use: ros2 launch bringup navigation.launch.py map_yaml:=%s' % yaml)


if __name__ == '__main__':
    main()

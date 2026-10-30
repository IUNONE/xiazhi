"""Offline sampled position-workspace video; never connects to robot hardware.

Uses calibrated absolute joint mapping, actual URDF lengths (no display scale),
and optional measured independent leader joint bounds. No collision/IK claims.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from data_pipeline.data_rerun.leader_preview import (
    FOLLOWER_BASE_ROTATION, LEADER_BASE_ROTATION, LEADER_BASE_ROTATED_ANCHOR, LEADER_URDF,
)
from data_pipeline.data_rerun.robot_model import DEFAULT_ROBOT_XACRO, build_robot_urdf

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))


def vector(text):
    return np.asarray([float(v) for v in text.split()])


def rotation(axis, angles):
    """Batched Rodrigues rotations, including URDF arbitrary joint axes."""
    norm = np.linalg.norm(axis)
    axis = axis / norm
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    c, s = np.cos(angles)[..., None, None], np.sin(angles)[..., None, None]
    return c * np.eye(3) + (1 - c) * np.outer(axis, axis) + s * skew


class Chain:
    """Vectorized FK for the six revolute joints and any fixed tip links."""

    def __init__(self, urdf, arm, base, tip, tcp):
        root = ET.fromstring(urdf)
        parents = {j.find('child').attrib['link']: j for j in root.findall('joint')}
        chain = []
        while tip != base:
            joint = parents[tip]
            chain.append(joint)
            tip = joint.find('parent').attrib['link']
        self.steps, bounds, names = [], [], []
        for joint in reversed(chain):
            kind = joint.attrib['type']
            origin = joint.find('origin')
            xyz = vector(origin.get('xyz', '0 0 0') if origin is not None else '0 0 0')
            rpy = vector(origin.get('rpy', '0 0 0') if origin is not None else '0 0 0')
            transform = np.eye(4)
            transform[:3, :3] = (
                rotation(np.array([0, 0, 1]), rpy[2])
                @ rotation(np.array([0, 1, 0]), rpy[1])
                @ rotation(np.array([1, 0, 0]), rpy[0])
            )
            transform[:3, 3] = xyz
            axis = joint.find('axis')
            axis = vector(axis.get('xyz', '1 0 0') if axis is not None else '1 0 0')
            index = None
            if kind != 'fixed':
                index = len(names)
                names.append(joint.attrib['name'])
                limit = joint.find('limit')
                bounds.append([-np.pi, np.pi] if kind == 'continuous' else
                              [float(limit.attrib['lower']), float(limit.attrib['upper'])])
            self.steps.append((transform, axis, index))
        self.bounds = np.asarray(bounds)
        self.tcp = tcp
        self.root = np.eye(4)
        self.root[:3, :3] = LEADER_BASE_ROTATION if arm == 'leader' else FOLLOWER_BASE_ROTATION
        if arm == 'leader':
            self.root[:3, 3] = -LEADER_BASE_ROTATED_ANCHOR

    def fk(self, angles):
        transforms = np.broadcast_to(self.root, (len(angles), 4, 4)).copy()
        points = [transforms[:, :3, 3].copy()]
        for origin, axis, index in self.steps:
            transforms = transforms @ origin
            points.append(transforms[:, :3, 3].copy())
            if index is not None:
                transforms[:, :3, :3] = transforms[:, :3, :3] @ rotation(axis, angles[:, index])
        points.append((transforms[:, :3, :3] @ self.tcp) + transforms[:, :3, 3])
        return np.stack(points, axis=1)


def intersect(a, b):
    return np.column_stack((np.maximum(a[:, 0], b[:, 0]),
                            np.minimum(a[:, 1], b[:, 1])))


def measured_bounds(path, device_path, calibration_path):
    document = json.loads(path.read_text())
    bounds = np.asarray(document['bounds'], dtype=float)
    if document['coordinate'] == 'leader_model_deg':
        return np.deg2rad(bounds)
    device = json.loads(device_path.read_text())
    calibration = json.loads(calibration_path.read_text())
    indices = device['joint_indices']
    lo, hi = np.asarray(device['raw_min'])[indices], np.asarray(device['raw_max'])[indices]
    degrees_per_raw = np.asarray(device['angle_range_deg'])[indices] / (hi - lo)
    gain = np.asarray(calibration['direction']) * np.asarray(calibration.get('scale', [1.] * 6))
    result = (bounds - np.asarray(calibration['zero_raw'])[:, None]) * degrees_per_raw[:, None] * gain[:, None]
    return np.deg2rad(np.sort(result, axis=1))


def render(path, clouds, skeletons, args, measured):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FFMpegWriter

    fig = plt.figure(figsize=(18, 7), facecolor='#fafafa')
    titles = ['Same mapped joint domain: geometry', 'U-Arm: effect of leader bounds',
              'ECO63: native vs leader-commandable']
    groups = [
        [(clouds['leader_baseline'], '#277da8', 'U-Arm baseline'),
         (clouds['follower_baseline'], '#e68a2e', 'ECO mapped baseline')],
        [(clouds['leader_baseline'], '#b8c1ca', 'URDF/mapping baseline'),
         (clouds['leader_limited'], '#277da8', 'With supplied bounds' if measured else 'No measured bounds')],
        [(clouds['follower_native'], '#b8c1ca', 'ECO native URDF bounds'),
         (clouds['follower_baseline'], '#e68a2e', 'Mapped baseline'),
         (clouds['follower_limited'], '#2d9b65', 'With supplied bounds' if measured else 'Mapped baseline (no limits file)')],
    ]
    axes, dots, lines = [], [], []
    all_points = np.concatenate([v.reshape(-1, 3) for v in clouds.values()]
                                + [v.reshape(-1, 3) for v in skeletons])
    low, high = all_points.min(axis=0), all_points.max(axis=0)
    center, radius = (low + high) / 2, max(float(np.max(high - low)) * .55, .05)
    for i, group in enumerate(groups):
        ax = fig.add_subplot(1, 3, i + 1, projection='3d')
        axes.append(ax)
        ax.set_title(titles[i], fontsize=11, pad=14)
        for dim, setter in enumerate((ax.set_xlim, ax.set_ylim, ax.set_zlim)):
            setter(center[dim] - radius, center[dim] + radius)
        ax.set_box_aspect((1, 1, 1))
        ax.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)')
        for points, color, label in group:
            artist = ax.scatter([], [], [], s=2, color=color, alpha=.28, label=label, depthshade=False)
            dots.append((artist, points))
        ax.legend(loc='lower left', fontsize=8)
        arms = [(0, '#155c83'), (1, '#b35b12')] if i == 0 else [(0 if i == 1 else 1, '#242424')]
        for arm, color in arms:
            line, = ax.plot([], [], [], '-o', color=color, linewidth=2, markersize=3)
            lines.append((line, skeletons[arm]))
    fig.suptitle('U-Arm config2 / ECO63 | sampled end-point position workspaces', fontsize=17)
    fig.text(.5, .085, 'True metric scale; bases visually aligned using existing preview transforms. '
             'Lines connect joint origins (not collision meshes).', ha='center', fontsize=10)
    fig.text(.5, .045, 'Independent sampled poses, not a motion plan. No self-collision, environment, '
             'orientation or dynamic feasibility checks.', ha='center', fontsize=10)
    status = fig.text(.5, .01, '', ha='center', fontsize=10)
    fig.subplots_adjust(left=.03, right=.97, top=.88, bottom=.15, wspace=.12)
    frames = max(2, round(args.seconds * args.fps))
    writer = FFMpegWriter(fps=args.fps, codec='libx264', extra_args=['-pix_fmt', 'yuv420p'])
    try:
        with writer.saving(fig, str(path), dpi=100):
            for frame in range(frames):
                progress = min(1., (frame + 1) / (frames * .7))
                count = max(1, round(len(next(iter(clouds.values()))) * progress))
                for artist, points in dots:
                    artist._offsets3d = tuple(points[:count].T)
                index = min(count - 1, len(skeletons[0]) - 1)
                for line, poses in lines:
                    line.set_data_3d(*poses[index].T)
                for ax in axes:
                    ax.view_init(elev=24, azim=-60 + 120 * frame / (frames - 1))
                status.set_text(f'{count:,} samples per domain | '
                                + ('Supplied independent joint bounds; coupled interference not modeled'
                                   if measured else 'URDF-ONLY: no measured mechanical limits supplied'))
                writer.grab_frame()
    finally:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--leader-config', type=Path, default=Path('configs/teleoperation/zhongling_eco63.example.json'))
    parser.add_argument('--model-calibration-input', type=Path, default=Path('configs/teleoperation/uarm.model.calibrated.json'))
    parser.add_argument('--leader-follower-mapping', type=Path, default=Path('configs/teleoperation/leader_follower.mapping.json'))
    parser.add_argument('--leader-limits', type=Path, help='Measured independent bounds JSON; omission is explicitly URDF-only')
    parser.add_argument('--robot-xacro', type=Path, default=DEFAULT_ROBOT_XACRO)
    for arm in ('leader', 'follower'):
        parser.add_argument(f'--{arm}-base', default='base_link')
        parser.add_argument(f'--{arm}-tip', default='link6')
        parser.add_argument(f'--{arm}-tcp', type=vector, default=np.zeros(3), help='Quoted "x y z" metres in tip frame')
    parser.add_argument('--samples', type=int, default=12000)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--fps', type=int, default=24)
    parser.add_argument('--seconds', type=float, default=12)
    parser.add_argument('--output', type=Path, default=Path('outputs/arm_workspaces.mp4'))
    args = parser.parse_args()
    report_path = args.output.with_suffix('.json')
    leader = LEADER_URDF.read_bytes()
    follower = build_robot_urdf(args.robot_xacro.resolve())
    mapping = json.loads(args.leader_follower_mapping.read_text())
    lc = Chain(leader, 'leader', args.leader_base, args.leader_tip, args.leader_tcp)
    fc = Chain(follower, 'follower', args.follower_base, args.follower_tip, args.follower_tcp)
    offset, sign = np.deg2rad(mapping['offset_deg']), np.asarray(mapping['direction'])
    # Intersect analytically in joint space: never clip individual mapped samples.
    inverse = np.sort(sign[:, None] * (fc.bounds - offset[:, None]), axis=1)
    baseline = intersect(lc.bounds, inverse)
    limited = baseline if args.leader_limits is None else intersect(
        baseline, measured_bounds(args.leader_limits, args.leader_config, args.model_calibration_input))
    unit = np.random.default_rng(args.seed).random((args.samples, 6))
    def sample(bounds):
        return bounds[:, 0] + unit * (bounds[:, 1] - bounds[:, 0])

    q0, q1 = sample(baseline), sample(limited)
    l0, l1 = lc.fk(q0), lc.fk(q1)
    f0, f1 = fc.fk(offset + sign * q0), fc.fk(offset + sign * q1)
    native = fc.fk(sample(fc.bounds))
    clouds = dict(leader_baseline=l0[:, -1], follower_baseline=f0[:, -1],
                  leader_limited=l1[:, -1], follower_limited=f1[:, -1], follower_native=native[:, -1])
    report = {
        'kind': 'sampled_position_workspace_comparison', 'version': 1,
        'arguments': {k: v.tolist() if isinstance(v, np.ndarray) else str(v) if isinstance(v, Path) else v
                      for k, v in vars(args).items()},
        'joint_names': list(JOINT_NAMES), 'measured_limits_supplied': args.leader_limits is not None,
        'baseline_leader_bounds_deg': np.rad2deg(baseline).tolist(),
        'limited_leader_bounds_deg': np.rad2deg(limited).tolist(),
        'follower_native_bounds_deg': np.rad2deg(fc.bounds).tolist(),
        'retained_joint_box_fraction': float(np.prod((limited[:, 1] - limited[:, 0]) /
                                                    (baseline[:, 1] - baseline[:, 0]))),
        'sampled_position_bounds_m': {k: [v.min(axis=0).tolist(), v.max(axis=0).tolist()] for k, v in clouds.items()},
        'limitations': ['Finite random FK sampling, not workspace volume or IK coverage.',
                        'Independent bounds do not model posture-dependent interference.',
                        'No collision, orientation, dynamics or path feasibility checks.',
                        'Encoder full-scale range is not a mechanical joint limit.',
                        'Preview base alignment, true lengths; default endpoint is link6 origin.'],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    print('Rendering sampled FK workspaces; no collision checking or hardware access.', flush=True)
    render(args.output, clouds, (l1, f1), args, args.leader_limits is not None)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(f'Video: {args.output}\nReport: {report_path}')


if __name__ == '__main__':
    main()

"""Offline U-Arm zero-pose editing and export. No hardware access."""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import socket
import subprocess
import time
from pathlib import Path
from uuid import uuid4

import numpy as np

from data_pipeline.data_rerun.leader_preview import (
    LEADER_URDF, LEADER_BASE_ROTATION, LEADER_BASE_ROTATED_ANCHOR,
    LEADER_DISPLAY_SCALE, _viewer_env,
)

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))


def validate_angles(angles):
    values = tuple(float(v) for v in angles)
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Provide six finite joint angles in degrees.')
    return values


def pose_document(angles, urdf=LEADER_URDF):
    return {
        'kind': 'uarm_zero_pose', 'version': 1,
        'source_urdf': Path(urdf).name,
        'urdf_sha256': hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),
        'joint_angles_deg': dict(zip(JOINT_NAMES, validate_angles(angles))),
    }


def load_pose(path, urdf=LEADER_URDF):
    document = json.loads(Path(path).read_text())
    if document.get('kind') != 'uarm_zero_pose' or document.get('version') != 1:
        raise ValueError('Not a U-Arm zero-pose file.')
    if document.get('urdf_sha256') != hashlib.sha256(Path(urdf).read_bytes()).hexdigest():
        raise ValueError('The URDF has changed since this pose was saved.')
    angles = document.get('joint_angles_deg', {})
    if set(angles) != set(JOINT_NAMES):
        raise ValueError('The pose must contain joint1 through joint6.')
    return validate_angles(angles[name] for name in JOINT_NAMES)


def save_pose(path, angles, *, overwrite=False):
    """Save exclusively by default; return whether an existing file was replaced."""
    path = Path(path)
    text = json.dumps(pose_document(angles), indent=2) + '\n'
    replaced = False
    try:
        stream = path.open('x', encoding='utf-8')
    except FileExistsError:
        if not overwrite:
            raise FileExistsError(f'{path} already exists. Use --overwrite to replace it.') from None
        stream = path.open('w', encoding='utf-8')
        replaced = True
    with stream:
        stream.write(text)
    return replaced


class ZeroPosePreview:
    def __init__(self, memory_limit='25%'):
        import rerun as rr
        import rerun.blueprint as rrb
        from rerun.urdf import UrdfTree

        if not importlib.metadata.version('rerun-sdk').startswith('0.33.'):
            raise RuntimeError('The zero-pose editor requires Rerun SDK 0.33.')
        version = subprocess.run(['rerun', '--version'], check=True, capture_output=True, text=True).stdout
        if '0.33.' not in version:
            raise RuntimeError('The Rerun CLI must also be version 0.33.')
        self.rr = rr
        self.viewer = None
        self.recording = None
        self.started = time.monotonic()
        try:
            self.tree = UrdfTree.from_file_path(
                LEADER_URDF, entity_path_prefix='leader', frame_prefix='leader/',
                static_transform_entity_path='/leader/tf_static',
            )
            self.joints = [self.tree.get_joint_by_name(name) for name in JOINT_NAMES]
            if any(joint is None or joint.joint_type != 'revolute' for joint in self.joints):
                raise ValueError('The model must contain six revolute joints.')
            self.limits_deg = [(math.degrees(j.limit_lower), math.degrees(j.limit_upper)) for j in self.joints]
            blueprint = rrb.Blueprint(
                rrb.Vertical(
                    rrb.TextDocumentView(name='Zero Pose', origin='/status'),
                    rrb.Spatial3DView(
                        name='U-Arm Zero Pose Editor', origin='/', contents=['/leader/**', '/world/**'],
                        spatial_information=rrb.SpatialInformation(target_frame='world'),
                        eye_controls=rrb.EyeControls3D(position=(1.4, -1.6, 1.1), look_target=(0, -0.15, 0.4), eye_up=(0, 0, 1)),
                    ), row_shares=[1, 8],
                ),
                rrb.TimePanel(state='collapsed', timeline='editor_time', play_state='following'),
                rrb.BlueprintPanel(state='collapsed'), rrb.SelectionPanel(state='collapsed'),
            )
            with socket.socket() as sock:
                sock.bind(('127.0.0.1', 0))
                port = sock.getsockname()[1]
            self.viewer = subprocess.Popen([
                'rerun', '--port', str(port), '--bind', '127.0.0.1',
                '--memory-limit', memory_limit, '--server-memory-limit', '256MiB',
            ], env=_viewer_env())
            deadline = time.monotonic() + 15
            while True:
                if self.viewer.poll() is not None:
                    raise RuntimeError('The Rerun viewer exited during startup.')
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=.1):
                        break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise TimeoutError('Timed out waiting for the Rerun viewer.')
                    time.sleep(.05)
            self.recording = rr.RecordingStream('uarm_zero_editor', recording_id=uuid4())
            self.recording.connect_grpc(f'rerun+http://127.0.0.1:{port}/proxy', default_blueprint=blueprint)
            self.tree.log_urdf_to_recording(self.recording)
            self.recording.log('/world/leader', rr.Transform3D(
                translation=-LEADER_DISPLAY_SCALE * LEADER_BASE_ROTATED_ANCHOR,
                mat3x3=LEADER_DISPLAY_SCALE * LEADER_BASE_ROTATION,
                parent_frame='world', child_frame=f'leader/{self.tree.root_link().name}',
            ), static=True)
            for i, joint in enumerate(self.joints, 1):
                self.recording.log(f'/leader/joint_labels/{joint.name}',
                                   rr.CoordinateFrame(f'leader/{joint.child_link}'),
                                   rr.Points3D([[0, 0, 0]], labels=[f'joint_{i}'], show_labels=True,
                                               colors=[[255, 190, 60]], radii=.006), static=True)
        except BaseException:
            self.close()
            raise

    def update(self, angles):
        angles = validate_angles(angles)
        if any(not lo <= v <= hi for v, (lo, hi) in zip(angles, self.limits_deg)):
            raise ValueError('A joint angle is outside the URDF limits.')
        self.recording.set_time('editor_time', duration=time.monotonic() - self.started)
        for joint, value in zip(self.joints, angles):
            self.recording.log(f'/poses/leader/{joint.name}', joint.compute_transform(math.radians(value), clamp=False))
        values = ', '.join(f'{name}: {value:.1f}' for name, value in zip(JOINT_NAMES, angles))
        self.recording.log('/status', self.rr.TextDocument(
            f'Offline zero-pose editor | Display scale: {LEADER_DISPLAY_SCALE:g}x\n' + values + ' (deg)\n'
            'Use the separate slider window. Save Pose exports angles; it does not modify the URDF.'
        ))
        self.recording.flush(timeout_sec=0)

    def is_open(self):
        return self.viewer is not None and self.viewer.poll() is None

    def close(self):
        try:
            if self.recording is not None:
                self.recording.disconnect()
                self.recording = None
        finally:
            if self.viewer is not None and self.viewer.poll() is None:
                self.viewer.terminate()
                try:
                    self.viewer.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.viewer.kill()
                    self.viewer.wait()

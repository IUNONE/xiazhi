"""Offline joint-pose editing for U-Arm leaders and ECO63 followers."""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
import math
import socket
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from uuid import uuid4

import numpy as np

from data_pipeline.data_rerun.leader_preview import (
    LEADER_URDF, LEADER_BASE_ROTATION, LEADER_BASE_ROTATED_ANCHOR,
    LEADER_DISPLAY_SCALE, LEADER_BASE_POSITION, FOLLOWER_BASE_ROTATION, _viewer_env,
)
from data_pipeline.data_collection.teleoperation.mapping import JointMapping
from data_pipeline.data_rerun.robot_model import DEFAULT_ROBOT_XACRO, build_robot_urdf

JOINT_NAMES = tuple(f'joint{i}' for i in range(1, 7))
ARMS = ('leader', 'follower')
POSES = ('zero', 'reference')
DEFAULT_REFERENCE = {'leader': (90., 90., -90., 0., 0., 90.), 'follower': (30.,) * 6}


def validate_angles(angles):
    values = tuple(float(v) for v in angles)
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Provide six finite joint angles in degrees.')
    return values


@dataclass(frozen=True)
class ArmModel:
    arm: str
    source: Path
    urdf_bytes: bytes
    sha256: str

    @property
    def label(self):
        return 'Leader (U-Arm)' if self.arm == 'leader' else 'Follower (ECO63)'

    @classmethod
    def load(cls, arm, robot_xacro=DEFAULT_ROBOT_XACRO):
        if arm not in ARMS:
            raise ValueError(f'Unknown arm: {arm}')
        if arm == 'leader':
            source = LEADER_URDF.resolve()
            raw = source.read_bytes()
            digest = hashlib.sha256(raw).hexdigest()
            root = ET.fromstring(raw)
            for mesh in root.findall('.//mesh'):
                filename = mesh.get('filename')
                if '://' not in filename:
                    mesh.set('filename', (source.parent / filename).resolve().as_uri())
            raw = ET.tostring(root, encoding='utf-8')
        else:
            source = Path(robot_xacro).resolve()
            raw = build_robot_urdf(source)
            # Hash the expanded kinematics, independent of checkout path and xacro comments.
            normalized = raw.decode().replace(source.parent.parent.as_uri() + '/', 'package://rm_description/')
            canonical = ET.canonicalize(normalized, strip_text=True)
            digest = hashlib.sha256(canonical.encode()).hexdigest()
        return cls(arm, source, raw, digest)


def pose_identity(document):
    if document.get('kind') == 'uarm_zero_pose' and document.get('version') == 1:
        return 'leader', 'zero'
    if document.get('kind') != 'arm_joint_pose' or document.get('version') != 1:
        raise ValueError('Not an arm joint-pose file (encoder calibration files are not poses).')
    arm, purpose = document.get('arm'), document.get('pose')
    if arm not in ARMS or purpose not in POSES:
        raise ValueError('Invalid arm or pose type in pose file.')
    return arm, purpose


def pose_document(angles, model, purpose):
    if purpose not in POSES:
        raise ValueError(f'Unknown pose type: {purpose}')
    return {
        'kind': 'arm_joint_pose', 'version': 1, 'arm': model.arm, 'pose': purpose,
        'source_urdf': model.source.name, 'urdf_sha256': model.sha256,
        'joint_angles_deg': dict(zip(JOINT_NAMES, validate_angles(angles))),
    }


def load_pose(path, model, purpose):
    document = json.loads(Path(path).read_text())
    if pose_identity(document) != (model.arm, purpose):
        raise ValueError(f'The input pose does not match --arm {model.arm} --pose {purpose}.')
    if document.get('urdf_sha256') != model.sha256:
        raise ValueError('The robot model has changed since this pose was saved.')
    angles = document.get('joint_angles_deg', {})
    if set(angles) != set(JOINT_NAMES):
        raise ValueError('The pose must contain joint1 through joint6.')
    return validate_angles(angles[name] for name in JOINT_NAMES)


def save_document(path, document, *, overwrite=False):
    path = Path(path)
    text = json.dumps(document, indent=2) + '\n'
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


def save_pose(path, angles, model, purpose, *, overwrite=False):
    return save_document(path, pose_document(angles, model, purpose), overwrite=overwrite)


class ArmPosePreview:
    def __init__(self, model, purpose, memory_limit='25%', companion=None):
        import rerun as rr
        import rerun.blueprint as rrb
        from rerun.urdf import UrdfTree

        if not importlib.metadata.version('rerun-sdk').startswith('0.33.'):
            raise RuntimeError('The arm pose editor requires Rerun SDK 0.33.')
        version = subprocess.run(['rerun', '--version'], check=True, capture_output=True, text=True).stdout
        if '0.33.' not in version:
            raise RuntimeError('The Rerun CLI must also be version 0.33.')
        self.model, self.purpose = model, purpose
        self.scale = LEADER_DISPLAY_SCALE if model.arm == 'leader' else 1.0
        self.rr = rr
        self.viewer = None
        self.recording = None
        self.temp = tempfile.TemporaryDirectory(prefix='xiazhi-arm-pose-')
        self.started = time.monotonic()
        try:
            model_path = Path(self.temp.name) / f'{model.arm}.urdf'
            model_path.write_bytes(model.urdf_bytes)
            self.tree = UrdfTree.from_file_path(
                model_path, entity_path_prefix='arm', frame_prefix='arm/',
                static_transform_entity_path='/arm/tf_static',
            )
            self.joints = [self.tree.get_joint_by_name(name) for name in JOINT_NAMES]
            if any(joint is None or joint.joint_type != 'revolute' for joint in self.joints):
                raise ValueError('The model must contain six revolute joints.')
            self.limits_deg = [(math.degrees(j.limit_lower), math.degrees(j.limit_upper)) for j in self.joints]
            blueprint = rrb.Blueprint(
                rrb.Vertical(
                    rrb.TextDocumentView(name=f'{model.label} - {purpose.title()} Pose', origin='/status'),
                    rrb.Spatial3DView(
                        name='Absolute Leader/Follower Mapping' if companion else f'{model.label} Joint Editor',
                        origin='/', contents=['/arm/**', '/leader/**', '/world/**'],
                        spatial_information=rrb.SpatialInformation(target_frame='world'),
                        eye_controls=rrb.EyeControls3D(position=(1.4, -1.6, 1.1), look_target=(0, 0.275, 0.4) if companion else (0, -0.15, 0.4), eye_up=(0, 0, 1)),
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
            self.recording = rr.RecordingStream('arm_joint_editor', recording_id=uuid4())
            self.recording.connect_grpc(f'rerun+http://127.0.0.1:{port}/proxy', default_blueprint=blueprint)
            self.tree.log_urdf_to_recording(self.recording)
            self.recording.log('/world/arm', rr.Transform3D(
                translation=-self.scale * LEADER_BASE_ROTATED_ANCHOR if model.arm == 'leader' else [0., 0., 0.],
                mat3x3=self.scale * LEADER_BASE_ROTATION if model.arm == 'leader' else FOLLOWER_BASE_ROTATION,
                parent_frame='world', child_frame=f'arm/{self.tree.root_link().name}',
            ), static=True)
            for i, joint in enumerate(self.joints, 1):
                self.recording.log(f'/arm/joint_labels/{joint.name}',
                                   rr.CoordinateFrame(f'arm/{joint.child_link}'),
                                   rr.Points3D([[0, 0, 0]], labels=[f'joint_{i}'], show_labels=True,
                                               colors=[[255, 190, 60] if model.arm == 'leader' else [80, 200, 255]], radii=.006), static=True)
            if companion is not None:
                companion_path = Path(self.temp.name) / 'leader.urdf'
                companion_path.write_bytes(companion.urdf_bytes)
                self.leader_tree = UrdfTree.from_file_path(
                    companion_path, entity_path_prefix='leader', frame_prefix='leader/',
                    static_transform_entity_path='/leader/tf_static',
                )
                self.leader_joints = [self.leader_tree.get_joint_by_name(name) for name in JOINT_NAMES]
                if any(j is None or j.joint_type != 'revolute' for j in self.leader_joints):
                    raise ValueError('The leader model must contain six revolute joints.')
                self.leader_tree.log_urdf_to_recording(self.recording)
                self.recording.log('/world/leader', rr.Transform3D(
                    translation=np.asarray(LEADER_BASE_POSITION) - LEADER_DISPLAY_SCALE * LEADER_BASE_ROTATED_ANCHOR,
                    mat3x3=LEADER_DISPLAY_SCALE * LEADER_BASE_ROTATION,
                    parent_frame='world', child_frame=f'leader/{self.leader_tree.root_link().name}',
                ), static=True)
                for i, joint in enumerate(self.leader_joints, 1):
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
            self.recording.log(f'/poses/arm/{joint.name}', joint.compute_transform(math.radians(value), clamp=False))
        values = ', '.join(f'{name}: {value:.1f}' for name, value in zip(JOINT_NAMES, angles))
        self.recording.log('/status', self.rr.TextDocument(
            f'{self.model.label} | {self.purpose.title()} pose | Display scale: {self.scale:g}x\n' + values + ' (deg)\n'
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
            try:
                if self.viewer is not None and self.viewer.poll() is None:
                    self.viewer.terminate()
                    try:
                        self.viewer.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        self.viewer.kill()
                        self.viewer.wait()
            finally:
                self.temp.cleanup()


class MappingPreview(ArmPosePreview):
    def __init__(self, leader, follower, memory_limit='25%'):
        self.leader_model = leader
        super().__init__(follower, 'mapping', memory_limit, companion=leader)

    def update_mapping(self, offsets, directions, probe=None):
        mapping = JointMapping.create(offsets, directions, self.leader_model.sha256, self.model.sha256)
        leader = np.zeros(6)
        if probe is not None:
            leader[probe] = np.deg2rad(10.)
            joint = self.leader_joints[probe]
            if not joint.limit_lower <= leader[probe] <= joint.limit_upper:
                raise ValueError('The +10 degree direction preview exceeds the leader limit.')
        follower = mapping.apply(leader)
        self.update(np.rad2deg(follower))
        for joint, value in zip(self.leader_joints, leader):
            self.recording.log(f'/poses/leader/{joint.name}', joint.compute_transform(float(value), clamp=False))
        self.recording.log('/status', self.rr.TextDocument(
            'Absolute mapping: follower = offset + direction * leader\n'
            + ('Leader held at URDF zero.' if probe is None else f'Direction check: leader joint_{probe+1} = +10 deg (preview only).')
            + f'\nOffsets (deg): {list(offsets)}\nDirections: {list(directions)}'
        ))
        self.recording.flush(timeout_sec=0)
        self.current_mapping = mapping

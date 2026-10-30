"""Live leader calibration preview; no follower hardware or motion commands."""
from __future__ import annotations

import importlib.metadata
import os
import socket
import subprocess
import tempfile
import time
from pathlib import Path
from uuid import uuid4

import numpy as np

from data_pipeline.data_rerun.robot_model import build_robot_urdf

LEADER_URDF = Path(__file__).resolve().parents[2] / "assets/robots/uarm_config2/uarm_config2_leader.urdf"
# The large mounting face of base_gen (not the narrow sitter side faces)
# must face down. Its CAD outward normal is approximately
# (+0.36959, -0.92830, -0.04067), mapped here to world -Z.
# Relative to the former side-face alignment, rotate +90 degrees about X.
# Apply at the root so every mesh and joint frame rotates together.
LEADER_BASE_ROTATION = np.array([
    [0.9291949300533162, 0.369258404421656, 0.015652882392246596],
    [-0.0004886500491315925, 0.043579403244319484, -0.9990498470216581],
    [-0.3695896957226246, 0.9283044039413159, 0.0406741986868336],
])
# Bounds center in XY and minimum Z of all four fixed meshes AFTER rotation.
LEADER_BASE_ROTATED_ANCHOR = np.array([0.18929311440502936, -0.08889246692740739, 0.07760459952730553])
LEADER_BASE_POSITION = (0.0, 0.55, 0.0)
LEADER_DISPLAY_SCALE = 3.0
_PROXY_ENV = (
    "ALL_PROXY", "all_proxy", "HTTP_PROXY", "http_proxy", "HTTPS_PROXY", "https_proxy",
    "SOCKS_PROXY", "socks_proxy", "GIT_HTTP_PROXY", "GIT_HTTPS_PROXY",
)


def _viewer_env():
    """Drop inherited proxy variables. Rerun's HTTP client warns on every mesh when they are set."""
    env = os.environ.copy()
    for key in _PROXY_ENV:
        env.pop(key, None)
    env["NO_PROXY"] = "*"
    env["no_proxy"] = "*"
    env["RUST_LOG"] = "warn,ureq=error,re_perf_telemetry=error,re_grpc_server=error"
    return env


class LeaderPreview:
    def __init__(self, calibration, robot_xacro: Path, joint_names, memory_limit: str, model_calibration=None):
        import rerun as rr
        import rerun.blueprint as rrb
        from rerun.urdf import UrdfTree

        if not importlib.metadata.version('rerun-sdk').startswith('0.33.'):
            raise RuntimeError('主臂实时预览需要 Rerun 0.33')
        cli_version = subprocess.run(['rerun', '--version'], check=True, capture_output=True, text=True).stdout
        if '0.33.' not in cli_version:
            raise RuntimeError('Rerun CLI 与 SDK 都需要为 0.33')
        self.rr = rr
        self.calibration = calibration
        self.model_calibration = model_calibration
        self.indices = list(calibration.joint_indices)
        self.reference = None
        self.t0 = None
        self.last_timestamp = None
        self.last_status = None
        self.recording = None
        self.viewer = None
        self.temp = tempfile.TemporaryDirectory(prefix='xiazhi-leader-model-')
        try:
            if not LEADER_URDF.is_file():
                raise FileNotFoundError(f'主臂 URDF 不存在: {LEADER_URDF}')
            follower_urdf = build_robot_urdf(robot_xacro.resolve())
            follower_model = Path(self.temp.name) / 'eco63.urdf'
            follower_model.write_bytes(follower_urdf)
            self.follower = UrdfTree.from_file_path(
                follower_model, entity_path_prefix='follower', frame_prefix='follower/',
                static_transform_entity_path='/follower/tf_static',
            )
            self.leader = UrdfTree.from_file_path(
                LEADER_URDF, entity_path_prefix='leader', frame_prefix='leader/',
                static_transform_entity_path='/leader/tf_static',
            )
            self.joints = self._revolute_joints(self.follower, joint_names)
            self.leader_joints = self._revolute_joints(self.leader, joint_names)
            scene = rrb.Spatial3DView(
                name='主臂与从臂', origin='/', contents=['/follower/**', '/leader/**', '/world/**'],
                spatial_information=rrb.SpatialInformation(target_frame='world'),
                eye_controls=rrb.EyeControls3D(position=(1.4, -1.1, 1.0), look_target=(0.15, 0.275, 0.3), eye_up=(0, 0, 1)),
            )
            blueprint = rrb.Blueprint(
                rrb.Vertical(
                    rrb.TextDocumentView(name='状态', origin='/status'),
                    rrb.Horizontal(scene, rrb.Grid(*[
                        rrb.TimeSeriesView(name=f'{name} (rad)', contents=[f'/joints/{name}', f'/leader_joints/{name}'])
                        for name in joint_names
                    ], grid_columns=2), column_shares=[3, 2]),
                    row_shares=[1, 10],
                ),
                rrb.TimePanel(state='collapsed', timeline='leader_time', play_state='following'),
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
                    raise RuntimeError('Rerun Viewer 启动失败')
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=0.1):
                        break
                except OSError:
                    if time.monotonic() >= deadline:
                        raise TimeoutError('等待 Rerun Viewer 超时')
                    time.sleep(0.05)
            self.recording = rr.RecordingStream('xiazhi_leader_preview', recording_id=uuid4())
            self.recording.connect_grpc(f'rerun+http://127.0.0.1:{port}/proxy', default_blueprint=blueprint)
            self.recording.send_blueprint(blueprint)
            self.follower.log_urdf_to_recording(self.recording)
            self.leader.log_urdf_to_recording(self.recording)
            self._log_joint_labels('leader', self.leader_joints, [255, 190, 60])
            self._log_joint_labels('follower', self.joints, [80, 200, 255])
            self._log_base('follower', self.follower.root_link().name, (0.0, 0.0, 0.0))
            self._log_base(
                'leader', self.leader.root_link().name,
                np.asarray(LEADER_BASE_POSITION) - LEADER_DISPLAY_SCALE * LEADER_BASE_ROTATED_ANCHOR,
                rotation=LEADER_DISPLAY_SCALE * LEADER_BASE_ROTATION,
            )
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _revolute_joints(tree, joint_names):
        joints = [tree.get_joint_by_name(name) for name in joint_names]
        if any(joint is None or joint.joint_type not in ('revolute', 'continuous') for joint in joints):
            raise ValueError(f'URDF 必须包含六个指定的旋转关节: {joint_names}')
        return joints

    def _log_base(self, name, root_link, translation, rotation=None):
        self.recording.log(
            f'/world/{name}',
            self.rr.Transform3D(translation=list(translation), mat3x3=rotation, parent_frame='world', child_frame=f'{name}/{root_link}'),
            static=True,
        )

    def _log_joint_labels(self, name, joints, color):
        for index, joint in enumerate(joints, start=1):
            # A revolute joint's child-frame origin is its pivot. Binding to
            # that frame keeps labels attached through all upstream motion.
            self.recording.log(
                f'/{name}/joint_labels/{joint.name}',
                self.rr.CoordinateFrame(f'{name}/{joint.child_link}'),
                self.rr.Points3D(
                    [[0.0, 0.0, 0.0]],
                    labels=[f'joint_{index}'], show_labels=True,
                    colors=[color], radii=0.006,
                ),
                static=True,
            )

    def is_open(self):
        return self.viewer is not None and self.viewer.poll() is None

    def update(self, sample, target_deg=None, message=None, leader_target_deg=None):
        if sample.timestamp_ns == self.last_timestamp:
            return
        angles = np.asarray(sample.positions, dtype=float)[self.indices]
        if self.reference is None:
            # Match teleoperation's relative engagement, without moving hardware.
            self.reference = angles.copy()
            self.t0 = sample.timestamp_ns
        # Model zero/direction belong to the U-Arm, not the ECO63 mapping.
        relative = angles - self.reference
        live = (self.model_calibration.joint_positions(sample.positions)
                if self.model_calibration is not None else relative)
        if leader_target_deg is not None:
            live = np.deg2rad(leader_target_deg)
        if target_deg is not None:
            follower = np.deg2rad(target_deg)
        elif self.calibration.document.get('zero_raw') is not None:
            follower = self.calibration.joint_positions(sample.positions)
        else:
            follower = np.asarray(self.calibration.direction)*self.calibration.scale*relative
        if not np.isfinite(live).all() or not np.isfinite(follower).all():
            raise ValueError('虚拟关节目标包含非有限数值')
        self.recording.set_time('leader_time', duration=(sample.timestamp_ns-self.t0)/1e9)
        outside = []
        for index, (joint, value) in enumerate(zip(self.joints, follower)):
            if value < joint.limit_lower or value > joint.limit_upper:
                outside.append(joint.name)
            # Show the unmodified mapping; no dynamics or tracking-error simulation.
            self.recording.log(f'/poses/follower/{joint.name}', joint.compute_transform(float(value), clamp=False))
            self.recording.log(f'/joints/{joint.name}', self.rr.Scalars(float(value)))
            self.recording.log(f'/leader_joints/{joint.name}', self.rr.Scalars(float(live[index])))
        for joint, value in zip(self.leader_joints, live):
            self.recording.log(f'/poses/leader/{joint.name}', joint.compute_transform(float(value), clamp=False))
        status = message or '原点是 ECO63 从臂。Y 正方向 0.55 m 处是 U-Arm Config 2 底座，显示放大 3 倍，底座最低点位于地面，零位为 CAD 装配姿态。不连接真实从臂。'
        if target_deg is not None:
            status += '\n从臂保持在目标姿态，主臂网格跟随相对启动时的读数变化。'
        elif self.calibration.document.get('zero_raw') is None:
            status += '\n尚未标定：从臂显示启动后的相对变化。'
        if self.model_calibration is None and leader_target_deg is None:
            status += '\n主臂模型未标定：当前启动姿态暂作 CAD 零位。用 --calibrate-model 逐关节标定。'
        if outside:
            status += '\n超出 URDF 限位：' + ', '.join(outside)
        if status != self.last_status:
            self.recording.log('/status', self.rr.TextDocument(status))
            self.last_status = status
        self.last_timestamp = sample.timestamp_ns
        self.recording.flush(timeout_sec=0)

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

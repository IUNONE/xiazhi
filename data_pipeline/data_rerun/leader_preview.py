"""Live leader calibration preview; no follower hardware or motion commands."""
from __future__ import annotations

import importlib.metadata
import socket
import subprocess
import tempfile
import time
from pathlib import Path
from uuid import uuid4

import numpy as np

from data_pipeline.data_rerun.robot_model import build_robot_urdf


class LeaderPreview:
    def __init__(self, calibration, robot_xacro: Path, joint_names, memory_limit: str):
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
        self.indices = list(calibration.joint_indices)
        self.reference = None
        self.t0 = None
        self.last_timestamp = None
        self.last_status = None
        self.recording = None
        self.viewer = None
        self.temp = tempfile.TemporaryDirectory(prefix='xiazhi-leader-model-')
        try:
            urdf = build_robot_urdf(robot_xacro.resolve())
            model = Path(self.temp.name) / 'eco63.urdf'
            model.write_bytes(urdf)
            self.tree = UrdfTree.from_file_path(model, entity_path_prefix='robot_model')
            self.joints = [self.tree.get_joint_by_name(name) for name in joint_names]
            if any(joint is None or joint.joint_type not in ('revolute', 'continuous') for joint in self.joints):
                raise ValueError(f'URDF 必须包含六个指定的旋转关节: {joint_names}')
            scene = rrb.Spatial3DView(
                name='Virtual ECO63', origin='/', contents=['/robot_model/**', '/tf_static', '/joint_transforms/**'],
                spatial_information=rrb.SpatialInformation(target_frame=self.tree.root_link().name),
                eye_controls=rrb.EyeControls3D(position=(1.4, -1.4, 1.2), look_target=(0, 0, 0.45), eye_up=(0, 0, 1)),
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
            ])
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
            self.tree.log_urdf_to_recording(self.recording)
        except BaseException:
            self.close()
            raise

    def is_open(self):
        return self.viewer is not None and self.viewer.poll() is None

    def update(self, sample, target_deg=None, message=None):
        if sample.timestamp_ns == self.last_timestamp:
            return
        angles = np.asarray(sample.positions, dtype=float)[self.indices]
        if self.reference is None:
            # Match teleoperation's relative engagement, without moving hardware.
            self.reference = angles.copy()
            self.t0 = sample.timestamp_ns
        if target_deg is not None:
            target = np.deg2rad(target_deg)
        elif self.calibration.document.get('zero_raw') is not None:
            target = self.calibration.joint_positions(sample.positions)
        else:
            target = np.asarray(self.calibration.direction)*self.calibration.scale*(angles-self.reference)
        if not np.isfinite(target).all():
            raise ValueError('虚拟关节目标包含非有限数值')
        self.recording.set_time('leader_time', duration=(sample.timestamp_ns-self.t0)/1e9)
        outside = []
        for index, (joint, value) in enumerate(zip(self.joints, target)):
            if value < joint.limit_lower or value > joint.limit_upper:
                outside.append(joint.name)
            # Show the unmodified mapping; no dynamics or tracking-error simulation.
            self.recording.log(f'/joint_transforms/{joint.name}', joint.compute_transform(float(value), clamp=False))
            self.recording.log(f'/joints/{joint.name}', self.rr.Scalars(float(value)))
            self.recording.log(f'/leader_joints/{joint.name}', self.rr.Scalars(float(angles[index])))
        status = message or '虚拟从臂预览 · joints 为标定角，leader_joints 为原始角(rad) · 不连接真实从臂'
        if target_deg is None and self.calibration.document.get('zero_raw') is None:
            status += '\n尚未标定：当前仅显示启动后的相对变化'
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

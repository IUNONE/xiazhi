"""Inspect and calibrate the leader with an optional GPU preview of both arms."""
from __future__ import annotations

import argparse
import json
import math
import os
import select
import sys
import termios
import time
import tty
from pathlib import Path

import numpy as np

from data_pipeline.data_collection.hardware.zhongling import ZhonglingLeader
from data_pipeline.data_collection.teleoperation.calibration import calibrate_two_poses
from data_pipeline.data_collection.teleoperation.config import Calibration, LeaderConfig
from data_pipeline.data_rerun.leader_calibration import LeaderModelCalibration, model_axis_direction, sample_stable_raw
from data_pipeline.data_rerun.robot_model import DEFAULT_ROBOT_XACRO


def six_angles(value):
    try:
        angles = tuple(float(v) for v in value.strip('[] ').split(','))
    except ValueError as exc:
        raise argparse.ArgumentTypeError('请输入六个逗号分隔的角度') from exc
    if len(angles) != 6 or not all(math.isfinite(v) for v in angles):
        raise argparse.ArgumentTypeError('需要六个有限角度值，单位 degree')
    return angles


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--leader-config', type=Path, required=True, help='输入：U-Arm 设备参数配置；也可读取含这些参数的已有主从映射文件')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--unlock-leader', action='store_true')
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--save-init', type=Path, help='保存当前主臂 init；覆盖已有文件需 --overwrite')
    output.add_argument('--calibrate', type=Path, help='两姿态标定输出文件；覆盖已有文件需 --overwrite')
    output.add_argument('--calibrate-model', action='store_true', help='执行 U-Arm 模型固定参考姿态标定（只标零位和方向，不拟合角度比例）；需 --viewer 和 --model-calibration-output')
    parser.add_argument('--model-reference-joints', type=six_angles, default=(90., 90., -90., 0., 0., 90.), help='U-Arm 方向参考姿态，degree；只使用角度正负号，0° 轴另用 +30° 补充姿态')
    parser.add_argument('--model-calibration-output', type=Path, help='输出：U-Arm 模型标定 JSON；覆盖已有文件需 --overwrite')
    parser.add_argument('--overwrite', action='store_true', help='允许保存时覆盖已有输出文件')
    parser.add_argument('--model-calibration-input', type=Path, help='输入：加载已保存的 U-Arm 模型零位与正方向标定')
    parser.add_argument('--zero-joints', type=six_angles, default=(0.,)*6, help='零位对应的虚拟关节角，默认全零，degree')
    parser.add_argument('--reference-joints', type=six_angles, help='标定位对应的已知六关节角，degree；每轴需不同于零位')
    parser.add_argument('--viewer', action='store_true', help='打开 Rerun GPU Viewer：主臂、从臂和六轴曲线')
    parser.add_argument('--viewer-fps', type=float, default=30)
    parser.add_argument('--robot-xacro', type=Path, default=DEFAULT_ROBOT_XACRO)
    parser.add_argument('--memory-limit', default='25%')
    args = parser.parse_args()
    if not math.isfinite(args.viewer_fps) or not 1 <= args.viewer_fps <= 120:
        parser.error('viewer-fps 必须在 1..120')
    if args.calibrate and args.reference_joints is None:
        parser.error('--calibrate 需要 --reference-joints 指定已知标定位')
    if args.calibrate and any(abs(a-b) < 5 for a,b in zip(args.zero_joints,args.reference_joints)):
        parser.error('零位与标定位的每个关节至少需要 5° 差异')
    if args.calibrate_model != (args.model_calibration_output is not None):
        parser.error('--calibrate-model 与 --model-calibration-output 必须一起使用')
    if args.calibrate_model and (not args.viewer or args.model_calibration_input):
        parser.error('--calibrate-model 需要 --viewer，且不能同时指定 --model-calibration-input')
    if args.calibrate_model and any(0 < abs(v) < 5 for v in args.model_reference_joints):
        parser.error('模型标定位每轴需为 0° 或与零位至少相差 5°')
    destination = args.calibrate or args.save_init or args.model_calibration_output
    if destination is not None and destination.exists() and not args.overwrite:
        raise ValueError(f'{destination} already exists, add --overwrite to write')
    if (destination or args.unlock_leader) and not sys.stdin.isatty():
        parser.error('标定、保存 init 或卸力需要交互终端确认')
    calibration = Calibration.load(args.leader_config, require_verified=False)
    leader = ZhonglingLeader(LeaderConfig(args.port, args.leader_config, args.baudrate), calibration)
    from data_pipeline.data_rerun.leader_preview import LEADER_URDF
    model_calibration = (LeaderModelCalibration.load(args.model_calibration_input, calibration, LEADER_URDF)
                         if args.model_calibration_input else None)
    preview = None
    target_pose = [0.] * 6 if args.calibrate_model else None
    leader_target_pose = [0.] * 6 if args.calibrate_model else None
    message = None

    def tick():
        sample = leader.latest()
        if preview is not None:
            if not preview.is_open():
                raise KeyboardInterrupt
            preview.update(sample, target_deg=target_pose, message=message, leader_target_deg=leader_target_pose)
        return sample

    def wait_enter(prompt, viewer_prompt, pose=None):
        nonlocal target_pose, message
        target_pose, message = pose, viewer_prompt
        print(prompt, flush=True)
        while True:
            tick()
            ready, _, _ = select.select([sys.stdin], [], [], 1/args.viewer_fps)
            if ready:
                text = sys.stdin.readline()
                if not text or text.strip().lower() == 'q':
                    raise KeyboardInterrupt
                break

    def stable_reading(channels=None):
        return sample_stable_raw(
            tick, range(len(calibration.servo_ids)) if channels is None else channels,
            calibration.servo_ids, min(1 / args.viewer_fps, 0.1),
        )

    def wait_key(prompt, expected, viewer_prompt):
        nonlocal message
        message = viewer_prompt
        print(prompt, flush=True)
        fd = sys.stdin.fileno()
        settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                tick()
                ready, _, _ = select.select([sys.stdin], [], [], 1/args.viewer_fps)
                if ready:
                    key = os.read(fd, 1).lower()
                    if not key or key == b'q':
                        raise KeyboardInterrupt
                    if key == expected:
                        break
        finally:
            # Restore line input and discard queued keys before sampling prompts.
            termios.tcsetattr(fd, termios.TCSAFLUSH, settings)
        print(flush=True)

    def save(document):
        text = json.dumps(document, ensure_ascii=False, indent=2)
        destination.parent.mkdir(parents=True, exist_ok=True)
        replaced = False
        try:
            stream = destination.open('x', encoding='utf-8')
        except FileExistsError:
            if not args.overwrite:
                raise ValueError(f'{destination} already exists, add --overwrite to write') from None
            stream = destination.open('w', encoding='utf-8')
            replaced = True
        with stream:
            stream.write(text)
        print(f'覆盖 {destination}' if replaced else f'已保存: {destination}', flush=True)

    try:
        if args.viewer:
            from data_pipeline.data_rerun.leader_preview import LeaderPreview
            preview = LeaderPreview(calibration, args.robot_xacro, [f'joint{i}' for i in range(1,7)], args.memory_limit, model_calibration=model_calibration)
        leader.connect()
        if not args.calibrate_model and (args.unlock_leader or args.calibrate):
            wait_key('请扶稳主臂，按 t 卸力（无需 Enter）；q 退出。', b't', 'Support the leader arm. Press t to release torque, or q to quit.')
            leader.release_torque()
        if args.calibrate_model:
            print('参考姿态标定：模型固定显示参考姿态，不跟随实物。直接按 y 确认；按 q 退出。', flush=True)

            def capture_pose(pose, label, viewer_label, zero=None, fit_indices=()):
                nonlocal leader_target_pose
                leader_target_pose = list(pose)
                while True:
                    if zero is None:
                        prompt = '请参考 Rerun 将 u-arm 尽量准确摆为零位，保持静止，按 y 确认。'
                        viewer_prompt = 'Match the physical U-Arm to the displayed zero pose as accurately as possible. Hold still and press y; q to quit.'
                    else:
                        joints = ', '.join(f'joint_{i+1}' for i in fit_indices)
                        prompt = f'{label}：将 {joints} 从已记录零位沿画面所示方向移动，不必精确到显示角度。保持静止，按 y 确认。'
                        viewer_prompt = f'Move {joints} from the recorded zero in the directions shown. Exact angles are NOT required. Hold still and press y; q to quit.'
                    wait_key(prompt, b'y', viewer_prompt)
                    try:
                        channels = [calibration.joint_indices[i] for i in fit_indices] if zero is not None else calibration.joint_indices
                        raw = stable_reading(channels)[list(calibration.joint_indices)].tolist()
                        for index in fit_indices:
                            model_axis_direction(calibration, index, zero[index], raw[index], pose[index])
                        return raw
                    except ValueError as exc:
                        print(f'{exc}；请重试当前姿态。', flush=True)

            zero = capture_pose([0.] * 6, '零位', 'zero pose')
            reference_deg = list(args.model_reference_joints)
            changed = [i for i, value in enumerate(reference_deg) if value != 0]
            missing = [i for i, value in enumerate(reference_deg) if value == 0]
            reference = capture_pose(reference_deg, '标定位', 'reference pose', zero, changed)
            poses = [{'joints_deg': reference_deg.copy(), 'raw': reference.copy()}]
            if missing:
                supplemental = [0.] * 6
                for i in missing:
                    supplemental[i] = 30.
                print('以下关节在标定位中仍为零，需要补充 +30° 姿态：' + ', '.join(f'joint_{i+1}' for i in missing), flush=True)
                extra = capture_pose(supplemental, '补充标定位', 'supplemental reference pose', zero, missing)
                poses.append({'joints_deg': supplemental, 'raw': extra})
                for i in missing:
                    reference[i], reference_deg[i] = extra[i], 30.
            model_calibration = LeaderModelCalibration.from_reference_readings(
                calibration, LEADER_URDF, zero, reference, reference_deg, poses,
            )
            save(model_calibration.document)
            preview.model_calibration = model_calibration
            leader_target_pose = None
            target_pose = None
            message = 'Leader model calibration complete. Live tracking restored using saved zero positions and directions. Encoder angle conversion is unchanged. Press Ctrl-C to exit.'
            print('主臂模型零位和方向已保存，角度比例沿用设备配置，已恢复实物驱动模型。Ctrl-C 退出。', flush=True)
        elif args.calibrate:
            wait_enter(f'手动摆到零位 {list(args.zero_joints)}°，静止后按 Enter 记录。', f'Move the leader to the pose mapped to {list(args.zero_joints)} deg. Hold still and press Enter to record zero.', args.zero_joints)
            zero = stable_reading()
            wait_enter(f'手动摆到标定位 {list(args.reference_joints)}°，静止后按 Enter 记录。', f'Move the leader to the pose mapped to {list(args.reference_joints)} deg. Hold still and press Enter to record the reference.', args.reference_joints)
            reference = stable_reading()
            document = calibrate_two_poses(calibration, zero, reference, args.zero_joints, args.reference_joints)
            save(document)
            calibration = Calibration.load(destination)
            leader.calibration = calibration
            if preview is not None:
                preview.calibration = calibration
            target_pose = None
            message = 'Follower mapping calibration complete. ECO63 shows mapped angles. U-Arm uses its model calibration, or startup-relative angles if none is loaded. Press Ctrl-C to exit.'
            print('两姿态标定完成，已切换到实时预览。Ctrl-C 退出。', flush=True)
            if preview is None:
                return
        elif args.save_init:
            wait_enter('手动摆好主臂 init 姿态，保持稳定后按 Enter 保存。', 'Move the leader to its initial pose. Hold still and press Enter to save.')
            document = dict(calibration.document)
            document['home_raw'] = stable_reading().round().astype(int).tolist()
            # Never put motion/control settings into a calibration artifact.
            for field in ('homing','lower_rad','upper_rad','max_velocity_rad_s','max_tracking_error_rad'):
                document.pop(field, None)
            save(document)
            return
        next_print = 0.0
        while True:
            sample = tick()
            if time.monotonic() >= next_print:
                print(f'ids={calibration.servo_ids} raw={sample.raw} age_ms={(time.monotonic_ns()-sample.timestamp_ns)/1e6:.1f}', flush=True)
                next_print = time.monotonic() + (1.0 if preview else 0.1)
            time.sleep(1/args.viewer_fps if preview else 0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            leader.disconnect()
        finally:
            if preview is not None:
                preview.close()


if __name__ == '__main__':
    main()

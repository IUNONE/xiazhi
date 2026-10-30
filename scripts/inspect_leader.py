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
from data_pipeline.data_rerun.leader_calibration import LeaderModelCalibration, joint_direction
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
    output.add_argument('--save-init', type=Path, help='保存当前主臂 init 到新标定文件')
    output.add_argument('--calibrate', type=Path, help='两姿态标定输出文件，不覆盖已有文件')
    output.add_argument('--calibrate-model', action='store_true', help='执行 U-Arm 模型逐关节零位与正方向标定；需 --viewer 和 --model-calibration-output')
    parser.add_argument('--model-calibration-output', type=Path, help='输出：新建 U-Arm 模型标定 JSON，不覆盖已有文件')
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
    destination = args.calibrate or args.save_init or args.model_calibration_output
    if destination is not None and destination.exists():
        raise FileExistsError(destination)
    if (destination or args.unlock_leader) and not sys.stdin.isatty():
        parser.error('标定、保存 init 或卸力需要交互终端确认')
    calibration = Calibration.load(args.leader_config, require_verified=False)
    leader = ZhonglingLeader(LeaderConfig(args.port, args.leader_config, args.baudrate), calibration)
    from data_pipeline.data_rerun.leader_preview import LEADER_URDF
    model_calibration = (LeaderModelCalibration.load(args.model_calibration_input, calibration, LEADER_URDF)
                         if args.model_calibration_input else None)
    preview = None
    target_pose = None
    leader_target = None
    message = None

    def tick():
        sample = leader.latest()
        if preview is not None:
            if not preview.is_open():
                raise KeyboardInterrupt
            preview.update(sample, target_deg=target_pose, message=message, leader_target_deg=leader_target)
        return sample

    def wait_enter(prompt, pose=None):
        nonlocal target_pose, message
        target_pose, message = pose, prompt
        print(prompt, flush=True)
        while True:
            tick()
            ready, _, _ = select.select([sys.stdin], [], [], 1/args.viewer_fps)
            if ready:
                text = sys.stdin.readline()
                if not text or text.strip().lower() == 'q':
                    raise KeyboardInterrupt
                break

    def stable_reading():
        samples, seen = [], set()
        deadline = time.monotonic() + 0.8
        while time.monotonic() < deadline:
            sample = tick()
            if sample.timestamp_ns not in seen:
                seen.add(sample.timestamp_ns)
                samples.append(sample.raw)
            time.sleep(1/args.viewer_fps)
        if len(samples) < 3 or np.any(np.ptp(np.asarray(samples), axis=0) > 10):
            raise ValueError('姿态不稳定或新样本不足，请保持静止后重新标定')
        return np.median(np.asarray(samples), axis=0)

    def wait_key(prompt, expected):
        nonlocal message
        message = prompt
        print(message, flush=True)
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
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open('x', encoding='utf-8') as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
        print(f'已保存: {destination}', flush=True)

    try:
        if args.viewer:
            from data_pipeline.data_rerun.leader_preview import LeaderPreview
            preview = LeaderPreview(calibration, args.robot_xacro, [f'joint{i}' for i in range(1,7)], args.memory_limit, model_calibration=model_calibration)
        leader.connect()
        if not args.calibrate_model and (args.unlock_leader or args.calibrate):
            wait_key('请扶稳主臂，按 t 卸力（无需 Enter）；q 退出。', b't')
            leader.release_torque()
        if args.calibrate_model:
            print('逐关节标定：直接按 y 确认，无需 Enter；按 q 退出。', flush=True)
            zero, positive = [], []
            for index, channel in enumerate(calibration.joint_indices):
                leader_target = [0.0] * 6
                while True:
                    wait_key(f'移动 joint_{index+1} 到零位，确定按下 y。', b'y')
                    try:
                        z = float(stable_reading()[channel])
                        break
                    except ValueError as exc:
                        print(f'{exc}；请重试当前零位。', flush=True)
                leader_target[index] = 30.0
                while True:
                    wait_key(f'移动 joint_{index+1} 向正方向，确定按下 y。', b'y')
                    try:
                        p = float(stable_reading()[channel])
                        sign = joint_direction(calibration, index, z, p)
                        break
                    except ValueError as exc:
                        print(f'{exc}；请重试当前正方向。', flush=True)
                zero.append(z)
                positive.append(p)
                print(f'joint_{index+1} 已记录：zero_raw={z:g}, direction={sign:+d}', flush=True)
            model_calibration = LeaderModelCalibration.from_readings(calibration, LEADER_URDF, zero, positive)
            save(model_calibration.document)
            preview.model_calibration = model_calibration
            leader_target = None
            message = '主臂模型逐关节标定完成，正在按已保存的零位和方向实时显示。Ctrl-C 退出。'
            print(message, flush=True)
        elif args.calibrate:
            wait_enter(f'手动摆到零位 {list(args.zero_joints)}°，静止后按 Enter 记录。', args.zero_joints)
            zero = stable_reading()
            wait_enter(f'手动摆到标定位 {list(args.reference_joints)}°，静止后按 Enter 记录。', args.reference_joints)
            reference = stable_reading()
            document = calibrate_two_poses(calibration, zero, reference, args.zero_joints, args.reference_joints)
            save(document)
            calibration = Calibration.load(destination)
            leader.calibration = calibration
            if preview is not None:
                preview.calibration = calibration
            target_pose = None
            message = '两姿态标定完成。ECO63 显示映射后的关节角；U-Arm 使用独立模型标定（未加载时显示启动后的相对角度）。Ctrl-C 退出。'
            print(message, flush=True)
            if preview is None:
                return
        elif args.save_init:
            wait_enter('手动摆好主臂 init 姿态，保持稳定后按 Enter 保存。')
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

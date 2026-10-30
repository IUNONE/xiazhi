"""Inspect and calibrate the leader with an optional GPU preview of both arms."""
from __future__ import annotations

import argparse
import json
import math
import select
import sys
import time
from pathlib import Path

import numpy as np

from data_pipeline.data_collection.hardware.zhongling import ZhonglingLeader
from data_pipeline.data_collection.teleoperation.calibration import calibrate_two_poses
from data_pipeline.data_collection.teleoperation.config import Calibration, LeaderConfig
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
    parser.add_argument('--calibration', type=Path, required=True, help='设备模板或已保存的零位标定')
    parser.add_argument('--baudrate', type=int, default=115200)
    parser.add_argument('--unlock-leader', action='store_true')
    output = parser.add_mutually_exclusive_group()
    output.add_argument('--save-init', type=Path, help='保存当前主臂 init 到新标定文件')
    output.add_argument('--calibrate', type=Path, help='两姿态标定输出文件，不覆盖已有文件')
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
    destination = args.calibrate or args.save_init
    if destination is not None and destination.exists():
        raise FileExistsError(destination)
    if (destination or args.unlock_leader) and not sys.stdin.isatty():
        parser.error('标定、保存 init 或卸力需要交互终端确认')
    calibration = Calibration.load(args.calibration, require_verified=False)
    leader = ZhonglingLeader(LeaderConfig(args.port, args.calibration, args.baudrate), calibration)
    preview = None
    target_pose = None
    message = None

    def tick():
        sample = leader.latest()
        if preview is not None:
            if not preview.is_open():
                raise KeyboardInterrupt
            preview.update(sample, target_deg=target_pose, message=message)
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

    def save(document):
        destination.parent.mkdir(parents=True, exist_ok=True)
        with destination.open('x', encoding='utf-8') as stream:
            json.dump(document, stream, ensure_ascii=False, indent=2)
        print(f'已保存: {destination}', flush=True)

    try:
        if args.viewer:
            from data_pipeline.data_rerun.leader_preview import LeaderPreview
            preview = LeaderPreview(calibration, args.robot_xacro, [f'joint{i}' for i in range(1,7)], args.memory_limit)
        leader.connect()
        if args.unlock_leader or args.calibrate:
            wait_enter('请扶稳主臂，按 Enter 卸力；q 退出。')
            leader.release_torque()
        if args.calibrate:
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
            message = '两姿态标定完成。U-Arm 网格按标定方向跟随读数变化，ECO63 显示映射后的关节角。Ctrl-C 退出。'
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

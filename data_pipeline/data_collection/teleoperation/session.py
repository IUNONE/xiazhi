from __future__ import annotations

import threading
import time
from concurrent.futures import ThreadPoolExecutor

from data_pipeline.data_collection.hardware.zhongling import ZhonglingLeader
from data_pipeline.data_collection.io_utils.command_log import CommandLog
from data_pipeline.data_collection.teleoperation.config import RuntimeSettings
from data_pipeline.data_collection.teleoperation.controller import TeleoperationController


class _ExitSession(Exception):
    pass


class _CommandSink:
    """Detach the episode log atomically while keeping follower commands active."""
    def __init__(self, arm):
        self.arm = arm
        self.lock = threading.Lock()
        self.journal = None

    def attach(self, journal):
        with self.lock:
            self.journal = journal

    def __call__(self, command):
        with self.lock:
            if self.journal is not None:
                self.journal.submit(command)
        self.arm.publish_teleoperation(command)


def collect_ros_session(collector, path, manifest, keys):
    config = collector.config
    from data_pipeline.data_collection.teleoperation.mapping import load_absolute_calibration
    calibration = load_absolute_calibration(config.leader)
    runtime = RuntimeSettings.load(config.leader.runtime_config)
    runtime.homing(calibration)
    limits = runtime.follower_limits()
    collector._arm.validate_home(limits.lower_rad, limits.upper_rad)
    manifest.collection["teleoperation_runtime"] = runtime.document
    manifest.collection["follower_limits"] = {"lower_rad": limits.lower_rad, "upper_rad": limits.upper_rad, "max_velocity_rad_s": limits.max_velocity_rad_s, "max_tracking_error_rad": limits.max_tracking_error_rad}
    leader = ZhonglingLeader(config.leader, calibration)
    sink = _CommandSink(collector._arm)
    controller = journal = recorder = episode = None
    failure = None
    reason = 'q'
    ready = False
    index = 0
    worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='episode-io')

    def poll():
        collector._ensure_processes_alive()
        leader.latest()
        if controller is not None:
            controller.heartbeat()
        key = keys.read(timeout_s=0.0)
        if key == 'q':
            raise _ExitSession()
        # Ignore i/r/s while an operation is in progress: no queued motion.

    def operation(fn, *, allow_quit=True):
        """Disk/process finalization must not starve the ongoing follow heartbeat."""
        future = worker.submit(fn)
        try:
            while not future.done():
                if allow_quit:
                    poll()
                elif controller is not None:
                    controller.heartbeat()
                time.sleep(0.03)
            return future.result()
        except BaseException:
            # An interrupt during follower homing must cancel motion immediately.
            if controller is not None:
                controller.stop()
            else:
                collector._arm.stop_motion()
            future.result()
            raise

    def finish_episode(stop_reason):
        nonlocal journal, recorder, episode, index
        if episode is None:
            return
        episode['stop_requested_at_ns'] = time.time_ns()
        sink.attach(None)
        operation(lambda: collector._stop_process(recorder), allow_quit=False)
        if recorder.returncode not in (0, -2):
            raise RuntimeError(f'rosbag 未正常关闭: {recorder.returncode}')
        recorder = None
        operation(journal.close, allow_quit=False)
        journal = None
        manifest.finish_episode(episode, status='completed', stop_reason=stop_reason)
        episode = None
        operation(lambda: manifest.write(path), allow_quit=False)
        index += 1

    try:
        leader.connect()
        print('按 i 让主从分别回到 init；回位完成、扶稳主臂后按 r。按 q 退出。', flush=True)
        while True:
            collector._ensure_processes_alive()
            leader.latest()
            if controller is not None:
                controller.heartbeat()
            if recorder is not None and recorder.poll() is not None:
                raise RuntimeError(f'rosbag record 提前退出: {recorder.returncode}')
            if journal is not None:
                journal.drain()
            key = keys.read(timeout_s=0.03)
            if key == 'q':
                if controller is not None:
                    controller.stop()
                    controller = None
                finish_episode('q')
                break
            if key == 'i':
                if episode is not None:
                    print('正在录制，请先按 s 保存，再按 i 回位。', flush=True)
                    continue
                ready = False
                if controller is not None:
                    controller.stop()
                    controller = None
                print('跟随已暂停。主臂回到标定 init，随后从臂独立回位；不录包。', flush=True)
                leader.move_home(poll=poll)
                operation(collector._arm.restore_home)
                ready = True
                print('主从已到 init。请扶好主臂！按 r 卸载主臂力矩并开始跟随、录制。', flush=True)
            elif key == 'r':
                if episode is not None:
                    print('当前 episode 正在录制；按 s 保存。', flush=True)
                    continue
                if not ready and controller is None:
                    print('请先按 i 完成主从回位。', flush=True)
                    continue
                episode_path = config.episode_output_path(index)
                if episode_path.exists():
                    raise FileExistsError(episode_path)
                episode = manifest.start_episode(index, episode_path)
                manifest.write(path)
                recorder = collector._start(collector._record_command(episode_path), managed=False)
                operation(lambda: collector._wait_for_recorder_start(recorder, episode_path))
                deadline = time.monotonic() + config.startup_timeout_s
                while collector._arm._command_pub.get_subscription_count() == 0:
                    poll()
                    if recorder.poll() is not None or time.monotonic() > deadline:
                        raise TimeoutError('rosbag 未订阅遥操 action')
                    time.sleep(0.03)
                journal = CommandLog(path / 'teleoperation' / f'episode_{index:06d}.jsonl')
                sink.attach(journal)
                if controller is None:
                    leader.read_once()
                    leader.release_torque()
                    leader.read_once()
                    controller = TeleoperationController(collector._arm, leader, config.leader, calibration, limits, sink)
                    controller.start()
                ready = False
                print(f'episode {index:06d} 录制中；按 s 保存并继续跟随。', flush=True)
            elif key == 's':
                if episode is None:
                    print('当前未录制。跟随状态不变；按 i 回位或 r 开始录制。', flush=True)
                    continue
                finish_episode('s')
                print('episode 已保存，主臂仍卸力，主从继续跟随。按 i 暂停跟随并回位，或 r 继续录制。', flush=True)
    except _ExitSession:
        reason = 'q'
    except BaseException as exc:
        failure = exc
        reason = 'ctrl_c' if isinstance(exc, KeyboardInterrupt) else 'error'
    finally:
        sink.attach(None)
        if controller is not None:
            try:
                controller.stop()
            except BaseException as exc:
                failure = failure or exc
        worker.shutdown(wait=True)
        operations = []
        if journal is not None:
            operations.append(journal.close)
        if recorder is not None:
            operations.append(lambda: collector._stop_process(recorder))
        operations.append(leader.disconnect)
        for close in operations:
            try:
                close()
            except BaseException as exc:
                failure = failure or exc
        if episode is not None:
            manifest.finish_episode(episode, status='failed', stop_reason=reason, error=failure)
        manifest.write(path)
        print('会话已结束；主臂力矩状态未自动切换，请扶稳或放置好主臂。', flush=True)
    if failure is not None:
        raise failure
    return reason

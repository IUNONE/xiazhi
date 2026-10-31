"""Best-effort, bounded follower-target preview, isolated from robot control."""
from __future__ import annotations

import threading
import time

import numpy as np

from data_pipeline.data_rerun.arm_pose import ArmModel, ArmPosePreview


class _TargetView(ArmPosePreview):
    def __init__(self, model, cancel):
        super().__init__(model, 'target', memory_limit='128MiB',
                         server_memory_limit='32MiB', cancel_event=cancel)
        self.last_status = None

    def show_target(self, target_rad, caption):
        target = np.asarray(target_rad, dtype=float)
        if target.shape != (6,) or not np.isfinite(target).all():
            raise ValueError('Target preview requires six finite joint angles.')
        self.recording.set_time('editor_time', duration=time.monotonic() - self.started)
        outside = []
        for joint, value in zip(self.joints, target):
            if not joint.limit_lower <= value <= joint.limit_upper:
                outside.append(joint.name)
            # Display the exact desired pose, including out-of-limit targets;
            # the acquisition control path independently rejects invalid motion.
            self.recording.log(f'/poses/arm/{joint.name}', joint.compute_transform(float(value), clamp=False))
        status = 'FOLLOWER TARGET PREVIEW - not measured robot state\n' + caption
        if outside:
            status += '\nOUTSIDE JOINT LIMITS: ' + ', '.join(outside)
        if status != self.last_status:
            self.recording.log('/status', self.rr.TextDocument(status))
            self.last_status = status
        self.recording.flush(timeout_sec=0)


class CollectionTargetPreview:
    """A single worker owns Rerun; producers replace one latest-value slot."""
    def __init__(self, expected_model_sha256):
        self.expected_model_sha256 = expected_model_sha256
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._visible = False
        self._latest = None
        self._cancel = None
        self._thread = threading.Thread(target=self._run, name='follower-target-preview', daemon=True)
        self._thread.start()

    def show(self):
        with self._lock:
            if not self._visible:
                self._latest = None
                self._visible = True

    def hide(self):
        with self._lock:
            self._visible = False
            self._latest = None
            if self._cancel is not None:
                self._cancel.set()

    @property
    def visible(self):
        with self._lock:
            return self._visible

    def submit(self, target_rad, caption):
        # No Rerun call or I/O on the collection/control thread.
        item = (tuple(float(v) for v in target_rad), caption)
        with self._lock:
            if self._visible:
                self._latest = item

    def _run(self):
        view = model = None
        last_item = None
        try:
            while not self._stop.is_set():
                with self._lock:
                    visible, item = self._visible, self._latest
                    cancelled = self._cancel is not None and self._cancel.is_set()
                if view is not None and (not visible or cancelled):
                    view.close()
                    view = None
                    last_item = None
                if visible and item is not None and view is None:
                    cancel = threading.Event()
                    with self._lock:
                        if not self._visible or self._stop.is_set():
                            continue
                        self._cancel = cancel
                    try:
                        if model is None:
                            model = ArmModel.load('follower')
                            if model.sha256 != self.expected_model_sha256:
                                raise ValueError('Follower URDF does not match the active mapping.')
                        if cancel.is_set():
                            continue
                        view = _TargetView(model, cancel)
                        if cancel.is_set():
                            view.close()
                            view = None
                            continue
                        last_item = None
                    except InterruptedError:
                        continue
                    except Exception as exc:
                        model = None
                        self._disable(exc)
                        continue
                if view is not None:
                    if not view.is_open():
                        # Closing the preview is not a robot stop command.
                        self.hide()
                    elif item is not None and item != last_item:
                        try:
                            view.show_target(*item)
                            last_item = item
                        except Exception as exc:
                            self._disable(exc)
                self._stop.wait(0.1)  # At most 10 visual updates per second.
        except Exception as exc:
            self._disable(exc)
        finally:
            if view is not None:
                try:
                    view.close()
                except Exception as exc:
                    print(f'目标预览关闭失败: {exc}', flush=True)

    def _disable(self, error):
        self.hide()
        print(f'目标预览不可用: {error}。机械臂控制和录制逻辑未改变；按 s 可重新打开预览。', flush=True)

    def close(self):
        self.hide()
        self._stop.set()
        self._thread.join(timeout=25)
        if self._thread.is_alive():
            raise RuntimeError('目标预览工作线程未在期限内退出')

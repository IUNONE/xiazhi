from __future__ import annotations

import select
import os
import sys
import termios
import tty
import time
from types import TracebackType


class TerminalKeyReader:
    """读取 POSIX 交互终端中的单键采集指令。"""

    def __init__(self, *, allow_non_tty=False) -> None:
        self.allow_non_tty = allow_non_tty
        self._fd: int | None = None
        self._old_settings: list | None = None

    def __enter__(self) -> TerminalKeyReader:
        if not sys.stdin.isatty():
            if self.allow_non_tty:
                return self
            raise RuntimeError(
                "采集必须在交互式 TTY 中运行，不允许无确认自动开始"
            )
        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_value: BaseException | None,
        traceback: TracebackType | None,
    ) -> None:
        if self._fd is not None and self._old_settings is not None:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)
        self._fd = None
        self._old_settings = None

    def read(self, timeout_s: float = 0.1) -> str | None:
        if self._fd is None:
            if self.allow_non_tty:
                time.sleep(max(0., timeout_s))
                return None
            raise RuntimeError("TerminalKeyReader 尚未进入上下文")
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None
        data = os.read(self._fd, 1)
        if not data:
            return 'q'
        key = data.decode('ascii', errors='ignore').lower()
        if key == "\x03":
            raise KeyboardInterrupt
        return key

    def wait_for_drag_start(self, stage: str) -> str:
        """等待操作者确认后开启拖动示教。"""
        print(
            f"{stage} 已准备。按 t 开启拖动示教并开始拖动；"
            "按 q 或 Ctrl-C 结束会话。",
            flush=True,
        )
        while True:
            key = self.read()
            if key in {"t", "q"}:
                return key

    def wait_for_episode_start(self) -> str:
        """等待操作者开始录制 direct episode。"""
        print(
            "拖动示教已开启。按 r 开始录制新 episode；"
            "按 q 或 Ctrl-C 结束会话。",
            flush=True,
        )
        while True:
            key = self.read()
            if key in {"r", "q"}:
                return key

    def recording_command(self, timeout_s: float = 0.0) -> str | None:
        key = self.read(timeout_s)
        return key if key in {"s", "q"} else None

    def wait_for_teaching_finish(self, attempt_index: int) -> str:
        """等待操作者结束当前拖动示教。"""
        print(
            f"示教 attempt {attempt_index:06d} 正在记录控制器轨迹，不录制 rosbag2。"
            "按 s 完成示教；按 q 或 Ctrl-C 结束会话。",
            flush=True,
        )
        while True:
            key = self.read()
            if key in {"s", "q"}:
                return key

    def wait_for_origin_command(self) -> str:
        """等待操作者确认工作区无人后返回示教起点。"""
        print(
            "示教轨迹已保存。移除操作者和障碍物后，按 o 运动到轨迹起点；"
            "按 q 或 Ctrl-C 结束会话且不自动运动。",
            flush=True,
        )
        while True:
            key = self.read()
            if key in {"o", "q"}:
                return key

    def wait_for_replay_command(self) -> str:
        """等待场景复位完成后开始回放录制或废弃示教。"""
        print(
            "机械臂已到达轨迹起点。复位场景并退出摄像头和工作区后，"
            "按 r 回放并录制 rosbag2；按 d 废弃本次示教；"
            "按 q 或 Ctrl-C 结束会话。",
            flush=True,
        )
        while True:
            key = self.read()
            if key in {"r", "d", "q"}:
                return key

    def wait_for_home_command(self) -> str:
        """等待操作者确认后返回 session 初始位姿。"""
        print(
            "episode 已结束。确认返航路径无人且无障碍物后，"
            "按 h 返回 session 初始位姿并继续；"
            "按 q 或 Ctrl-C 在当前位姿结束会话。",
            flush=True,
        )
        while True:
            key = self.read()
            if key in {"h", "q"}:
                return key

from __future__ import annotations

import select
import sys
import termios
import tty
from types import TracebackType


class TerminalKeyReader:
    """Read single-key recording commands from an interactive POSIX terminal."""

    def __init__(self) -> None:
        self._fd: int | None = None
        self._old_settings: list | None = None

    def __enter__(self) -> TerminalKeyReader:
        if not sys.stdin.isatty():
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
            raise RuntimeError("TerminalKeyReader 尚未进入上下文")
        ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
        if not ready:
            return None
        key = sys.stdin.read(1).lower()
        if key == "\x03":
            raise KeyboardInterrupt
        return key

    def wait_for_episode_start(self) -> str:
        print(
            "拖动示教已开启。按 r 开始新 episode；按 q 或 Ctrl-C 结束会话。",
            flush=True,
        )
        while True:
            key = self.read()
            if key in {"r", "q"}:
                return key

    def recording_command(self, timeout_s: float = 0.0) -> str | None:
        key = self.read(timeout_s)
        return key if key in {"s", "q"} else None

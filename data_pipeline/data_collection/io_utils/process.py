from __future__ import annotations

import ctypes
import os
import signal


_PR_SET_PDEATHSIG = 1


def _set_parent_death_signal() -> None:
    """配置父进程退出时向当前进程发送 SIGINT。"""
    libc = ctypes.CDLL(None, use_errno=True)
    prctl = libc.prctl
    prctl.argtypes = [
        ctypes.c_int,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_ulong,
        ctypes.c_ulong,
    ]
    prctl.restype = ctypes.c_int
    if prctl(_PR_SET_PDEATHSIG, signal.SIGINT, 0, 0, 0) != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def exec_guarded(expected_parent_pid: int, command: list[str]) -> None:
    """Replace this process after establishing the parent-death signal."""
    if not command:
        raise ValueError("子进程命令不能为空")
    _set_parent_death_signal()
    if os.getppid() != expected_parent_pid:
        raise RuntimeError("父进程已退出, 拒绝启动受管子进程")
    os.execvp(command[0], command)

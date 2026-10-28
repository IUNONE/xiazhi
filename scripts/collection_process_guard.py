from __future__ import annotations

import sys

from data_pipeline.data_collection.io_utils.process import exec_guarded


def main() -> None:
    if len(sys.argv) < 3:
        raise ValueError("需要提供父进程 PID 和子进程命令")
    exec_guarded(int(sys.argv[1]), sys.argv[2:])


if __name__ == "__main__":
    main()

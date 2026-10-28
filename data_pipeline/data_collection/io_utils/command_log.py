from __future__ import annotations

import json
import queue
from dataclasses import asdict
from pathlib import Path


class CommandLog:
    """Bounded handoff keeps disk I/O out of the motion thread."""
    def __init__(self, path: Path):
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = path.open("x", encoding="utf-8")
        self.pending = queue.Queue(maxsize=4096)

    def submit(self, command):
        self.pending.put_nowait(command)

    def drain(self):
        while True:
            try:
                command = self.pending.get_nowait()
            except queue.Empty:
                break
            self.file.write(json.dumps(asdict(command)) + "\n")
        self.file.flush()

    def close(self):
        try:
            self.drain()
        finally:
            self.file.close()

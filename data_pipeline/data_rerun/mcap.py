from __future__ import annotations

import subprocess
from pathlib import Path


class McapRerunVisualizer:
    """使用 Rerun 官方 MCAP importer 打开 ROS 2 MCAP。

    子函数
    --------
    - view: 在 Rerun Viewer 中打开一个或多个 MCAP 文件.
    - convert: 将单个 MCAP 文件转换为 RRD.
    """

    def __init__(self, inputs: tuple[Path, ...]) -> None:
        """初始化 MCAP 可视化器。

        参数
        --------
        inputs : tuple[Path, ...]
            MCAP 文件或 rosbag2 MCAP 目录.

        返回
        --------
        None
            初始化实例状态.
        """
        self.paths = self._resolve(inputs)

    @staticmethod
    def _resolve(inputs: tuple[Path, ...]) -> tuple[Path, ...]:
        """解析输入中的全部 MCAP 文件。

        参数
        --------
        inputs : tuple[Path, ...]
            文件或目录输入.

        返回
        --------
        tuple[Path, ...]
            去重并排序的 MCAP 文件.
        """
        paths: list[Path] = []
        for input_path in inputs:
            if input_path.is_dir():
                paths.extend(sorted(input_path.glob("*.mcap")))
            elif input_path.is_file() and input_path.suffix == ".mcap":
                paths.append(input_path)
            else:
                raise FileNotFoundError(f"无效的 MCAP 输入: {input_path}")
        unique_paths = tuple(dict.fromkeys(path.resolve() for path in paths))
        if not unique_paths:
            raise FileNotFoundError("输入中没有 MCAP 文件")
        return unique_paths

    def view(self) -> None:
        """在 Rerun Viewer 中打开 MCAP 文件。

        返回
        --------
        None
            Viewer 关闭后返回.
        """
        subprocess.run(["rerun", *(str(path) for path in self.paths)], check=True)

    def convert(self, output_rrd: Path) -> Path:
        """使用 Rerun 官方 importer 将单个 MCAP 转换为 RRD。

        参数
        --------
        output_rrd : Path
            目标 RRD 文件路径.

        返回
        --------
        Path
            完成转换的 RRD 文件路径.
        """
        if len(self.paths) != 1:
            raise ValueError("MCAP 到 RRD 转换每次只接受一个输入文件")
        if output_rrd.exists():
            raise FileExistsError(f"输出文件已存在: {output_rrd}")
        output_rrd.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                "rerun",
                "mcap",
                "convert",
                str(self.paths[0]),
                "--output",
                str(output_rrd),
            ],
            check=True,
        )
        return output_rrd

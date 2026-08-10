from __future__ import annotations

import json
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class McapConversionConfig:
    """定义 rosbag2 到 MCAP 的容器转换配置。

    参数
    --------
    input_bag : Path
        rosbag2 输入目录.
    output_bag : Path
        输出 rosbag2 MCAP 目录.
    """

    input_bag: Path
    output_bag: Path


class Rosbag2ToMcapConverter:
    """通过 ROS 2 官方 bag convert 转换 MCAP storage。

    子函数
    --------
    - convert: 执行容器转换并返回输出目录.
    """

    def __init__(self, config: McapConversionConfig) -> None:
        """初始化 MCAP 转换器。

        参数
        --------
        config : McapConversionConfig
            输入与输出路径配置.

        返回
        --------
        None
            初始化实例状态.
        """
        self.config = config

    def convert(self) -> Path:
        """调用 ros2 bag convert 生成 MCAP rosbag2。

        返回
        --------
        Path
            包含 metadata.yaml 与 mcap 文件的输出目录.
        """
        if not self.config.input_bag.is_dir():
            raise NotADirectoryError(f"rosbag2 输入必须是目录: {self.config.input_bag}")
        if self.config.output_bag.exists():
            raise FileExistsError(f"输出路径已存在: {self.config.output_bag}")
        self.config.output_bag.parent.mkdir(parents=True, exist_ok=True)

        with tempfile.TemporaryDirectory(prefix="xiazhi_mcap_") as temp_dir:
            options_path = Path(temp_dir) / "convert.yaml"
            output_uri = json.dumps(str(self.config.output_bag))
            options_path.write_text(
                "output_bags:\n"
                f"  - uri: {output_uri}\n"
                "    storage_id: mcap\n"
                "    all: true\n",
                encoding="utf-8",
            )
            subprocess.run(
                [
                    "ros2",
                    "bag",
                    "convert",
                    "-i",
                    str(self.config.input_bag),
                    "-o",
                    str(options_path),
                ],
                check=True,
            )

        mcap_files = list(self.config.output_bag.glob("*.mcap"))
        if not mcap_files:
            raise RuntimeError(f"转换输出中没有 MCAP 文件: {self.config.output_bag}")
        return self.config.output_bag

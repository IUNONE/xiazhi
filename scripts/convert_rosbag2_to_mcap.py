from __future__ import annotations

import argparse
from pathlib import Path

from data_pipeline.data_format import McapConversionConfig, Rosbag2ToMcapConverter


def main() -> None:
    """解析路径参数并将 rosbag2 转换为 MCAP storage。

    返回
    --------
    None
        输出目录打印到标准输出.
    """
    parser = argparse.ArgumentParser(description="Convert rosbag2 storage to MCAP")
    parser.add_argument("--input-bag", type=Path, required=True)
    parser.add_argument("--output-bag", type=Path, required=True)
    args = parser.parse_args()

    output_path = Rosbag2ToMcapConverter(
        McapConversionConfig(
            input_bag=args.input_bag,
            output_bag=args.output_bag,
        )
    ).convert()
    print(output_path)


if __name__ == "__main__":
    main()

from __future__ import annotations

import argparse
import json
import tempfile
from pathlib import Path
from typing import Literal

from data_pipeline.data_format import McapConversionConfig, Rosbag2ToMcapConverter
from data_pipeline.data_rerun import (
    LeRobotRerunConfig,
    LeRobotRerunVisualizer,
    McapRerunVisualizer,
)

RerunInputKind = Literal["mcap", "rosbag2", "lerobot"]


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="自动识别输入类型并在 Rerun 中可视化 mcap, rosbag2, lerobot"
    )
    parser.add_argument(
        "path",
        type=Path,
        help="输入路径: mcap 文件, mcap rosbag2 目录, sqlite rosbag2 目录, 或 lerobot 数据集目录",
    )
    parser.add_argument("--output-rrd", type=Path, default=None, help="输出 RRD 路径")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="兼容旧入口, 对 lerobot 生效时表示输出目录",
    )
    parser.add_argument("--repo-id", default=None, help="lerobot 可视化需要的 repo id")
    parser.add_argument(
        "--episode-index",
        type=int,
        default=0,
        help="lerobot 可视化的 episode index",
    )
    parser.add_argument("--root", type=Path, default=None, help="lerobot root 目录, 默认使用输入目录")
    parser.add_argument("--batch-size", type=int, default=32, help="lerobot batch_size")
    parser.add_argument("--num-workers", type=int, default=0, help="lerobot num_workers")
    return parser.parse_args()


def _resolve_output_rrd(args: argparse.Namespace) -> Path | None:
    if args.output_rrd is not None and args.output_dir is not None:
        raise ValueError("请只传一个输出参数, --output-rrd 或 --output-dir")
    return args.output_rrd if args.output_rrd is not None else args.output_dir


def _detect_input_kind(path: Path) -> RerunInputKind:
    if not path.exists():
        raise FileNotFoundError(f"输入不存在: {path}")
    if path.is_file():
        if path.suffix.lower() == ".mcap":
            return "mcap"
        raise ValueError(f"不支持的输入文件类型: {path.suffix or path.name}")

    metadata_file = path / "metadata.yaml"
    has_db3 = any(path.glob("*.db3"))
    has_mcap = any(path.glob("*.mcap"))
    has_lerobot_info = (path / "meta" / "info.json").is_file()

    if metadata_file.is_file() and has_db3:
        return "rosbag2"
    if has_lerobot_info:
        return "lerobot"
    if has_mcap:
        return "mcap"
    if metadata_file.is_file():
        return "rosbag2"
    raise ValueError(f"无法识别的输入类型: {path}")


def _infer_lerobot_repo_id(root: Path) -> str:
    info_path = root / "meta" / "info.json"
    data = json.loads(info_path.read_text(encoding="utf-8"))
    repo_id = data.get("repo_id")
    if isinstance(repo_id, str) and repo_id.strip():
        return repo_id.strip()
    raise ValueError(f"无法从 meta/info.json 读取 repo_id: {info_path}")


def _visualize_mcap(path: Path, output_rrd: Path | None) -> None:
    visualizer = McapRerunVisualizer((path,))
    if output_rrd is None:
        visualizer.view()
        return
    result = visualizer.convert(output_rrd)
    print(result)


def _visualize_rosbag2(path: Path, output_rrd: Path | None) -> None:
    with tempfile.TemporaryDirectory(prefix="xiazhi_rerun_") as temp_dir:
        mcap_dir = Path(temp_dir) / "mcap"
        converted = Rosbag2ToMcapConverter(
            McapConversionConfig(input_bag=path, output_bag=mcap_dir)
        ).convert()
        _visualize_mcap(converted, output_rrd)


def _visualize_lerobot(
    path: Path,
    args: argparse.Namespace,
    output_rrd: Path | None,
) -> None:
    root = args.root if args.root is not None else path
    repo_id = args.repo_id or _infer_lerobot_repo_id(path)
    result = LeRobotRerunVisualizer(
        LeRobotRerunConfig(
            repo_id=repo_id,
            episode_index=args.episode_index,
            root=root,
            save=output_rrd is not None,
            output_dir=output_rrd,
            batch_size=args.batch_size,
            num_workers=args.num_workers,
        )
    ).visualize()
    if result is not None:
        print(result)


def main() -> None:
    args = _parse_args()
    input_path = args.path
    output_rrd = _resolve_output_rrd(args)
    kind = _detect_input_kind(input_path)

    if kind == "mcap":
        _visualize_mcap(input_path, output_rrd)
    elif kind == "rosbag2":
        _visualize_rosbag2(input_path, output_rrd)
    else:
        _visualize_lerobot(input_path, args, output_rrd)


if __name__ == "__main__":
    main()

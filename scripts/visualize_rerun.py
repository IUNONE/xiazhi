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
_DEFAULT_ROBOT_XACRO = (
    Path(__file__).resolve().parents[1]
    / "third_party/ros2_rm_robot/rm_description/urdf/rm_eco63.urdf.xacro"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="自动识别输入类型并在 Rerun 中可视化 mcap, rosbag2, lerobot"
    )
    parser.add_argument(
        "path",
        type=Path,
        nargs="?",
        help="输入路径: mcap 文件, mcap rosbag2 目录, sqlite rosbag2 目录, 或 lerobot 数据集目录",
    )
    directories = parser.add_mutually_exclusive_group()
    directories.add_argument("--mcap_datasets_dir", "--mcap-datasets-dir", type=Path,
                             help="递归浏览目录下所有 MCAP episode")
    directories.add_argument("--lerobot_datasets_dir", "--lerobot-datasets-dir", type=Path,
                             help="递归浏览 LeRobot 数据集中的所有 episode")
    parser.add_argument("--cache-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "outputs/rerun_cache",
                        help="目录浏览使用的 RRD 缓存目录")
    parser.add_argument("--memory-limit", default="50%", help="目录 Viewer 的内存上限，例如 4GB 或 50%")
    parser.add_argument(
        "--instruction", default=None,
        help="MCAP/rosbag2 Viewer 左上角显示的任务说明",
    )
    parser.add_argument("--eef-link", default="Link6", help="FK 末端 link，默认 Link6；位移单位米")
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
        default=None,
        help="lerobot 单 episode 索引，默认 0；目录模式查看全部",
    )
    parser.add_argument("--root", type=Path, default=None, help="lerobot root 目录, 默认使用输入目录")
    parser.add_argument("--batch-size", type=int, default=32, help="lerobot batch_size")
    parser.add_argument("--num-workers", type=int, default=0, help="lerobot num_workers")
    parser.add_argument(
        "--camera-name",
        action="append",
        default=[],
        help="MCAP 相机命名空间, 可重复提供. 默认从 metadata.yaml 自动读取",
    )
    parser.add_argument(
        "--no-robot-model",
        action="store_true",
        help="不加载 ECO63 模型和末端相机视锥（MCAP / LeRobot 均生效）",
    )
    args = parser.parse_args()
    directory_mode = args.mcap_datasets_dir is not None or args.lerobot_datasets_dir is not None
    if (args.path is not None) == directory_mode:
        parser.error("请提供单个 path，或 --mcap_datasets_dir / --lerobot_datasets_dir 之一")
    if directory_mode and any(value is not None for value in (
        args.output_rrd, args.output_dir, args.root, args.repo_id, args.episode_index,
    )):
        parser.error("目录模式不能使用输出参数、--root、--repo-id 或 --episode-index")
    if args.batch_size <= 0 or args.num_workers < 0:
        parser.error("batch-size 必须为正数，num-workers 不能为负数")
    if args.episode_index is not None and args.episode_index < 0:
        parser.error("episode-index 不能为负数")
    return args


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


def _visualize_mcap(
    path: Path,
    output_rrd: Path | None,
    args: argparse.Namespace,
) -> None:
    robot_xacro = None if args.no_robot_model else _DEFAULT_ROBOT_XACRO
    visualizer = McapRerunVisualizer(
        (path,),
        robot_xacro=robot_xacro,
        camera_names=tuple(args.camera_name),
        instruction=args.instruction,
        eef_link=args.eef_link,
    )
    if output_rrd is None:
        visualizer.view()
        return
    result = visualizer.convert(output_rrd)
    print(result)


def _visualize_rosbag2(
    path: Path,
    output_rrd: Path | None,
    args: argparse.Namespace,
) -> None:
    with tempfile.TemporaryDirectory(prefix="xiazhi_rerun_") as temp_dir:
        mcap_dir = Path(temp_dir) / "mcap"
        converted = Rosbag2ToMcapConverter(
            McapConversionConfig(input_bag=path, output_bag=mcap_dir)
        ).convert()
        _visualize_mcap(converted, output_rrd, args)


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
            episode_index=args.episode_index if args.episode_index is not None else 0,
            root=root,
            save=output_rrd is not None,
            output_dir=output_rrd,
            batch_size=args.batch_size,
            num_workers=args.num_workers,
            robot_xacro=None if args.no_robot_model else _DEFAULT_ROBOT_XACRO,
            eef_link=args.eef_link,
        )
    ).visualize()
    if result is not None:
        print(result)


def main() -> None:
    args = _parse_args()
    if args.mcap_datasets_dir is not None or args.lerobot_datasets_dir is not None:
        from data_pipeline.data_rerun.directory import view_directory
        kind = "mcap" if args.mcap_datasets_dir is not None else "lerobot"
        root = args.mcap_datasets_dir if kind == "mcap" else args.lerobot_datasets_dir
        view_directory(root, kind, args.cache_dir, {
            "robot_xacro": str(_DEFAULT_ROBOT_XACRO) if not args.no_robot_model else None,
            "camera_names": args.camera_name,
            "instruction": args.instruction,
            "eef_link": args.eef_link,
            "batch_size": args.batch_size,
            "num_workers": args.num_workers,
        }, args.memory_limit)
        return
    input_path = args.path
    output_rrd = _resolve_output_rrd(args)
    kind = _detect_input_kind(input_path)

    if kind == "mcap":
        _visualize_mcap(input_path, output_rrd, args)
    elif kind == "rosbag2":
        _visualize_rosbag2(input_path, output_rrd, args)
    else:
        _visualize_lerobot(input_path, args, output_rrd)


if __name__ == "__main__":
    main()

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True, slots=True)
class LeRobotRerunConfig:
    """定义 LeRobot episode 的 Rerun 可视化配置。

    参数
    --------
    repo_id : str
        LeRobotDataset repository id.
    episode_index : int
        需要可视化的 episode 索引.
    root : Path | None
        可选的本地 LeRobotDataset 根目录.
    save : bool
        是否保存为 RRD 而不打开 Viewer.
    output_dir : Path | None
        RRD 输出目录.
    """

    repo_id: str
    episode_index: int
    root: Path | None = None
    save: bool = False
    output_dir: Path | None = None
    batch_size: int = 32
    num_workers: int = 0


class LeRobotRerunVisualizer:
    """复用 LeRobot 官方 Rerun visualizer 显示 episode。

    子函数
    --------
    - visualize: 加载数据集并启动官方可视化流程.
    """

    def __init__(self, config: LeRobotRerunConfig) -> None:
        """初始化 LeRobot 可视化器。

        参数
        --------
        config : LeRobotRerunConfig
            数据集与 episode 配置.

        返回
        --------
        None
            初始化实例状态.
        """
        if config.episode_index < 0:
            raise ValueError("episode_index 不允许为负数")
        if config.save and config.output_dir is None:
            raise ValueError("保存 RRD 时必须提供 output_dir")
        if config.batch_size <= 0 or config.num_workers < 0:
            raise ValueError("batch_size 或 num_workers 非法")
        self.config = config

    def visualize(self) -> Path | None:
        """调用 LeRobot 官方可视化函数显示或保存 episode。

        返回
        --------
        Path | None
            保存模式返回 RRD 路径. Viewer 模式返回 None.
        """
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
            from lerobot.scripts.lerobot_dataset_viz import visualize_dataset
        except ImportError as exc:
            raise ImportError(
                "需要安装 third_party/lerobot 的 dataset_viz 依赖后再执行可视化"
            ) from exc

        dataset = LeRobotDataset(
            self.config.repo_id,
            episodes=[self.config.episode_index],
            root=self.config.root,
        )
        result = visualize_dataset(
            dataset=dataset,
            episode_index=self.config.episode_index,
            batch_size=self.config.batch_size,
            num_workers=self.config.num_workers,
            mode="local",
            save=self.config.save,
            output_dir=self.config.output_dir,
            display_mode="rerun",
        )
        return Path(result) if result is not None else None

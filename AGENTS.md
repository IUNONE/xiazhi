# XiaZhi-2026 Agent Guide

当前数据管线负责人为 zsh. 开发分支为 `zsh`.

## 1. 当前范围

数据管线当前只实现四个领域:

1. `data_collection`: 从 RealMan ECO63 一体化六维力机械臂和三台 Orbbec Gemini 335 采集 rosbag2 或直接采集 LeRobotDataset.
2. `data_format`: 将 rosbag2 转换为 MCAP storage 或 LeRobotDataset.
3. `data_rerun`: 使用 Rerun 可视化 MCAP 或 LeRobotDataset.
4. `data_curation`: 数据管理预留模块. 当前不实现.

禁止在未确认需求前恢复 SampleRef DataPlan sampler ranking HF Hub StreamingDataset WebDataset 等旧功能.

## 2. 设计边界

- RealMan Orbbec LeRobot 均使用 `third_party` 中的官方 Git submodule.
- 项目代码只负责启动编排 topic 映射 格式适配和可视化调用.
- 不复制官方 SDK 驱动 LeRobotDataset writer 或 Rerun importer.
- action 必须与部署控制接口语义一致并记录来源. 拖动示教没有独立 action 流时，当前统一使用下一采样时刻的绝对关节目标，最后一个 observation 丢弃.
- 所有 CLI 入口放在 `scripts/`. `data_pipeline` 内不解析命令行.
- 一个 rosbag2 目录对应一个 episode.
- 一次采集命令对应一个 session. ROS session 含多个 rosbag2 目录，LeRobot session 是含多个 episode 的一个数据集.
- 输入和派生数据不覆盖. 输出路径已存在时应直接报错.

## 3. 仓库结构

```text
XiaZhi-2026/
├── data_pipeline/
│   ├── data_collection/
│   ├── data_format/
│   ├── data_rerun/
│   └── data_curation/
├── scripts/
│   ├── collect_data_ros2.py
│   ├── collect_data_lerobot.py
│   ├── convert_rosbag2_to_mcap.py
│   ├── convert_rosbag2_to_lerobot.py
│   └── visualize_rerun.py
└── third_party/
    ├── ros2_rm_robot/
    ├── OrbbecSDK_ROS2/
    └── lerobot/
```

## 4. 模块职责

| 模块 | 当前职责 |
|---|---|
| `data_collection/config.py` | 三相机 多 episode session 和 RealMan 拖动配置 |
| `data_collection/ros2_collector.py` | 官方设备 launch 编排 topic 检查和多 rosbag2 episode 录制 |
| `data_collection/lerobot_collector.py` | SDK 直连采样和多 episode LeRobot writer 编排 |
| `data_format/rosbag2.py` | rosbag2 CDR 读取 数值字段提取和 RGB/depth 解码 |
| `data_format/mcap.py` | ROS 2 官方 bag convert 调用 |
| `data_format/lerobot.py` | observation 对齐 下一关节目标 action 构造和官方 LeRobot writer 调用 |
| `data_rerun/mcap.py` | Rerun 官方 MCAP importer 调用 |
| `data_rerun/lerobot.py` | LeRobot 官方 dataset visualizer 调用 |
| `data_curation/` | 当前仅保留 README |

## 5. 环境与依赖

- 设备与 rosbag2 功能运行在 Ubuntu 22.04 和 ROS 2 Humble.
- ROS 2 入口使用 Humble 系统 Python 3.10. 最新 LeRobot 入口使用 Python 3.12 或更高，二者使用独立环境.
- RealMan 和 Orbbec package 需要在 ROS 2 workspace 中完成编译并 source.
- `rosbag2_py` 和 rosbag2 storage plugin 使用 ROS 2 系统 package. 不通过 PyPI 安装.
- LeRobot 直采环境使用以下方式安装:

```bash
git submodule update --init --recursive
conda env create -f env.yaml
conda activate xiazhi
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
```

## 6. 数据规则

- 原始 rosbag2 MCAP DB3 图像 视频 LeRobot 数据集和 RRD 不进入普通 Git history.
- `.gitignore` 忽略 raw processed datasets outputs build install log 等目录.
- `.gitattributes` 为需要纳入仓库的 MCAP PDF 和 assets 视频配置 Git LFS.
- 三台相机必须通过唯一 serial number 绑定到 `camera_01` `camera_02` `camera_03`.

## 7. 检查

用户未要求时不运行项目代码. 不使用 pytest ruff black 等测试或格式工具，
仅进行源码阅读和不会连接硬件的静态语法检查.

```bash
git diff --check
```

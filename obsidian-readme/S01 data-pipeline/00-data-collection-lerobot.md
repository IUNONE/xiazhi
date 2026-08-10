# 00 LeRobot 数据采集

## 1. 环境

在仓库根目录初始化 submodule：

```bash
git clone --recurse-submodules https://github.com/IUNONE/xiazhi.git
cd xiazhi
```

创建并激活 Python 3.12 环境：

```bash
conda env create -f env.yaml
conda activate xiazhi
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
```

## 2. 运行前检查

1. 操作者在机械臂旁，急停可立即触及，返航路径无人员和障碍物。
2. ECO63 是一体化六维力版本，控制器无报错。
3. 三台 Gemini 335 的 serial number 已确定，并固定对应 `camera_01`、`camera_02`、`camera_03`。
4. 输出 session 目录不存在；脚本不会覆盖或自动删除旧数据。
5. 在交互式 TTY 中运行。管道、后台任务和无终端环境不会自动开始录制。

## 3. 启动采集

```bash
conda activate xiazhi
cd /path/to/XiaZhi-2026
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

python scripts/collect_data_lerobot.py \
  --output-root /data/lerobot \
  --repo-id local/xiazhi_pick \
  --task 'pick the object' \
  --scene table_a \
  --session-id session_000001 \
  --robot-ip 192.168.1.18 \
  --camera-serial <SN_1> \
  --camera-serial <SN_2> \
  --camera-serial <SN_3> \
  --fps 30
```

默认写入三路 RGB 和三路 uint16 毫米深度视频。使用 `--no-depth` 可以关闭深度，使用 `--no-videos` 可以改为逐帧图像。

## 4. 按键

```text
连接机械臂和三台相机
  -> 检查状态和新鲜 RGB-D 图像
  -> 保存 session 初始关节位置
  -> 开启重力补偿拖动示教
  -> READY：等待 r / q / Ctrl-C
  -> r：开始新 episode
  -> RECORDING：持续采样
  -> s：停止并保存当前 episode，返航后回到 READY
  -> q / Ctrl-C：停止并保存当前 episode，返航后结束 session
```

在 READY 状态按 `q` 或 `Ctrl-C` 会直接返航并结束 session。按 `s` 后返回 READY，不会连续录制。

采集或返航失败时，episode 和 session 会标为 failed，已有数据不会自动删除。

## 5. 输出

一次命令创建一个多 episode LeRobotDataset：

```text
<output-root>/<task>/<scene>/<session-id>/
├── data/
├── meta/
├── videos/                 # --no-videos 时可能不存在
└── session_manifest.json
```

episode 索引依次为 `0, 1, ...`。`session_manifest.json` 记录设备配置、episode 状态、帧数、停止原因和错误。

## 6. 数据字段

| feature | shape | 单位/格式 |
|---|---:|---|
| `observation.state` | 6 | 关节位置，rad |
| `observation.ee_pose` | 7 | `x y z qw qx qy qz`，m + quaternion |
| `observation.force` | 6 | `fx fy fz mx my mz`，N + Nm |
| `observation.images.camera_01..03` | H×W×3 | RGB uint8 |
| `observation.images.camera_01_depth..03_depth` | H×W×1 | depth uint16，mm |
| `action` | 6 | 下一采样时刻的绝对关节目标，rad |

```text
action(t) = observation.state(t + 1)[0:6]
```

每个 episode 的最后一个 observation 会被丢弃，不跨 episode 配对。

## 7. MCAP

当前不支持 LeRobotDataset 转 MCAP。需要完整 ROS topic 时使用 [ROS 2/MCAP 采集](01-data-collection-ros2-mcap.md)。

## 8. 常用参数

```bash
python scripts/collect_data_lerobot.py --help
```

- `--session-id`：session 目录名；省略时生成 UUID。
- `--drag-mode 3`：位置和姿态同时拖动，默认值。
- `--disable-singular-wall`：关闭奇异墙，不建议在常规采集使用。
- `--home-return-speed 10`：返航速度百分比。
- `--home-tolerance-deg 1.0`：返航后最大关节误差。
- `--max-camera-age-ms`：允许使用的最大图像陈旧时间。
- `--camera-width`、`--camera-height`、`--fps`：直采图像和采样规格。

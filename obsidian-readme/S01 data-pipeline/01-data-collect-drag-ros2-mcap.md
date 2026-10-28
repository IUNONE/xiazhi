
```bash
TASK_NAME="pick"
OUTPUT_ROOT="$HOME/datasets"
SCENE="Lab"
ID="000001"

/usr/bin/python3 scripts/collect_data_ros2.py \
  --output-root "${OUTPUT_ROOT}" \
  --task "${TASK_NAME}" \
  --scene "${SCENE}" \
  --session-id "session_${ID}" \
  --robot-ip 192.168.1.18 \
  --robot-port 8080 \
  --camera-serial YOUR_SN_1 \
  --camera-serial YOUR_SN_2 \
  --camera-serial YOUR_SN_3 \
  --drag-mode 3 \
  --force-drag-mode precise \
  --capture-mode teach-replay \
  --storage-id mcap \
  --init-joints='[0,60,-135,75,90,0]'
  
```

机器人地址须与驱动配置一致，输出 session 目录不能已存在。
根据终端提示，按顺序操作：

| 阶段 | 按键 | 行为 |
|---|---|---|
| 准备示教 | `t` | 开启拖动示教 |
| 示教结束 | `s` | 保存轨迹，此时尚未录包 |
| 人员离开工作区 | `o` | 机械臂运动到示教起点 |
| 场景复位完成 | `r` | 回放轨迹并录包 |
| 放弃本次示教 | `d` | 在等待回放时废弃轨迹，不创建 episode |
| 回放中止 | `s` | 停止回放，保留录包并标为 `failed` |
| episode 结束 | `h` | 确认路径安全后返航，继续下一次示教 |
| 结束会话 | `q` / `Ctrl-C` | 停止并退出，不自动返航 |

参数说明：

| 参数                                    | 作用                                 |
| ------------------------------------- | ---------------------------------- |
| `--init-joints='[0,60,-135,75,90,0]'` | 启动后主动运动到指定六关节角，单位 degree；不填则使用当前位姿 |
| `--force-drag-mode fast/precise`      | 快速/精准拖动，仅对 `--drag-mode 1/2/3` 生效  |
| `--replay-pre-roll-s 1.0`             | 回放前录包时长，仅用于 `teach-replay`         |
| `--replay-post-roll-s 0.5`            | 回放后录包时长，仅用于 `teach-replay`         |
| `--camera-fps 30`                     | 相机帧率                               |
| `--no-depth`                          | 不采集深度                              |
| `--extra-topic /some/topic`           | 追加录制 topic，可重复提供                   |

输出：一次命令创建一个 session，每个 `episode_XXXXXX` 是独立录包：

```text
$HOME/datasets/pick/Lab/session_000001/
├── session_manifest.json
├── trajectories/             # teach-replay 的示教轨迹
├── episode_000000/
│   ├── metadata.yaml
│   └── *.mcap
└── episode_000001/
```

在 `session_manifest.json` 中确认目标 episode 为 `completed`。被废弃的示教保留轨迹，但不占用 episode 编号。

接下来：[转换为 LeRobot](03-data-collection-mcap2lerobot.md) · [Rerun 查看与验收](04-data-collect-rerun-visualization.md)。

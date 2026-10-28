## 1. 零位标定与实时预览

连接主臂 USB 串口后，查看设备：

```bash
ls -l /dev/serial/by-id/
```


```bash
LEADER_PORT="YOUR_LEADER"

python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --calibration configs/teleoperation/zhongling_eco63.example.json \
  --calibrate configs/teleoperation/zp10d_eco63.calibrated.json \
  --zero-joints='[0,0,0,0,0,0]' \
  --reference-joints='[30,30,30,30,30,30]' \
  --viewer
```

按终端提示：

1. 扶稳主臂，按 Enter 卸力。
2. 对照 Viewer 摆到零位，静止后按 Enter。
3. 摆到标定位，静止后按 Enter。
4. 保存标定文件后，移动主臂检查虚拟 ECO63 和六轴曲线。

仅检查已有标定：

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --calibration configs/teleoperation/zp10d_eco63.calibrated.json \
  --unlock-leader \
  --viewer
```

## 2. 启动采集

替换机器人地址、相机 serial 和主臂路径：

```bash
TASK_NAME="pick"
OUTPUT_ROOT="$HOME/datasets"
SCENE="Lab"
ID="000001"
LEADER_PORT=""

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
  --storage-id mcap \
  --teaching-mode leader-follower \
  --capture-mode direct \
  --leader-port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-calibration configs/teleoperation/zp10d_eco63.calibrated.json \
  --leader-baudrate 115200 \
  --control-hz 30 \
  --init-joints='[0,60,-135,75,90,0]'
```


| 按键 | 行为 |
|---|---|
| `i` | 暂停跟随，先让主臂回到标定 init，再独立控制从臂回位，不录包 |
| `r` | 回位后提示扶好主臂；按下后卸力、开始跟随并录制 |
| `s` | 保存当前 episode；主臂仍卸力，主从继续跟随 |
| 再次 `i` | 暂停跟随并重新回位，准备下一轮 `r → s` |
| `q` | 停止跟随，保存正在录制的 episode 后退出，不自动返航 |
| `Ctrl-C` | 停止并退出，未完成 episode 标记失败 |

输出：

```text
$HOME/datasets/pick/Lab/session_000001/
├── session_manifest.json
├── teleoperation/
│   └── episode_000000.jsonl   # 主臂位置与提交的目标
└── episode_000000/
    ├── metadata.yaml
    └── *.mcap
```


接下来：[转换为 LeRobot](03-data-collection-mcap2lerobot.md) · [Rerun 查看与验收](04-data-collect-rerun-visualization.md)。

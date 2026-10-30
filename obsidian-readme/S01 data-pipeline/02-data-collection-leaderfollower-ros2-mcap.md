
主从臂采用 gello 设计，参考: https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm

## 1. 零位标定与实时预览

连接主臂 USB 串口后，查看设备：

```bash
ls -l /dev/serial/by-id/
```

```bash
cd xiazhi
source /opt/ros/humble/setup.zsh
source ${HOME}/xiazhi/install/setup.zsh
source ${HOME}/xiazhi/.venv/bin/activate

export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

LEADER_PORT="usb-1a86_USB_Serial-if00-port0"
```

### 可选：统一调整主臂／从臂的零位和标定位


```bash
# U-Arm 主臂零位, 初始 [0,0,0,0,0,0]
# configs/teleoperation/leader.zero.json
python scripts/arm_joints.py --arm leader --pose zero

# U-Arm 主臂标定位, [90,90,-90,0,0,90]
# configs/teleoperation/leader.reference.json
python scripts/arm_joints.py --arm leader --pose reference

# ECO63 从臂零位, [0,0,0,0,0,0]
# configs/teleoperation/follower.zero.json
python scripts/arm_joints.py --arm follower --pose zero
```


## 1.1 标定主臂零位和正方向

根据终端和 rerun 画面提示，将 主臂依次摆到 零位 和 标定位。

```bash
python scripts/inspect_leader.py \
	--port "/dev/serial/by-id/${LEADER_PORT}" \
	--leader-config configs/teleoperation/zhongling_eco63.example.json \
	--calibrate-model \
	--model-reference-joints='[90,90,-90,0,0,90]' \
	--model-calibration-output configs/teleoperation/uarm.model.calibrated.json \
	--viewer \
	--overwrite
```

默认零位：

![主臂零位模型与实物对照|265](../assets/zero.png)

检查标定文件，观察初始位置和正方向一致性。

```bash
python scripts/inspect_leader.py \
	--port "/dev/serial/by-id/${LEADER_PORT}" \
	--leader-config configs/teleoperation/zhongling_eco63.example.json \
	--model-calibration-input configs/teleoperation/uarm.model.calibrated.json \
	--viewer
```


## 1.2 主从映射配置


```bash
python scripts/arm_joints.py \
  --calibrate_leader_follwer \
  --output configs/teleoperation/leader_follower.mapping.json \
  --overwrite
```

连接真实主臂检查完整映射：

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-config configs/teleoperation/zhongling_eco63.example.json \
  --model-calibration-input configs/teleoperation/uarm.model.calibrated.json \
  --leader-follower-mapping configs/teleoperation/leader_follower.mapping.json \
  --viewer
```


带力矩的回位置测试:

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-config configs/teleoperation/zhongling_eco63.example.json \
  --model-calibration-input configs/teleoperation/uarm.model.calibrated.json \
  --leader-follower-mapping configs/teleoperation/leader_follower.mapping.json \
  --follower_init_pose='[0,60,-135,75,90,0]' \
  --viewer
```

## 2. 启动采集

替换机器人地址、相机 serial 和主臂路径：

```bash
TASK_NAME="test"
OUTPUT_ROOT="$HOME/datasets"
SCENE="Lab"
ID="000001"

python scripts/collect_data_ros2.py \
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
  --leader-config configs/teleoperation/zhongling_eco63.example.json \
  --leader-model-calibration configs/teleoperation/uarm.model.calibrated.json \
  --leader-follower-mapping configs/teleoperation/leader_follower.mapping.json \
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

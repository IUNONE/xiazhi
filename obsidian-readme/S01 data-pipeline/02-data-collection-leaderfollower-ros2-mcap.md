
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


```bash
# 检查相机序列号

ros2 run orbbec_camera list_devices_node

ARM_CAM_SN_1="CP0N763000ER"
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

观察 rerun 窗口， 调整主从臂之间的零位偏置和正负方向关系（默认主臂固定，调整从臂的相对偏置来让二者运动一致）：

```bash
python scripts/arm_joints.py \
  --calibrate_leader_follwer \
  --output configs/teleoperation/leader_follower.mapping.json \
  --overwrite
```

![主从映射零位偏置|277](../assets/leader_follower_offset.png)


连接真实主臂检查完整映射：

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-config configs/teleoperation/zhongling_eco63.example.json \
  --model-calibration-input configs/teleoperation/uarm.model.calibrated.json \
  --leader-follower-mapping configs/teleoperation/leader_follower.mapping.json \
  --viewer
```

![[assets/leader_follower_rerun.webm]]

带力矩的回位置测试: （ TODO:  未测试成功，疑似主臂舵机供电功率不足）

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

本机 ip 设置为 `192.168.1.100`，机械臂默认 ip 为 `192.168.1.18`

可在浏览器打开 `192.168.1.18` 进入机器人管理器后台 ( 用户名：user 密码：123 )

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
  --camera-serial "${ARM_CAM_SN_1}" \
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
  --speed 20 \
  --viewer
  # --init-joints='[0,60,-135,75,90,0]'
```


| 按键 | 行为 |
|---|---|
| `i` | 暂停跟随；无 `--init-joints` 时仅让从臂对齐按键时的主臂姿态；有该参数时按原流程主从回位。不录包 |
| `r` | 从臂对齐或回位后，扶稳主臂；按下后卸力、开始跟随并录制 |
| `s` | 保存当前 episode；主臂仍卸力，主从继续跟随 |
| 再次 `i` | 暂停跟随，按当前模式重新对齐或回位，准备下一轮 `r → s` |
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

![[assets/tele.webm]]

接下来：[转换为 LeRobot](03-data-collection-mcap2lerobot.md) · [Rerun 查看与验收](04-data-collect-rerun-visualization.md)。

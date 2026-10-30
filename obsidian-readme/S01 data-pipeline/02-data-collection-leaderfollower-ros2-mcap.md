
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

### 可选：用滑块选择模型零位/标定位

```bash
python scripts/adjust_leader_zero.py \
  --output configs/teleoperation/uarm.desired-zero.json \
  --overwrite
```

保存的 json 文件用于记录相对于当前 URDF 的六轴角供后续修改 URDF 零位使用。重新打开同一姿态：

```bash
python scripts/adjust_leader_zero.py \
  --input configs/teleoperation/uarm.desired-zero.json
```

## 1.1 Rerun 标定主臂零位和正方向

本步骤读取实物编码器，标定其与当前 U-Arm URDF 的对应关系。主臂已卸力，程序不发送卸力或运动命令。采样期间 Rerun 固定显示参考姿态，实物运动不会改变画面；完成后再恢复实物驱动模型。

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-config configs/teleoperation/zhongling_eco63.example.json \
  --calibrate-model \
  --model-reference-joints='[90,90,-90,0,0,90]' \
  --model-calibration-output configs/teleoperation/uarm.model.calibrated.json \
  --viewer
```

依次完成三次确认（直接按 `y`，无需 Enter）：

1. Rerun 显示当前 URDF 的零位 `[0,0,0,0,0,0]`。参考画面尽量准确摆好实物 U-Arm，保持静止，按 `y` 记录六轴零位。
2. Rerun 显示标定位 `[90,90,-90,0,0,90]`。将 joint 1、2、3、6 从已记录零位沿画面所示方向明显转动，不需要精确到 90° 或 −90°。保持静止，按 `y` 记录方向；joint 3 的示例为负方向，程序会据此换算正方向。
3. Rerun 显示补充标定位 `[0,0,0,30,30,0]`。将 joint 4、5 从已记录零位向画面中的正方向转动，不需要精确到 30°，静止后按 `y` 记录方向。其余关节可参考画面回到零位，不参与本次方向计算。

joint 4、5 在前两组姿态中都为 0°，不能由两次读数推算其方向，所以需要第三次采样。角度均相对于当前 URDF 零位：此前设置的 joint 1=-10°、joint 2=+55°、joint 3=-150° 已体现在 URDF 中，不需要在命令中重复叠加。

只有第一次零位需要尽量准确。第二、三次的参考角度只是方向示例，程序只比较相对零位的读数增减与示例角度的正负号，不用示例角度拟合比例。角度换算始终沿用设备配置的 `raw_min`、`raw_max` 和 `angle_range_deg`，新标定文件的额外比例固定为 1。保存后恢复实时跟随。文件包含 URDF 校验值；改动 URDF 后需要重新标定。输出文件已存在时不会覆盖，请改用新文件名。

零位采样检查六个机械臂关节，不包括夹爪；第二次只检查 joint 1、2、3、6，第三次只检查 joint 4、5。读数不稳定或参考读数变化不足时提示重试当前姿态；按 `q` 或 Ctrl-C 退出，未完成时不保存部分结果。Rerun 提示使用英文，终端提示使用中文。

预览标定结果：

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-config configs/teleoperation/zhongling_eco63.example.json \
  --model-calibration-input configs/teleoperation/uarm.model.calibrated.json \
  --viewer
```

## 1.2 检查 eco63 零位正方向

先确认 ECO63 的关节编号、零位定义和正方向，再进行主从映射。U-Arm 的 CAD 零位不等于 ECO63 的零位；1.1 保存的模型标定文件不能用作从臂控制标定。

1. 对照 ECO63 的机械臂说明和示教器关节读数，确认 `joint1`～`joint6` 与实物各轴一一对应。
2. 确认各关节零角度对应的连杆姿态，并核对正角度增加时的实际旋转方向。需要实机点动核对时，通过示教器逐轴小幅操作，记录结果。
3. 确定下一步用于主从映射的两组已知关节角。示例使用全 0° 和全 +30°，应按实际可用姿态替换；两组角度的每一轴至少相差 5°。

`inspect_leader.py` 只读取 U-Arm 并显示虚拟 ECO63，不连接或驱动真实 ECO63，也不能自动确认实机零位。下一步 Viewer 显示的从臂目标姿态仅用于核对映射，不能替代示教器读数。若实机与 ECO63 模型的轴向或零位定义不一致，应先查清原因再进行映射。

## 1.3 标定主臂从臂正方向

完成 1.1 的 U-Arm 模型标定和 1.2 的 ECO63 检查后，再进行主从两姿态映射。这里的 `--calibrate` 输出主从映射文件，`--model-calibration-input` 加载已有 U-Arm 模型标定，用于显示主臂自身姿态。

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-config configs/teleoperation/zhongling_eco63.example.json \
  --model-calibration-input configs/teleoperation/uarm.model.calibrated.json \
  --calibrate configs/teleoperation/zp10d_eco63.calibrated.json \
  --zero-joints='[0,0,0,0,0,0]' \
  --reference-joints='[30,30,30,30,30,30]' \
  --viewer
```

按终端提示：

1. 扶稳主臂，按 `t` 卸力（无需 Enter）。
2. 将主臂摆到希望映射为 `--zero-joints` 的姿态，静止后按 Enter。这里定义的是主从映射基准，不会重写 1.1 的 U-Arm 模型零位。
3. 将主臂摆到希望映射为 `--reference-joints` 的姿态，静止后按 Enter。程序用这两组读数计算各轴映射方向和比例。
4. 保存 `zp10d_eco63.calibrated.json` 后，逐轴移动主臂，检查虚拟 ECO63 的变化方向和角度是否符合预期，再用于第 2 节采集。

检查已有模型标定和主从映射：

```bash
python scripts/inspect_leader.py \
  --port "/dev/serial/by-id/${LEADER_PORT}" \
  --leader-config configs/teleoperation/zp10d_eco63.calibrated.json \
  --model-calibration-input configs/teleoperation/uarm.model.calibrated.json \
  --unlock-leader \
  --viewer
```

两个输出文件分别使用：`uarm.model.calibrated.json` 用于 U-Arm 模型显示，`zp10d_eco63.calibrated.json` 用于主从映射及采集命令的 `--leader-calibration`。标定输出均不覆盖已有文件，重新标定时请指定新文件名并同步后续加载路径。

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

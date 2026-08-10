# 01 ROS 2、rosbag2 和 MCAP

## 1. 数据链路

| 格式 | 项目用途 | 是否保留完整 ROS topic |
|---|---|---|
| rosbag2 + MCAP storage | 默认原始采集格式 | 是 |
| rosbag2 + SQLite3 storage | 可选原始采集格式 | 是 |
| MCAP rosbag2 | SQLite3 bag 转换后的统一 storage | 是 |
| LeRobotDataset | 从单个 bag episode 派生的训练格式 | 否 |

一个 rosbag2 目录对应一个 episode。一次 `collect_data_ros2.py` 命令对应一个 session，并在 session 目录下创建多个独立 rosbag2 episode。

## 2. 初始化 submodule

```bash
cd /path/to/XiaZhi-2026
git submodule sync --recursive
git submodule update --init --recursive
```

## 3. 官方编译说明页

### 3.1 RealMan

- [本地官方中文 README](../../third_party/ros2_rm_robot/README_CN.md)
- [官方 Humble 在线 README](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/README_CN.md)
- [本地 rm_driver 说明](../../third_party/ros2_rm_robot/rm_driver/README_CN.md)
- [官方 rm_driver 在线说明](https://github.com/RealManRobot/ros2_rm_robot/blob/humble/rm_driver/README_CN.md)

### 3.2 Orbbec

- [本地官方中文 README](../../third_party/OrbbecSDK_ROS2/README_CN.MD)
- [官方 v2-main 在线 README](https://github.com/orbbec/OrbbecSDK_ROS2/blob/v2-main/README_CN.MD)
- [官方 OrbbecSDK ROS2 文档](https://orbbec.github.io/OrbbecSDK_ROS2/zh/index.html)

## 4. ROS 2 Humble 环境

运行平台：Ubuntu 22.04 + ROS 2 Humble。

```bash
sudo apt update
sudo apt install \
  ros-humble-desktop \
  ros-humble-moveit \
  ros-humble-controller-interface \
  ros-humble-controller-manager-msgs \
  ros-humble-controller-manager \
  ros-humble-rosbag2-storage-mcap \
  python3-colcon-common-extensions \
  python3-rosdep
```

安装 RealMan Python SDK：

```bash
/usr/bin/python3 -m pip install --user robotic-arm==1.1.5
```

## 5. 编译 RealMan 和 Orbbec

### 5.1 准备同一个 colcon workspace

```bash
mkdir -p ~/xiazhi_ros2_ws/src
ln -s /path/to/XiaZhi-2026/third_party/ros2_rm_robot \
  ~/xiazhi_ros2_ws/src/ros2_rm_robot
ln -s /path/to/XiaZhi-2026/third_party/OrbbecSDK_ROS2 \
  ~/xiazhi_ros2_ws/src/OrbbecSDK_ROS2
```

### 5.2 安装 RealMan 本地库

```bash
cd /path/to/XiaZhi-2026/third_party/ros2_rm_robot/rm_driver/lib
sudo bash lib_install.sh
```

执行前检查 `lib_install.sh`；脚本会写入 `/usr/local/lib`。

### 5.3 安装 Orbbec 构建依赖

```bash
source /opt/ros/humble/setup.bash

sudo apt install \
  libgflags-dev nlohmann-json3-dev \
  ros-$ROS_DISTRO-image-transport \
  ros-${ROS_DISTRO}-image-transport-plugins \
  ros-${ROS_DISTRO}-compressed-image-transport \
  ros-$ROS_DISTRO-image-publisher \
  ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-diagnostic-updater \
  ros-$ROS_DISTRO-diagnostic-msgs \
  ros-$ROS_DISTRO-statistics-msgs \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-backward-ros \
  libdw-dev libssl-dev mesa-utils libgl1 libgoogle-glog-dev
```

### 5.4 解析依赖并编译

```bash
source /opt/ros/humble/setup.bash
cd ~/xiazhi_ros2_ws

rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

colcon build --packages-select rm_ros_interfaces
source install/setup.bash
colcon build \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

```bash
source /opt/ros/humble/setup.bash
source ~/xiazhi_ros2_ws/install/setup.bash
```

### 5.5 安装 Orbbec udev 规则

```bash
cd /path/to/XiaZhi-2026/third_party/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

重新插拔相机并检查设备：

```bash
source /opt/ros/humble/setup.bash
source ~/xiazhi_ros2_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node
```

## 6. RealMan 配置和运行前检查

修改源码中的：

```text
third_party/ros2_rm_robot/rm_driver/config/rm_eco63_config.yaml
```

`arm_ip` 和 `tcp_port` 必须与 `--robot-ip`、`--robot-port` 一致。修改后重新编译并 source workspace。

1. 操作者可立即触及急停，返航路径没有人员和障碍物。
2. ECO63 控制器无报错，机械臂固定可靠。
3. 三台 Gemini 335 的 serial number 唯一且顺序固定。
4. `ros2 pkg prefix rm_bringup` 和 `ros2 pkg prefix orbbec_camera` 均成功。
5. 输出 session 目录不存在。
6. 已在当前控制器上验证 ROS driver 与 Python SDK 可以安全并发连接。

## 7. rosbag2/MCAP 采集

```bash
source /opt/ros/humble/setup.bash
source ~/xiazhi_ros2_ws/install/setup.bash
cd /path/to/XiaZhi-2026
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

python3 scripts/collect_data_ros2.py \
  --output-root /data/raw \
  --task 'pick the object' \
  --scene table_a \
  --session-id session_000001 \
  --robot-ip 192.168.1.18 \
  --camera-serial <SN_1> \
  --camera-serial <SN_2> \
  --camera-serial <SN_3> \
  --storage-id mcap \
  --allow-sdk-driver-coexistence
```

只有完成 ROS driver 和 Python SDK 并发连接的真机安全验证后，才能使用 `--allow-sdk-driver-coexistence`。

默认录制：

- `/joint_states`、末端位姿、原始/零点六维力、错误状态和 TF；
- 三台相机的 RGB、depth 和 camera info；
- MCAP rosbag2 storage。使用 `--storage-id sqlite3` 可改为 DB3；
- 使用 `--extra-topic /some/topic` 可重复添加其他 topic。

## 8. 多 episode 状态机和输出

```text
启动官方驱动并检查 topic 类型和新鲜消息
  -> 保存 session 初始关节位置
  -> 开启重力补偿拖动示教
  -> READY：等待 r / q / Ctrl-C
  -> r：启动新的 ros2 bag record
  -> s：停止并保存当前 bag，返航后回到 READY
  -> q / Ctrl-C：停止并保存当前 bag，返航后结束 session
```

按 `s` 后返回 READY；下一个 recorder 只在再次按 `r` 后启动。

```text
<output-root>/<task>/<scene>/<session-id>/
├── session_manifest.json
├── episode_000000/          # 独立 rosbag2 目录
├── episode_000001/
└── ...
```

目录存在时立即报错。返航不会进入 bag。

## 9. SQLite3 rosbag2 转 MCAP

使用 SQLite3 storage 时执行：

```bash
python3 scripts/convert_rosbag2_to_mcap.py \
  --input-bag /data/raw/pick/table_a/session_000001/episode_000000 \
  --output-bag /data/processed/pick/table_a/session_000001/episode_000000_mcap
```

输入、输出必须是不同目录，输出目录必须不存在。

## 10. MCAP/rosbag2 转 LeRobotDataset

输入必须是包含 `metadata.yaml` 的 rosbag2 episode 目录，storage 可以是 `mcap` 或 `sqlite3`。

三路 RGB 必须显式映射，depth 可选：

```bash
python scripts/convert_rosbag2_to_lerobot.py \
  --input-bag /data/raw/pick/table_a/session_000001/episode_000000 \
  --output-root /data/lerobot/pick/table_a/session_000001_episode_000000 \
  --repo-id local/xiazhi_pick \
  --task 'pick the object' \
  --fps 30 \
  --image-topic camera_01=/camera_01/color/image_raw \
  --image-topic camera_02=/camera_02/color/image_raw \
  --image-topic camera_03=/camera_03/color/image_raw \
  --depth-topic camera_01_depth=/camera_01/depth/image_raw \
  --depth-topic camera_02_depth=/camera_02/depth/image_raw \
  --depth-topic camera_03_depth=/camera_03/depth/image_raw
```

默认数值来源：

- state：`/joint_states.position`；
- EE pose：`/rm_driver/udp_arm_position`；
- force：`/rm_driver/udp_six_zero_force`。

转换过程：

1. 以 joint state 为基准按 `--fps` 降采样。
2. 在 `--tolerance-s` 内选择最近的 EE pose、force、RGB 和 depth。
3. 相邻 observation 生成 `action(t)=q(t+1)`，每个 bag 独立成为一个 episode。
4. 丢弃缺少必需传感器的帧和最后一个 observation。
5. 使用当前 `LeRobotDataset.create/add_frame/save_episode/finalize` 写入。

可使用 `--state-topic`、`--state-field`、`--ee-pose-topic` 和 `--force-topic` 修改默认映射。

### Python 版本限制

标准 Humble 的 `rosbag2_py` 使用 Python 3.10，当前 LeRobot 要求 Python 3.12。现有转换入口不能直接在标准环境中执行；使用 LeRobot 直采，或准备带 Python 3.12 ROS bindings 的转换环境。

# ROS 2 数据采集

采集统一使用官方 RealMan 和 Orbbec ROS driver，输出多 episode rosbag2/MCAP session。支持拖动直采、控制器示教后回放，以及众灵主臂遥操。LeRobotDataset 由离线 MCAP 转换产生。

## 目录

| 目录 | 职责 |
|---|---|
| `collectors/rosbag2.py` | 官方驱动编排、topic 检查、录包、session 生命周期 |
| `ros/realman.py` | rm_driver 单连接控制客户端 |
| `hardware/zhongling.py` | 众灵 ASCII 主臂反馈读取 |
| `drag/` | 拖动模式常量、拖动与回放 episode 流程 |
| `teleoperation/` | 主臂标定、关节映射、限速、停止和 ROS 遥操 episode |
| `io_utils/` | 按键、manifest、有界命令日志、子进程管理 |
| `config.py` | 设备与 session 配置 |

所有 CLI 位于 `scripts/`。不包含直接连接 SDK 写入 LeRobot 的采集入口。

## 使用说明

- [拖动采集与 setup.sh](../../obsidian-readme/S01%20data-pipeline/01-data-collect-drag-ros2-mcap.md)
- [主从遥操采集](../../obsidian-readme/S01%20data-pipeline/02-data-collection-leaderfollower-ros2-mcap.md)
- [MCAP 转 LeRobot](../../obsidian-readme/S01%20data-pipeline/03-data-collection-mcap2lerobot.md)
- [Rerun 可视化](../../obsidian-readme/S01%20data-pipeline/04-data-collect-rerun-visualization.md)

主臂实现参考 [MINT-SJTU U-Arm 的众灵读取器](https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm/blob/main/src/uarm/scripts/Uarm_teleop/Zhonglin_servo/servo_reader.py)。ZP10D 设备参数需与实物一致；位置、保持、停止采用众灵 ASCII 协议，运动前读取工作模式并确认保持响应。当前控制六轴，未驱动夹爪。ROS 目标记录表示已发布命令，不表示控制器确认执行。

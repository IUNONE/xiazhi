# 02 Rerun 可视化与采集验收

## 1. 统一入口

`scripts/visualize_rerun.py` 按输入类型选择现有官方 importer/visualizer：

- `.mcap` 或 MCAP rosbag2；
- SQLite rosbag2，先临时转 MCAP；
- LeRobotDataset，目录包含 `meta/info.json`。

## 2. 命令

```bash
# rosbag2 episode
python scripts/visualize_rerun.py \
  /data/raw/pick/table_a/session_000001/episode_000000

# MCAP 文件
python scripts/visualize_rerun.py /data/processed/episode_000001.mcap

# 直接采集或转换得到的 LeRobotDataset
python scripts/visualize_rerun.py \
  /data/lerobot/pick/table_a/session_000001 \
  --repo-id local/xiazhi_pick \
  --episode-index 0
```

保存 RRD 而不启动窗口：

```bash
python scripts/visualize_rerun.py INPUT --output-rrd /data/review/episode_000001.rrd
```

## 3. 每个 episode 的验收项

采集完成后同时检查 `session_manifest.json` 中对应的 episode 记录和可视化结果：

1. session 和目标 episode 的 `status` 都是 `completed`，manifest 中的所有 camera serial 与现场设备一致。
2. 命令行指定的各路 RGB 均连续，没有黑帧、冻结或相机名称交换。
3. 开启 depth 时各路深度单位为 mm，形状为 H×W×1，没有被当作 RGB 归一化。
4. `observation.state` 为六关节 rad；轨迹连续且不超关节极限。
5. `observation.ee_pose` 四元数顺序为 `qw qx qy qz`。
6. `observation.force` 顺序为 `fx fy fz mx my mz`。
7. `action[t]` 等于下一帧关节状态；episode 帧数比完整 observation 序列少一帧。
8. 返航动作不出现在录制数据中。
9. 不存在 NaN/Inf、异常 action 跳变或明显的相机/关节错位。

## 4. 两条采集链路的一致性检查

在同一安全小幅度示教动作上分别录一次 ROS 和 LeRobot：

- 相同 `task`、`scene`、fps、相机顺序和深度开关；
- ROS bag 转换后与直采 LeRobot 使用相同 feature 名称、维度和单位；
- 对比关节范围、六维力数量级、实际帧数和图像时间连续性；
- 不要求逐帧数值完全相同，因为两次示教和采样时刻不同。

## 5. 真机分阶段测试

不要直接从未测试代码跳到完整示教：

1. 连接/状态读取，不运动。
2. 单独启动和停止 drag teach。
3. 空旷环境低速返航，验证误差。
4. 单相机、再三相机同时采样。
5. 五秒 rosbag2，不拖动。
6. 五秒 LeRobotDataset，不拖动。
7. 操作者握有急停时进行小幅度完整 `r -> s -> return` 测试。
8. 验证 `s -> READY -> r` 能产生第二个独立 episode，且等待期间没有新增录制数据。
9. 分别在 READY 和 RECORDING 状态验证 `q`/`Ctrl-C` 能安全返航并结束 session。
10. 完成上述可视化和 schema 验收后，才进入正式采集。

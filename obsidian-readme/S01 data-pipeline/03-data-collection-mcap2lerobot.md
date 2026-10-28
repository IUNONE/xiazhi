
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

| 常用参数 | 作用 |
|---|---|
| `--fps` | 按关节状态降采样的目标频率 |
| `--tolerance-s` | 传感器对齐窗口，默认 `0.05` 秒 |
| `--state-topic` / `--state-field` | 关节来源，默认 `/joint_states` / `position` |
| `--ee-pose-topic` | 末端位姿，默认 `/rm_driver/udp_arm_position` |
| `--force-topic` | 六维力，默认 `/rm_driver/udp_six_zero_force` |

输出路径：
```text
<output-root>/
├── data/
├── meta/
├── videos/
└── action_provenance.json
```

`action_provenance.json` 记录 action 来源和对齐方式。

| 字段 | 内容 |
|---|---|
| `observation.state` | 六关节位置，rad |
| `observation.ee_pose` | `x y z qw qx qy qz`，位置单位 m |
| `observation.force` | `fx fy fz mx my mz`，N / Nm |
| `observation.images.camera_N` | RGB uint8 |
| `observation.images.camera_N_depth` | 单通道深度 uint16，mm |
| `action` | 六关节绝对目标，rad |

转换后按 [Rerun 验收说明](04-data-collect-rerun-visualization.md) 检查结果。

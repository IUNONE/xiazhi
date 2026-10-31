
mcap 格式:

```bash
TASK_NAME="test"
OUTPUT_ROOT="$HOME/datasets"
SCENE="Lab"
ID="000001"

python scripts/visualize_rerun.py \
	--mcap_datasets_dir "${OUTPUT_ROOT}/${TASK_NAME}/${SCENE}/session_${ID}"
```

LeRobot 格式:

```bash
python scripts/visualize_rerun.py \
	--lerobot_datasets_dir "${OUTPUT_ROOT}/${TASK_NAME}/${SCENE}/session_${ID}_lerobot"
```

两种模式均显示 ECO63 机械臂和 `camera_01` 的 RGB 相机视锥，图像贴在视锥平面上。默认相机与末端 `Link6` 原点重合、姿态相同（光轴沿末端 +Z），随关节状态运动。`--eef-link` 可更换挂载 link；`--no-robot-model` 可关闭模型和默认视锥。

这是可视化用的默认安装姿态，不是手眼标定结果，也不会修改采集数据。MCAP 使用 `/camera_01/color/camera_info` 内参；LeRobot 通过 `action_provenance.json` 尝试读取原始 MCAP 的内参。原始包已移走或内参缺失时，使用水平视场角 60° 的示意内参，终端会明确打印来源。视锥图像平面距相机 0.2 m，仅控制显示大小，不代表物体距离。

其他相机仍显示在右侧二维窗口；其三维安装位置需要提供真实外参后才能确定。更新代码后直接重跑上述命令，RRD 缓存会自动重建，无需重新转换 LeRobot。

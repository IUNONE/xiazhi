
```bash
conda activate xiazhi

export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

TASK_NAME="test"
OUTPUT_ROOT="$HOME/datasets"
SCENE="Lab"
ID="000001"

python scripts/convert_mcap2lerobot.py \
  --mcap_dir "${OUTPUT_ROOT}/${TASK_NAME}/${SCENE}/session_${ID}" \
  --output_dir "${OUTPUT_ROOT}/${TASK_NAME}/${SCENE}/session_${ID}_lerobot" \
  --fps 30
```

```bash
# 指定：把 MCAP 中哪个 ROS 图像 topic，保存为 LeRobot 中哪个图像字段。
  --image-topic camera_01=/camera_01/color/image_raw \
  --image-topic camera_02=/camera_02/color/image_raw \
  --image-topic camera_03=/camera_03/color/image_raw \
  --depth-topic camera_01_depth=/camera_01/depth/image_raw \
  --depth-topic camera_02_depth=/camera_02/depth/image_raw \
  --depth-topic camera_03_depth=/camera_03/depth/image_raw
```


输出路径：
```text
<output_dir>/
├── data/
├── meta/
├── videos/
└── action_provenance.json
```


转换后按 [Rerun 验收说明](04-data-collect-rerun-visualization.md) 检查结果。

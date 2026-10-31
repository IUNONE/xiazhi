
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

![[assets/rerun.webm]]


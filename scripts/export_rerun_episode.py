"""Internal isolated export worker; CLI parsing stays in scripts/."""
from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path

from data_pipeline.data_rerun.mcap import McapRerunVisualizer
from data_pipeline.data_rerun.lerobot import LeRobotRerunConfig, LeRobotRerunVisualizer


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('request', type=Path)
    request = json.loads(parser.parse_args().request.read_text())
    output = Path(request['output'])
    if output.exists():
        raise FileExistsError(output)
    options = request['options']
    if request['kind'] == 'mcap':
        McapRerunVisualizer(
            (Path(request['path']),),
            robot_xacro=Path(options['robot_xacro']) if options['robot_xacro'] else None,
            camera_names=tuple(options['camera_names']), instruction=options['instruction'],
            eef_link=options['eef_link'],
        ).save_episode(output, request['label'], request['recording_id'])
    else:
        # The official exporter uses global Rerun state; isolate every episode.
        with tempfile.TemporaryDirectory(prefix='lerobot-', dir=output.parent) as temp:
            exported = LeRobotRerunVisualizer(LeRobotRerunConfig(
                repo_id=request['repo_id'], episode_index=request['episode_index'],
                root=Path(request['path']), save=True, output_dir=Path(temp),
                batch_size=options['batch_size'], num_workers=options['num_workers'],
                blueprint_path=output.with_suffix('.rbl'),
            )).visualize()
            if exported is None:
                raise RuntimeError('LeRobot visualizer 未返回 RRD')
            import rerun as rr
            recording = rr.get_data_recording()
            if recording is None:
                raise RuntimeError('LeRobot visualizer 未创建 Rerun recording')
            try:
                recording.send_recording_name(request['label'])
                recording.flush()
            finally:
                recording.disconnect()
            subprocess.run([
                'rerun', 'rrd', 'route', '--application-id', request['label'],
                '--recording-id', request['recording_id'], '--output', str(output), str(exported),
            ], check=True)


if __name__ == '__main__':
    main()

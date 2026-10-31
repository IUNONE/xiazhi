"""Discover episodes, cache independent recordings, and serve a local Rerun catalog."""
from __future__ import annotations

import fcntl
import hashlib
import importlib.metadata
import importlib.util
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import asdict, dataclass
from pathlib import Path
from uuid import NAMESPACE_URL, uuid5

PROJECT_ROOT = Path(__file__).resolve().parents[2]
CACHE_VERSION = 1


@dataclass(frozen=True)
class Episode:
    kind: str
    path: Path
    label: str
    episode_index: int = 0
    repo_id: str | None = None


def discover_episodes(root: Path, kind: str, cache_dir: Path) -> list[Episode]:
    root = root.resolve()
    if not root.is_dir():
        raise NotADirectoryError(root)
    result = []
    seen = set()
    cache_dir = cache_dir.resolve()
    if root == cache_dir or root.is_relative_to(cache_dir):
        raise ValueError("数据目录不能位于 RRD 缓存目录内")

    def add(path, index=0, repo_id=None):
        identity = (path.resolve(), index)
        if identity in seen:
            return
        seen.add(identity)
        relative = path.relative_to(root).as_posix()
        label = root.name if relative == "." else f"{root.name}/{relative}"
        if kind == "lerobot":
            label += f"/episode_{index:06d}"
        result.append(Episode(kind, path.resolve(), label, index, repo_id))

    for directory, dirs, files in os.walk(root, followlinks=False):
        path = Path(directory)
        dirs[:] = sorted(d for d in dirs if not d.startswith('.') and (path / d).resolve() != cache_dir)
        if kind == "lerobot":
            info = path / "meta/info.json"
            if not info.is_file():
                continue
            metadata = json.loads(info.read_text(encoding="utf-8"))
            count = metadata.get("total_episodes")
            if type(count) is not int or count < 1:
                raise ValueError(f"无效的 total_episodes: {info}")
            repo_id = metadata.get("repo_id") or f"local/{path.name}"
            for index in range(count):
                add(path, index, repo_id)
            dirs[:] = []
        elif "metadata.yaml" in files:
            import yaml
            metadata = yaml.safe_load((path / "metadata.yaml").read_text())["rosbag2_bagfile_information"]
            if metadata.get("storage_identifier") == "mcap":
                names = metadata.get("relative_file_paths", [])
                if not names or any(not (path / n).is_file() or Path(n).suffix != '.mcap' or (path / n).resolve().parent != path.resolve() for n in names):
                    raise ValueError(f"MCAP episode 分卷不完整: {path}")
                add(path)
            dirs[:] = []
        else:
            for name in sorted(files):
                if name.endswith('.mcap') and kind == 'mcap':
                    add(path / name)
    if not result:
        raise ValueError(f"目录中没有 {kind} episode: {root}")
    return sorted(result, key=lambda item: (str(item.path), item.episode_index))


def _file_stamp(path: Path) -> tuple[str, int, int]:
    stat = path.stat()
    return str(path.resolve()), stat.st_size, stat.st_mtime_ns


def _source_stamp(episode: Episode):
    if episode.path.is_file():
        paths = [episode.path]
        metadata = episode.path.parent / 'metadata.yaml'
        if metadata.is_file():
            paths.append(metadata)
    elif episode.kind == 'mcap':
        import yaml
        metadata = episode.path / 'metadata.yaml'
        info = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
        paths = [metadata, *(episode.path / n for n in info['relative_file_paths'])]
    else:
        paths = []
        for name in ('meta', 'data', 'videos', 'images'):
            folder = episode.path / name
            if folder.is_dir():
                paths.extend(p for p in folder.rglob('*') if p.is_file())
    return [_file_stamp(path) for path in sorted(paths)]


def _require_directory_version() -> str:
    version = importlib.metadata.version('rerun-sdk')
    numbers = tuple(int(p) for p in version.split('+')[0].split('.')[:2])
    if numbers != (0, 33):
        raise RuntimeError(f"目录浏览需要 rerun-sdk>=0.33,<0.34，当前为 {version}；请更新 env.yaml 环境")
    if importlib.util.find_spec('datafusion') is None:
        raise ImportError('目录布局需要 rerun-sdk[catalog]；请更新 env.yaml 环境')
    cli_version = subprocess.run(['rerun', '--version'], check=True, capture_output=True, text=True).stdout
    if re.search(r'\b0\.33\.', cli_version) is None:
        raise RuntimeError(f"PATH 中的 rerun CLI 必须与 SDK 同为 0.33: {cli_version.strip()}")
    return version


def cache_episodes(episodes: list[Episode], cache_dir: Path, options: dict) -> list[Path]:
    version = _require_directory_version()
    cache_dir = cache_dir.resolve()
    cache_dir.mkdir(parents=True, exist_ok=True)
    code = hashlib.sha256()
    for path in sorted(Path(__file__).parent.glob('*.py')):
        code.update(path.read_bytes())
    code.update((PROJECT_ROOT / 'scripts/export_rerun_episode.py').read_bytes())
    code.update((PROJECT_ROOT / 'data_pipeline/data_format/rosbag2.py').read_bytes())
    software = {'rerun': version, 'adapter': code.hexdigest(), 'cache_version': CACHE_VERSION}
    if episodes[0].kind == 'lerobot':
        software['lerobot'] = importlib.metadata.version('lerobot')
        spec = importlib.util.find_spec('lerobot.scripts.lerobot_dataset_viz')
        if spec is not None and spec.origin:
            software['lerobot_visualizer'] = hashlib.sha256(Path(spec.origin).read_bytes()).hexdigest()
    model = options.get('robot_xacro')
    model_stamp = []
    if model:
        description = Path(model).parent.parent
        if not Path(model).is_file():
            raise FileNotFoundError(model)
        model_stamp = [_file_stamp(p) for p in sorted(description.rglob('*')) if p.is_file()]
    outputs = []
    with (cache_dir / '.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"另一个进程正在生成缓存: {cache_dir}") from exc
        for index, episode in enumerate(episodes, 1):
            # Metadata and stat fingerprints avoid re-reading large videos on cache hits.
            before = _source_stamp(episode)
            request = {
                **asdict(episode), 'path': str(episode.path), 'options': options,
                'source': before, 'software': software, 'model': model_stamp,
            }
            encoded = json.dumps(request, sort_keys=True, ensure_ascii=False)
            key = hashlib.sha256(encoded.encode()).hexdigest()
            entry = cache_dir / key
            output = entry / 'episode.rrd'
            if entry.exists():
                manifest = json.loads((entry / 'manifest.json').read_text())
                blueprint = entry / 'episode.rbl'
                if not output.is_file() or output.stat().st_size != manifest['rrd_size'] or output.stat().st_mtime_ns != manifest['rrd_mtime_ns'] or not blueprint.is_file() or hashlib.sha256(blueprint.read_bytes()).hexdigest() != manifest['blueprint_sha256']:
                    raise ValueError(f"缓存损坏，请移走该缓存项后重试: {entry}")
                print(f"[{index}/{len(episodes)}] 缓存命中 {episode.label}", flush=True)
            else:
                print(f"[{index}/{len(episodes)}] 生成 RRD {episode.label}", flush=True)
                with tempfile.TemporaryDirectory(prefix='.building-', dir=cache_dir) as temp:
                    staging = Path(temp)
                    request['recording_id'] = str(uuid5(NAMESPACE_URL, key))
                    request['output'] = str(staging / 'episode.rrd')
                    request_file = staging / 'request.json'
                    request_file.write_text(json.dumps(request, ensure_ascii=False), encoding='utf-8')
                    subprocess.run([sys.executable, str(PROJECT_ROOT / 'scripts/export_rerun_episode.py'), str(request_file)], check=True)
                    generated = staging / 'episode.rrd'
                    if not generated.is_file() or generated.stat().st_size == 0:
                        raise RuntimeError(f"没有生成有效 RRD: {episode.label}")
                    if before != _source_stamp(episode):
                        raise RuntimeError(f"导出期间源数据发生变化: {episode.path}")
                    (staging / 'manifest.json').write_text(json.dumps({
                        'label': episode.label, 'source': str(episode.path),
                        'rrd_size': generated.stat().st_size, 'rrd_mtime_ns': generated.stat().st_mtime_ns,
                        'cache_key': key,
                        'blueprint_sha256': hashlib.sha256((staging / 'episode.rbl').read_bytes()).hexdigest(),
                    }, ensure_ascii=False, indent=2), encoding='utf-8')
                    # Atomic publication; failed exports never become cache hits.
                    staging.rename(entry)
            outputs.append(output)
    return outputs


def view_directory(root: Path, kind: str, cache_dir: Path, options: dict, memory_limit: str = '50%') -> None:
    episodes = discover_episodes(root, kind, cache_dir)
    outputs = cache_episodes(episodes, cache_dir, options)
    import rerun as rr
    groups = {}
    for episode, output in zip(episodes, outputs):
        # LeRobot datasets can have different feature schemas and camera layouts.
        group = str(episode.path) if kind == 'lerobot' else 'mcap'
        groups.setdefault(group, []).append(output)
    datasets = {f'xiazhi_{kind}_{i:03d}': paths for i, paths in enumerate(groups.values())}
    with tempfile.TemporaryDirectory(prefix='xiazhi-blueprint-') as temp:
        mcap_blueprint = None
        if kind == 'mcap':
            from data_pipeline.data_rerun.mcap import McapRerunVisualizer
            visualizer = McapRerunVisualizer(
                tuple(episode.path for episode in episodes), camera_names=tuple(options['camera_names']),
            )
            _, rrb = visualizer._import_rerun()
            mcap_blueprint = Path(temp) / 'directory.rbl'
            visualizer._build_blueprint(rrb, directory_mode=True).save('xiazhi_mcap', mcap_blueprint)
        with rr.server.Server(host='127.0.0.1', datasets=datasets) as server:
            client = server.client()
            for name, paths in datasets.items():
                blueprint = mcap_blueprint or paths[0].with_suffix('.rbl')
                client.get_dataset(name).register_blueprint(blueprint.resolve().as_uri())
            print(f"已注册 {len(episodes)} 个 episode。在 Viewer 左侧 Sources 中选择数据集及 episode。", flush=True)
            print(f"缓存: {cache_dir.resolve()}\n本地数据源: {server.url()}", flush=True)
            # Native viewer queries the catalog; do not stream every RRD into its memory.
            viewer = subprocess.Popen(['rerun', '--new', 'true', '--memory-limit', memory_limit, server.url()])
            try:
                code = viewer.wait()
                if code:
                    raise subprocess.CalledProcessError(code, viewer.args)
            except KeyboardInterrupt:
                viewer.terminate()
            finally:
                if viewer.poll() is None:
                    viewer.terminate()
                    try:
                        viewer.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        viewer.kill()
                        viewer.wait()

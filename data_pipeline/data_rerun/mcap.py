from __future__ import annotations

import importlib.metadata
import heapq
from itertools import groupby
import re
import socket
import subprocess
import time
from pathlib import Path
from typing import Any


_CAMERA_TOPIC_PATTERN = re.compile(
    r"^[ \t]*name:[ \t]*/([^/\s]+)/color/image_raw[ \t]*$",
    re.MULTILINE,
)
_MINIMUM_RERUN_VERSION = (0, 28, 0)


class McapRerunVisualizer:
    """使用 Rerun 官方 importer 显示 ROS 2 MCAP 和 ECO63 模型。

    子函数
    --------
    - view: 使用预设布局打开一个或多个 MCAP 文件.
    - convert: 将单个 MCAP 文件转换为 RRD.
    """

    def __init__(
        self,
        inputs: tuple[Path, ...],
        *,
        robot_xacro: Path | None = None,
        link6_type: str = "Link6_6fb",
        camera_names: tuple[str, ...] = (),
        instruction: str | None = None,
        eef_link: str = "Link6",
    ) -> None:
        """初始化 MCAP 可视化器。

        参数
        --------
        inputs : tuple[Path, ...]
            MCAP 文件或 rosbag2 MCAP 目录.
        robot_xacro : Path | None
            可选的机械臂 xacro 文件. None 表示不加载机器人模型.
        link6_type : str
            ECO63 末端模型类型.
        camera_names : tuple[str, ...]
            相机命名空间. 留空时从 rosbag2 metadata.yaml 自动读取.

        返回
        --------
        None
            初始化实例状态.
        """
        self.paths = self._resolve(inputs)
        self.robot_xacro = robot_xacro.resolve() if robot_xacro is not None else None
        self.link6_type = link6_type
        self.camera_names = self._resolve_camera_names(camera_names)
        self.instruction = instruction
        self.eef_link = eef_link

    @staticmethod
    def _resolve(inputs: tuple[Path, ...]) -> tuple[Path, ...]:
        """解析输入中的全部 MCAP 文件。

        参数
        --------
        inputs : tuple[Path, ...]
            文件或目录输入.

        返回
        --------
        tuple[Path, ...]
            去重并排序的 MCAP 文件.
        """
        paths: list[Path] = []
        for input_path in inputs:
            if input_path.is_dir():
                metadata = input_path / "metadata.yaml"
                if metadata.is_file():
                    import yaml
                    info = yaml.safe_load(metadata.read_text())["rosbag2_bagfile_information"]
                    parts = [input_path / name for name in info.get("relative_file_paths", [])]
                    if not parts or any(not part.is_file() or part.suffix != ".mcap" or part.resolve().parent != input_path.resolve() for part in parts):
                        raise ValueError(f"MCAP 分卷无效或缺失: {input_path}")
                    paths.extend(parts)
                else:
                    paths.extend(sorted(input_path.glob("*.mcap")))
            elif input_path.is_file() and input_path.suffix.lower() == ".mcap":
                paths.append(input_path)
            else:
                raise FileNotFoundError(f"无效的 MCAP 输入: {input_path}")
        unique_paths = tuple(dict.fromkeys(path.resolve() for path in paths))
        if not unique_paths:
            raise FileNotFoundError("输入中没有 MCAP 文件")
        return unique_paths

    def _resolve_camera_names(self, names: tuple[str, ...]) -> tuple[str, ...]:
        """解析相机命名空间并保持首次出现顺序。

        参数
        --------
        names : tuple[str, ...]
            命令行显式提供的相机名称.

        返回
        --------
        tuple[str, ...]
            显式名称或 metadata.yaml 中检测到的名称.
        """
        if names:
            return tuple(dict.fromkeys(name.strip("/") for name in names if name.strip("/")))

        detected: list[str] = []
        for path in self.paths:
            metadata_path = path.parent / "metadata.yaml"
            if not metadata_path.is_file():
                continue
            metadata = metadata_path.read_text(encoding="utf-8")
            detected.extend(_CAMERA_TOPIC_PATTERN.findall(metadata))
        return tuple(dict.fromkeys(detected))

    @staticmethod
    def _import_rerun() -> tuple[Any, Any]:
        """导入满足机器人命名坐标系要求的 Rerun SDK。

        返回
        --------
        tuple[Any, Any]
            rerun 和 rerun.blueprint 模块.
        """
        try:
            import rerun as rr
            import rerun.blueprint as rrb
        except ImportError as exc:
            raise ImportError(
                "需要安装 rerun-sdk>=0.28,<0.34 后再执行 MCAP 可视化"
            ) from exc

        version_text = importlib.metadata.version("rerun-sdk")
        version_numbers = tuple(
            int(part) for part in re.findall(r"\d+", version_text.split("+", 1)[0])[:3]
        )
        normalized_version = version_numbers + (0,) * (3 - len(version_numbers))
        if normalized_version < _MINIMUM_RERUN_VERSION:
            raise RuntimeError(
                f"当前 rerun-sdk {version_text} 过旧, 需要升级到 >=0.28,<0.34"
            )
        return rr, rrb

    def _build_blueprint(self, rrb: Any, *, directory_mode: bool = False) -> Any:
        """构建机械臂, 相机和关节曲线的固定布局。

        参数
        --------
        rrb : Any
            rerun.blueprint 模块.

        返回
        --------
        Any
            Rerun Blueprint 实例.
        """
        from data_pipeline.data_rerun.camera import robot_view
        scene_view = robot_view(rrb, ["/camera_01/color/image_raw"])
        signal_views = [
            rrb.TimeSeriesView(
                name=name, origin=origin, contents=contents,
            )
            for name, origin, contents in (
                ("Joints", "/joint_states", ["/joint_states/position"]),
                # Gripper remains reserved until a collection stream is configured.
                ("Gripper", "/gripper", ["/gripper/**"]),
                ("EEF_delta_xyz", "/eef_delta_xyz", ["/eef_delta_xyz/**"]),
                ("EEF_Force", "/rm_driver/udp_six_zero_force", ["/rm_driver/udp_six_zero_force/**"]),
            )
        ]
        camera_views = [
            rrb.Tabs(
                *[
                    rrb.Spatial2DView(
                        name=f"{camera_name} {label}",
                        origin=f"/{camera_name}/{stream}/image_raw",
                        contents=[
                            f"/{camera_name}/{stream}/image_raw",
                            f"/{camera_name}/{stream}/camera_info",
                        ],
                    )
                    for stream, label in (("color", "RGB"), ("depth", "Depth"))
                ],
                name=camera_name,
            )
            for camera_name in self.camera_names
        ]
        robot_panel = rrb.Vertical(
            rrb.TextDocumentView(name="Instruction", origin="/__xiazhi/instruction"),
            scene_view,
            row_shares=[1, 7],
        )
        main_panel = (
            rrb.Horizontal(
                robot_panel,
                rrb.Vertical(*camera_views),
                column_shares=[3, 1],
            )
            if camera_views else robot_panel
        )
        return rrb.Blueprint(
            rrb.Vertical(
                main_panel,
                rrb.Horizontal(*signal_views),
                row_shares=[3, 1],
                name="MCAP Episode",
            ),
            rrb.TimePanel(state="collapsed"),
            rrb.BlueprintPanel(state="expanded" if directory_mode else "collapsed"),
            rrb.SelectionPanel(state="collapsed"),
        )

    def _build_robot_urdf(self) -> bytes | None:
        from data_pipeline.data_rerun.robot_model import build_robot_urdf
        return build_robot_urdf(self.robot_xacro, self.link6_type)

    def _log_wrist_camera(self, recording: Any, rr: Any) -> None:
        from data_pipeline.data_rerun.camera import log_wrist_camera, read_mcap_intrinsics
        intrinsics = read_mcap_intrinsics(self.paths)
        if intrinsics is not None:
            log_wrist_camera(recording, rr, "/camera_01/color/image_raw", intrinsics, self.eef_link)

    @staticmethod
    def _find_available_viewer_port() -> int:
        """获取本机可用于独立 Rerun Viewer 的 TCP 端口。

        返回
        --------
        int
            当前可用的本机 TCP 端口.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
            server_socket.bind(("127.0.0.1", 0))
            return int(server_socket.getsockname()[1])

    @staticmethod
    def _wait_for_viewer_to_close(viewer_port: int) -> None:
        """保持数据流连接直到 Rerun Viewer 关闭。

        参数
        --------
        viewer_port : int
            Viewer gRPC 服务监听端口.

        返回
        --------
        None
            Viewer 端口连续不可用后返回.
        """
        unavailable_count = 0
        while unavailable_count < 3:
            time.sleep(0.5)
            try:
                with socket.create_connection(("127.0.0.1", viewer_port), timeout=0.5):
                    unavailable_count = 0
            except OSError:
                unavailable_count += 1

    def _log_eef_delta(self, recording: Any, rr: Any, urdf: bytes) -> None:
        """Log p(t)-p(t-1) in base_root, metres, on MCAP receipt time."""
        import numpy as np

        from data_pipeline.data_format.rosbag2 import Rosbag2Reader
        from data_pipeline.data_rerun.kinematics import EndEffectorKinematics

        fk = EndEffectorKinematics(urdf, "base_root", self.eef_link)
        for _, paths in groupby(self.paths, key=lambda path: path.parent):
            previous = previous_time = None
            streams = [Rosbag2Reader(path, storage_id="mcap").iter_records({"/joint_states"}) for path in paths]
            for record in heapq.merge(*streams, key=lambda item: item.timestamp_ns):
                if record.type_name != "sensor_msgs/msg/JointState":
                    raise ValueError("/joint_states must contain sensor_msgs/msg/JointState")
                position = fk.position(record.message)
                stamp = record.timestamp_ns
                if previous is not None and stamp > previous_time:
                    recording.set_time("message_log_time", timestamp=np.datetime64(stamp, "ns"))
                    for axis, current, old in zip("xyz", position, previous):
                        recording.log(f"/eef_delta_xyz/{axis}", rr.Scalars(current - old))
                previous, previous_time = position, stamp
        recording.reset_time()
        print(f"EEF_delta_xyz: base_root -> {self.eef_link}, p(t)-p(t-1), metres; timeline=message_log_time")

    def view(self) -> None:
        """在 Rerun Viewer 中打开 MCAP 和可选机器人模型。

        返回
        --------
        None
            数据发送给 Viewer 后返回.
        """
        rr, rrb = self._import_rerun()
        blueprint = self._build_blueprint(rrb)
        recording = rr.RecordingStream("xiazhi_ros2_mcap")
        viewer_port = self._find_available_viewer_port()
        recording.spawn(
            port=viewer_port,
            memory_limit="25%",
            default_blueprint=blueprint,
        )
        recording.send_blueprint(blueprint)
        recording.log(
            "/__xiazhi/instruction",
            rr.TextDocument(self.instruction or "No task instruction provided"),
            static=True,
        )

        robot_urdf = self._build_robot_urdf()
        if robot_urdf is not None:
            recording.log_file_from_contents(
                file_path="rm_eco63.urdf",
                file_contents=robot_urdf,
                entity_path_prefix="robot_model",
                static=True,
            )
        for path in self.paths:
            recording.log_file_from_path(path)
        if robot_urdf is not None:
            self._log_wrist_camera(recording, rr)
            self._log_eef_delta(recording, rr, robot_urdf)
        else:
            print("未加载 URDF，无法计算 EEF_delta_xyz")
        recording.flush()

        print(f"已发送 {len(self.paths)} 个 MCAP 文件到 Rerun Viewer")
        if self.camera_names:
            print(f"相机视图: {', '.join(self.camera_names)}")
        try:
            print("Viewer 正在流式加载数据. 关闭窗口后脚本会自动结束")
            self._wait_for_viewer_to_close(viewer_port)
        except KeyboardInterrupt:
            print("\n收到 Ctrl-C, 正在断开 Rerun Viewer")
        finally:
            recording.disconnect()

    def save_episode(self, output_rrd: Path, name: str, recording_id: str) -> Path:
        """Cache one complete episode with its layout, model and derived curves."""
        if output_rrd.exists():
            raise FileExistsError(output_rrd)
        rr, rrb = self._import_rerun()
        output_rrd.parent.mkdir(parents=True, exist_ok=True)
        blueprint = self._build_blueprint(rrb)
        blueprint_path = output_rrd.with_suffix('.rbl')
        if blueprint_path.exists():
            raise FileExistsError(blueprint_path)
        blueprint.save(name, blueprint_path)
        recording = rr.RecordingStream(name, recording_id=recording_id)
        try:
            recording.save(output_rrd)
            recording.send_recording_name(name)
            recording.send_blueprint(blueprint)
            recording.log("/__xiazhi/instruction", rr.TextDocument(self.instruction or name), static=True)
            robot_urdf = self._build_robot_urdf()
            if robot_urdf is not None:
                recording.log_file_from_contents(
                    file_path="rm_eco63.urdf", file_contents=robot_urdf,
                    entity_path_prefix="robot_model", static=True,
                )
            for path in self.paths:
                recording.log_file_from_path(path)
            if robot_urdf is not None:
                self._log_wrist_camera(recording, rr)
                self._log_eef_delta(recording, rr, robot_urdf)
            recording.flush()
        finally:
            recording.disconnect()
        return output_rrd

    def convert(self, output_rrd: Path) -> Path:
        """使用 Rerun 官方 importer 将单个 MCAP 转换为 RRD。

        参数
        --------
        output_rrd : Path
            目标 RRD 文件路径.

        返回
        --------
        Path
            完成转换的 RRD 文件路径.
        """
        if len(self.paths) != 1:
            raise ValueError("MCAP 到 RRD 转换每次只接受一个输入文件")
        if output_rrd.exists():
            raise FileExistsError(f"输出文件已存在: {output_rrd}")
        output_rrd.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(
            [
                "rerun",
                "mcap",
                "convert",
                str(self.paths[0]),
                "--output",
                str(output_rrd),
            ],
            check=True,
        )
        return output_rrd

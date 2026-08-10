from __future__ import annotations

import importlib.metadata
import os
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

    def _build_blueprint(self, rrb: Any) -> Any:
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
        scene_view = rrb.Spatial3DView(
            name="ECO63 Robot",
            origin="/",
            contents=["/robot_model/**", "/tf", "/tf_static"],
            background=[18, 22, 30],
            spatial_information=rrb.SpatialInformation(
                target_frame="base_root",
                show_axes=True,
                show_bounding_box=False,
            ),
            eye_controls=rrb.EyeControls3D(
                position=(1.4, -1.4, 1.2),
                look_target=(0.0, 0.0, 0.45),
                eye_up=(0.0, 0.0, 1.0),
            ),
        )
        signal_view = rrb.TimeSeriesView(
            name="Joint Positions",
            origin="/joint_states",
            contents=["/joint_states/position"],
        )
        camera_views = [
            rrb.Spatial2DView(
                name=f"{camera_name} RGB",
                origin=f"/{camera_name}/color/image_raw",
                contents=[
                    f"/{camera_name}/color/image_raw",
                    f"/{camera_name}/color/camera_info",
                ],
            )
            for camera_name in self.camera_names
        ]
        camera_views.extend(
            rrb.Spatial2DView(
                name=f"{camera_name} Depth",
                origin=f"/{camera_name}/depth/image_raw",
                contents=[
                    f"/{camera_name}/depth/image_raw",
                    f"/{camera_name}/depth/camera_info",
                ],
            )
            for camera_name in self.camera_names
        )

        if camera_views:
            review_panel = rrb.Vertical(
                rrb.Grid(*camera_views, grid_columns=max(1, len(self.camera_names))),
                signal_view,
                row_shares=[3, 2],
                name="Cameras and Joints",
            )
        else:
            review_panel = signal_view

        return rrb.Blueprint(
            rrb.Horizontal(
                scene_view,
                review_panel,
                column_shares=[3, 2],
                name="MCAP Episode",
            ),
            rrb.TimePanel(state="expanded"),
            rrb.BlueprintPanel(state="collapsed"),
            rrb.SelectionPanel(state="collapsed"),
        )

    def _build_robot_urdf(self) -> bytes | None:
        """展开 xacro 并将模型资源改为绝对文件路径。

        返回
        --------
        bytes | None
            可交给 Rerun URDF importer 的 XML 内容. 未配置模型时返回 None.
        """
        if self.robot_xacro is None:
            return None
        if not self.robot_xacro.is_file():
            raise FileNotFoundError(f"机械臂 xacro 不存在: {self.robot_xacro}")

        xacro_executable = Path("/opt/ros/humble/bin/xacro")
        xacro_command = str(xacro_executable) if xacro_executable.is_file() else "xacro"
        xacro_environment = os.environ.copy()
        ros_python_paths = (
            Path("/opt/ros/humble/lib/python3.10/site-packages"),
            Path("/opt/ros/humble/local/lib/python3.10/dist-packages"),
        )
        python_paths = [str(path) for path in ros_python_paths if path.is_dir()]
        existing_python_path = xacro_environment.get("PYTHONPATH")
        if existing_python_path:
            python_paths.append(existing_python_path)
        if python_paths:
            xacro_environment["PYTHONPATH"] = os.pathsep.join(python_paths)
        try:
            completed = subprocess.run(
                [xacro_command, str(self.robot_xacro), f"link6_type:={self.link6_type}"],
                check=True,
                capture_output=True,
                text=True,
                env=xacro_environment,
            )
        except subprocess.CalledProcessError as exc:
            detail = exc.stderr.strip() or str(exc)
            raise RuntimeError(f"ECO63 xacro 展开失败: {detail}") from exc
        description_root = self.robot_xacro.parent.parent
        resource_prefix = description_root.as_uri() + "/"
        urdf = completed.stdout.replace("package://rm_description/", resource_prefix)
        return urdf.encode("utf-8")

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
        recording.flush()

        print(f"已发送 {len(self.paths)} 个 MCAP 文件到 Rerun Viewer")
        if self.camera_names:
            print(f"相机视图: {', '.join(self.camera_names)}")
        if robot_urdf is not None and self.camera_names:
            print(
                "提示: 相机视锥只有在 MCAP 含相机坐标系到机械臂坐标系的外参时, "
                "才会出现在机械臂三维坐标系中"
            )
        try:
            print("Viewer 正在流式加载数据. 关闭窗口后脚本会自动结束")
            self._wait_for_viewer_to_close(viewer_port)
        except KeyboardInterrupt:
            print("\n收到 Ctrl-C, 正在断开 Rerun Viewer")
        finally:
            recording.disconnect()

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

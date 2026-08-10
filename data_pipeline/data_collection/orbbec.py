from __future__ import annotations

import importlib
import threading
import time
from dataclasses import dataclass
from types import ModuleType
from typing import Any

import numpy as np

from data_pipeline.data_collection.config import CameraConfig


@dataclass(frozen=True, slots=True)
class RGBDFrame:
    """A color/depth pair originating from one Orbbec frameset."""

    color: np.ndarray
    depth_mm: np.ndarray | None
    captured_at_ns: int


def _load_sdk() -> ModuleType:
    try:
        return importlib.import_module("pyorbbecsdk")
    except ImportError as exc:
        raise ImportError(
            "缺少 Orbbec Python SDK；请在 LeRobot 环境安装 pyorbbecsdk2"
        ) from exc


class OrbbecCamera:
    """Gemini 335 adapter with serial binding and synchronized RGB-D snapshots."""

    def __init__(self, config: CameraConfig) -> None:
        config.validate()
        self.config = config
        self._sdk: ModuleType | None = None
        self._pipeline: Any | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._new_frame_event = threading.Event()
        self._lock = threading.Lock()
        self._latest: RGBDFrame | None = None
        self._read_error: BaseException | None = None

    @property
    def is_connected(self) -> bool:
        return self._pipeline is not None

    @staticmethod
    def find_cameras() -> list[dict[str, str]]:
        sdk = _load_sdk()
        devices = sdk.Context().query_devices()
        cameras: list[dict[str, str]] = []
        for index in range(devices.get_count()):
            info = devices.get_device_by_index(index).get_device_info()
            cameras.append(
                {
                    "serial_number": str(info.get_serial_number()),
                    "name": str(info.get_name()),
                    "firmware_version": str(info.get_firmware_version()),
                }
            )
        return cameras

    def connect(self, warmup: bool = True) -> None:
        if self.is_connected:
            raise RuntimeError(f"{self.config.name} 已连接")
        sdk = _load_sdk()
        devices = sdk.Context().query_devices()
        device = None
        available: list[str] = []
        for index in range(devices.get_count()):
            candidate = devices.get_device_by_index(index)
            serial = str(candidate.get_device_info().get_serial_number())
            available.append(serial)
            if serial == self.config.serial_number:
                device = candidate
        if device is None:
            raise ConnectionError(
                f"未找到 {self.config.name} serial={self.config.serial_number}；"
                f"当前设备: {available}"
            )

        pipeline = sdk.Pipeline(device)
        stream_config = sdk.Config()
        try:
            color_profiles = pipeline.get_stream_profile_list(sdk.OBSensorType.COLOR_SENSOR)
            color_profile = color_profiles.get_video_stream_profile(
                self.config.width,
                self.config.height,
                sdk.OBFormat.RGB,
                self.config.fps,
            )
            stream_config.enable_stream(color_profile)
            if self.config.enable_depth:
                depth_profiles = pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR)
                depth_profile = depth_profiles.get_video_stream_profile(
                    self.config.width,
                    self.config.height,
                    sdk.OBFormat.Y16,
                    self.config.fps,
                )
                stream_config.enable_stream(depth_profile)
            pipeline.start(stream_config)
        except BaseException:
            try:
                pipeline.stop()
            except Exception:
                pass
            raise

        self._sdk = sdk
        self._pipeline = pipeline
        self._latest = None
        self._read_error = None
        self._stop_event.clear()
        self._new_frame_event.clear()
        self._thread = threading.Thread(
            target=self._read_loop,
            name=f"orbbec-{self.config.name}",
            daemon=True,
        )
        self._thread.start()

        if warmup:
            try:
                self._wait_for_first_frame()
            except BaseException:
                self.disconnect()
                raise

    def _wait_for_first_frame(self) -> None:
        deadline = time.monotonic() + self.config.warmup_s
        while time.monotonic() < deadline:
            if self._new_frame_event.wait(timeout=0.1):
                self.read_latest_rgbd(max_age_ms=max(500, int(self.config.warmup_s * 1000)))
                return
            self._raise_read_error()
        raise ConnectionError(f"{self.config.name} 预热期间未收到图像")

    def _raise_read_error(self) -> None:
        with self._lock:
            error = self._read_error
        if error is not None:
            raise RuntimeError(f"{self.config.name} 采集线程失败: {error}") from error

    def _read_loop(self) -> None:
        assert self._pipeline is not None
        try:
            while not self._stop_event.is_set():
                frames = self._pipeline.wait_for_frames(200)
                if frames is None:
                    continue
                color_frame = frames.get_color_frame()
                if color_frame is None:
                    continue
                color = np.frombuffer(color_frame.get_data(), dtype=np.uint8).reshape(
                    color_frame.get_height(),
                    color_frame.get_width(),
                    3,
                )
                depth_mm: np.ndarray | None = None
                if self.config.enable_depth:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame is None:
                        continue
                    raw_depth = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(
                        depth_frame.get_height(),
                        depth_frame.get_width(),
                    )
                    scale = float(depth_frame.get_depth_scale())
                    depth_mm = np.clip(
                        np.rint(raw_depth.astype(np.float32) * scale),
                        0,
                        np.iinfo(np.uint16).max,
                    ).astype(np.uint16)
                sample = RGBDFrame(
                    color=np.ascontiguousarray(color.copy()),
                    depth_mm=None
                    if depth_mm is None
                    else np.ascontiguousarray(depth_mm.copy()),
                    captured_at_ns=time.monotonic_ns(),
                )
                with self._lock:
                    self._latest = sample
                self._new_frame_event.set()
        except BaseException as exc:
            with self._lock:
                self._read_error = exc
            self._new_frame_event.set()

    def read(self) -> np.ndarray:
        return self.async_read()

    def async_read(self, timeout_ms: float = 200.0) -> np.ndarray:
        self._require_connected()
        self._new_frame_event.clear()
        if not self._new_frame_event.wait(timeout=timeout_ms / 1000.0):
            self._raise_read_error()
            raise TimeoutError(f"{self.config.name} 等待新彩色帧超时")
        return self.read_latest_rgbd(max_age_ms=max(500, int(timeout_ms))).color

    def read_latest(self, max_age_ms: int = 500) -> np.ndarray:
        return self.read_latest_rgbd(max_age_ms=max_age_ms).color

    def read_latest_rgbd(self, max_age_ms: int = 500) -> RGBDFrame:
        self._require_connected()
        self._raise_read_error()
        with self._lock:
            latest = self._latest
        if latest is None:
            raise RuntimeError(f"{self.config.name} 尚无可用图像")
        age_ms = (time.monotonic_ns() - latest.captured_at_ns) / 1_000_000
        if age_ms > max_age_ms:
            raise TimeoutError(
                f"{self.config.name} 最新图像已过期: {age_ms:.1f} ms > {max_age_ms} ms"
            )
        return RGBDFrame(
            color=latest.color.copy(),
            depth_mm=None if latest.depth_mm is None else latest.depth_mm.copy(),
            captured_at_ns=latest.captured_at_ns,
        )

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise RuntimeError(f"{self.config.name} 未连接")

    def disconnect(self) -> None:
        if self._pipeline is None:
            return
        pipeline = self._pipeline
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._thread = None
        self._pipeline = None
        self._sdk = None
        pipeline.stop()
        with self._lock:
            self._latest = None
            self._read_error = None
        self._new_frame_event.clear()

"""Publish ROS topics for the Duckietown lane-following stack."""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass
from pathlib import Path
from threading import Condition
from typing import Any

import numpy as np
import rospy
import yaml
from duckietown_msgs.msg import (
    LEDPattern,
    WheelEncoderStamped,
    WheelsCmdStamped,
)
from sensor_msgs.msg import CameraInfo, CompressedImage


@dataclass(frozen=True)
class _RGB:
    r: float
    g: float
    b: float


_DEFAULT_ENCODER_RESOLUTION = 135
_CALIBRATION_DIR = Path("/data/config/calibrations/camera_intrinsic")


def _camera_frame_id(namespace: str) -> str:
    stripped_namespace = namespace.strip("/")
    return stripped_namespace + "/camera_optical_frame"


def _vehicle_name() -> str:
    return os.getenv("VEHICLE_NAME", "agent")


def _coerce_positive_int(value: object) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        coerced = value
    elif isinstance(value, float):
        coerced = int(value)
    elif isinstance(value, str):
        try:
            coerced = int(value)
        except ValueError:
            return None
    else:
        return None
    if coerced <= 0:
        return None
    return coerced


class ROSAgent:
    """Mirror Duckiematrix observations onto ROS topics."""

    _cali_file: Path
    _cali_file_folder: Path
    _cam_info_pub: Any
    _cam_pub: Any
    _current_camera_info: CameraInfo
    _frame_id: str
    _ik_action_sub: Any
    _last_action_timestamp: float | None
    _left_encoder_pub: Any
    _left_encoder_driver_pub: Any
    _led_sub: Any
    _leds: list[_RGB]
    _leds_initialized: bool
    _original_camera_info: CameraInfo
    _right_encoder_pub: Any
    _right_encoder_driver_pub: Any
    _action_condition: Condition
    _action_update_count: int
    action: np.ndarray
    initialized: bool
    updated: bool
    vehicle: str

    def __init__(self) -> None:
        """Initialize publishers, subscribers, and calibration."""
        self.vehicle = _vehicle_name()

        rospy.init_node(
            "ROSTemplate",
            log_level=rospy.DEBUG,
            disable_rosout=False,
        )

        action_topic = f"/{self.vehicle}/wheels_driver_node/wheels_cmd"
        self._ik_action_sub = rospy.Subscriber(
            action_topic,
            WheelsCmdStamped,
            self._ik_action_cb,
        )
        led_topic = f"/{self.vehicle}/led_emitter_node/led_pattern"
        self._led_sub = rospy.Subscriber(led_topic, LEDPattern, self._led_cb)

        self.action = np.array([0, 0], dtype=float)
        self._action_condition = Condition()
        self._action_update_count = 0
        self._last_action_timestamp = None
        self.updated = True
        self.initialized = False
        self._leds_initialized = False
        self._leds = [_RGB(1, 1, 1)] * 5

        image_topic = f"/{self.vehicle}/camera_node/image/compressed"
        self._cam_pub = rospy.Publisher(
            image_topic,
            CompressedImage,
            queue_size=10,
        )

        camera_info_topic = f"/{self.vehicle}/camera_node/camera_info"
        self._cam_info_pub = rospy.Publisher(
            camera_info_topic,
            CameraInfo,
            queue_size=1,
        )

        left_encoder_topic = f"/{self.vehicle}/left_wheel_encoder_node/tick"
        self._left_encoder_pub = rospy.Publisher(
            left_encoder_topic,
            WheelEncoderStamped,
            queue_size=1,
        )
        left_encoder_driver_topic = (
            f"/{self.vehicle}/left_wheel_encoder_driver_node/tick"
        )
        self._left_encoder_driver_pub = rospy.Publisher(
            left_encoder_driver_topic,
            WheelEncoderStamped,
            queue_size=1,
        )
        right_encoder_topic = f"/{self.vehicle}/right_wheel_encoder_node/tick"
        self._right_encoder_pub = rospy.Publisher(
            right_encoder_topic,
            WheelEncoderStamped,
            queue_size=1,
        )
        right_encoder_driver_topic = (
            f"/{self.vehicle}/right_wheel_encoder_driver_node/tick"
        )
        self._right_encoder_driver_pub = rospy.Publisher(
            right_encoder_driver_topic,
            WheelEncoderStamped,
            queue_size=1,
        )

        self._cali_file_folder = _CALIBRATION_DIR
        namespace = rospy.get_namespace()
        self._frame_id = _camera_frame_id(namespace)
        self._cali_file = self._cali_file_folder / f"{self.vehicle}.yaml"
        if not self._cali_file.is_file():
            warning_message = (
                f"Calibration not found: {self._cali_file}.\n"
                "Using default instead."
            )
            rospy.logwarn(warning_message)
            self._cali_file = self._cali_file_folder / "default.yaml"

        if not self._cali_file.is_file():
            rospy.signal_shutdown("Found no calibration file. Aborting")

        calibration_file = self._cali_file
        self._original_camera_info = self._load_camera_info(calibration_file)
        self._original_camera_info.header.frame_id = self._frame_id
        self._current_camera_info = copy.deepcopy(self._original_camera_info)
        rospy.loginfo(f"Using calibration file: {self._cali_file}")
        rospy.loginfo("Just after init_node.")

    def _ik_action_cb(self, msg: WheelsCmdStamped) -> None:
        """Store the latest inverse-kinematics wheel command."""
        stamp = msg.header.stamp.to_sec()
        wheel_velocities = [msg.vel_left, msg.vel_right]
        with self._action_condition:
            self.initialized = True
            self.action = np.array(wheel_velocities, dtype=float)
            self.updated = True
            self._action_update_count += 1
            self._last_action_timestamp = stamp
            self._action_condition.notify_all()

    def action_state(self) -> tuple[int, float | None]:
        """Return the latest action counter and command timestamp."""
        with self._action_condition:
            return self._action_update_count, self._last_action_timestamp

    def wait_for_action_update(
        self,
        min_update_count: int,
        min_timestamp: float,
        timeout: float,
    ) -> bool:
        """Wait for a wheel command produced from a recent frame."""

        def _has_fresh_action() -> bool:
            if self._action_update_count <= min_update_count:
                return False
            action_timestamp = self._last_action_timestamp
            if action_timestamp is None:
                return False
            return action_timestamp >= min_timestamp

        with self._action_condition:
            return self._action_condition.wait_for(_has_fresh_action, timeout)

    def _led_cb(self, msg: LEDPattern) -> None:
        """Store the latest LED pattern emitted by the controller."""
        self._leds_initialized = True
        for index in range(5):
            rgb = msg.rgb_vals[index]
            self._leds[index] = _RGB(rgb.r, rgb.g, rgb.b)
        self.updated = True

    def publish_info(
        self,
        timestamp: float,
        camera: dict[str, Any] | None = None,
    ) -> None:
        """Publish the current camera info message."""
        if camera:
            self._update_camera_info_from_world(camera)
        stamp = rospy.Time.from_sec(timestamp)
        self._current_camera_info.header.stamp = stamp
        self._cam_info_pub.publish(self._current_camera_info)

    def _update_camera_info_from_world(self, camera: dict[str, Any]) -> None:
        """Update camera info using the selected camera calibration."""
        width = _coerce_positive_int(camera.get("width"))
        height = _coerce_positive_int(camera.get("height"))
        if width is None or height is None:
            return

        calibrated_camera_info = self._scaled_camera_info(width, height)
        calibrated_camera_info.header.frame_id = self._frame_id
        self._current_camera_info = calibrated_camera_info

    def publish_img(self, obs: bytes, timestamp: float) -> None:
        """Publish a compressed camera image."""
        if not obs:
            return

        stamp = rospy.Time.from_sec(timestamp)
        image_data = bytearray(obs)
        img_message = CompressedImage()
        img_message.header.stamp = stamp
        img_message.format = "jpeg"
        img_message.data = image_data
        self._cam_pub.publish(img_message)

    def publish_encoder_ticks(
        self,
        left_ticks: int,
        right_ticks: int,
        timestamp: float,
        resolution: int = _DEFAULT_ENCODER_RESOLUTION,
    ) -> None:
        """Publish cumulative encoder ticks for both wheels."""
        stamp = rospy.Time.from_sec(timestamp)
        encoder_type = WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL

        left_message = WheelEncoderStamped(
            data=left_ticks,
            resolution=resolution,
            type=encoder_type,
        )
        left_message.header.stamp = stamp
        self._left_encoder_pub.publish(left_message)
        self._left_encoder_driver_pub.publish(left_message)

        right_message = WheelEncoderStamped(
            data=right_ticks,
            resolution=resolution,
            type=encoder_type,
        )
        right_message.header.stamp = stamp
        self._right_encoder_pub.publish(right_message)
        self._right_encoder_driver_pub.publish(right_message)

    @staticmethod
    def _load_camera_info(filename: Path) -> CameraInfo:
        """Load camera calibration matrices from a YAML file."""
        with filename.open() as stream:
            calibration_data = yaml.safe_load(stream)

        if not isinstance(calibration_data, dict):
            message = f"Unexpected calibration payload in {filename}."
            raise TypeError(message)

        cam_info = CameraInfo()
        cam_info.width = calibration_data["image_width"]
        cam_info.height = calibration_data["image_height"]
        cam_info.K = calibration_data["camera_matrix"]["data"]
        cam_info.D = calibration_data["distortion_coefficients"]["data"]
        cam_info.R = calibration_data["rectification_matrix"]["data"]
        cam_info.P = calibration_data["projection_matrix"]["data"]
        cam_info.distortion_model = calibration_data["distortion_model"]
        return cam_info

    def _scaled_camera_info(self, width: int, height: int) -> CameraInfo:
        """Scale the loaded calibration to the requested resolution."""
        return self._scaled_camera_info_from(
            self._original_camera_info,
            width,
            height,
        )

    @staticmethod
    def _scaled_camera_info_from(
        source_camera_info: CameraInfo,
        width: int,
        height: int,
    ) -> CameraInfo:
        """Scale a camera calibration to the requested resolution."""
        camera_info = copy.deepcopy(source_camera_info)
        if width == camera_info.width and height == camera_info.height:
            return camera_info

        scale_x = width / camera_info.width
        scale_y = height / camera_info.height
        camera_info.width = width
        camera_info.height = height

        camera_matrix = np.array(camera_info.K, dtype=float).reshape(3, 3)
        camera_matrix[0, 0] *= scale_x
        camera_matrix[0, 2] *= scale_x
        camera_matrix[1, 1] *= scale_y
        camera_matrix[1, 2] *= scale_y
        camera_info.K = camera_matrix.reshape(-1).tolist()

        projection_matrix = np.array(camera_info.P, dtype=float).reshape(3, 4)
        projection_matrix[0, 0] *= scale_x
        projection_matrix[0, 2] *= scale_x
        projection_matrix[0, 3] *= scale_x
        projection_matrix[1, 1] *= scale_y
        projection_matrix[1, 2] *= scale_y
        projection_matrix[1, 3] *= scale_y
        camera_info.P = projection_matrix.reshape(-1).tolist()

        return camera_info

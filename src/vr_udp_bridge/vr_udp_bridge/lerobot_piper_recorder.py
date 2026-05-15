import json
import os
import select
import shutil
import socket
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy


DEFAULT_PORT = 5020
HEADER_SEPARATOR = b"\n"
MAGIC = "RSIMG1"
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
CAMERA_NAMES = ("main", "left_wrist", "right_wrist")


def import_lerobot_dataset():
    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset

        return LeRobotDataset
    except ImportError:
        try:
            from lerobot.datasets import LeRobotDataset

            return LeRobotDataset
        except ImportError as exc:
            raise RuntimeError(
                "LeRobot is not installed. Install it with: pip3 install lerobot"
            ) from exc


def parse_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes", "on")


def pose_to_array(msg) -> np.ndarray:
    return np.array(
        [
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        ],
        dtype=np.float32,
    )


def pose_stamped_to_array(msg: PoseStamped) -> np.ndarray:
    return pose_to_array(msg.pose)


def joy_to_array(msg: Optional[Joy]) -> np.ndarray:
    values = np.zeros(20, dtype=np.float32)
    if msg is None:
        return values
    axis_count = min(10, len(msg.axes))
    button_count = min(10, len(msg.buttons))
    if axis_count:
        values[:axis_count] = np.asarray(msg.axes[:axis_count], dtype=np.float32)
    if button_count:
        values[10 : 10 + button_count] = np.asarray(
            msg.buttons[:button_count],
            dtype=np.float32,
        )
    return values


def joint_state_to_array(msg: JointState) -> Optional[np.ndarray]:
    if msg.name:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            return np.array([positions[name] for name in JOINT_NAMES], dtype=np.float32)
        if all(name in positions for name in JOINT_NAMES[:6]) and len(msg.position) >= 7:
            return np.array(
                [positions[name] for name in JOINT_NAMES[:6]] + [msg.position[6]],
                dtype=np.float32,
            )
    if len(msg.position) >= 7:
        return np.asarray(msg.position[:7], dtype=np.float32)
    return None


class UdpImageReceiver(threading.Thread):
    def __init__(
        self,
        name: str,
        bind_address: str,
        port: int,
        recv_buffer: int,
        max_datagram_size: int,
        frame_timeout: float,
        logger,
    ) -> None:
        super().__init__(daemon=True)
        self.name = name
        self.bind_address = bind_address
        self.port = port
        self.recv_buffer = recv_buffer
        self.max_datagram_size = max_datagram_size
        self.frame_timeout = frame_timeout
        self.logger = logger
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_header: Optional[dict] = None
        self.latest_time = 0.0
        self.frames: Dict[Tuple[int, str], Dict[str, object]] = {}
        self.sock: Optional[socket.socket] = None

    def latest(self) -> Tuple[Optional[np.ndarray], Optional[dict], float]:
        with self.lock:
            if self.latest_rgb is None:
                return None, None, 0.0
            return self.latest_rgb.copy(), dict(self.latest_header or {}), self.latest_time

    def stop(self) -> None:
        self.stop_event.set()
        if self.sock is not None:
            self.sock.close()

    def run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock = sock
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, self.recv_buffer)
        sock.settimeout(0.2)
        sock.bind((self.bind_address, self.port))
        self.logger.info(
            "LeRobot %s image receiver listening on %s:%d"
            % (self.name, self.bind_address, self.port)
        )
        while not self.stop_event.is_set():
            try:
                packet, _ = sock.recvfrom(self.max_datagram_size)
            except socket.timeout:
                self._prune_old_frames()
                continue
            except OSError:
                break
            self._handle_packet(packet)

    def _handle_packet(self, packet: bytes) -> None:
        header_bytes, separator, payload = packet.partition(HEADER_SEPARATOR)
        if not separator:
            return
        try:
            header = json.loads(header_bytes.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            return
        if header.get("magic") != MAGIC:
            return
        if header.get("stream") != "color":
            return

        frame_id = int(header["frame_id"])
        stream = str(header["stream"])
        chunk_index = int(header["chunk_index"])
        chunk_count = int(header["chunk_count"])
        key = (frame_id, stream)
        frame = self.frames.get(key)
        if frame is None:
            frame = {"created_at": time.monotonic(), "header": header, "chunks": {}}
            self.frames[key] = frame
        frame["chunks"][chunk_index] = payload
        if len(frame["chunks"]) != chunk_count:
            self._prune_old_frames()
            return

        try:
            image_bytes = b"".join(frame["chunks"][idx] for idx in range(chunk_count))
        except KeyError:
            self.frames.pop(key, None)
            return
        self.frames.pop(key, None)

        image = self._decode_color(image_bytes, str(header.get("encoding", "")))
        if image is None:
            return
        with self.lock:
            self.latest_rgb = image
            self.latest_header = header
            self.latest_time = time.monotonic()

    def _decode_color(self, image_bytes: bytes, encoding: str) -> Optional[np.ndarray]:
        if encoding != "jpeg":
            return None
        data = np.frombuffer(image_bytes, dtype=np.uint8)
        image_bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image_bgr is None:
            return None
        return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)

    def _prune_old_frames(self) -> None:
        now = time.monotonic()
        stale = [
            key
            for key, frame in self.frames.items()
            if now - float(frame["created_at"]) > self.frame_timeout
        ]
        for key in stale:
            self.frames.pop(key, None)


class KeyboardReader(threading.Thread):
    def __init__(self, on_key, logger) -> None:
        super().__init__(daemon=True)
        self.on_key = on_key
        self.logger = logger
        self.stop_event = threading.Event()
        self.enabled = sys.stdin.isatty()
        self.old_settings = None

    def stop(self) -> None:
        self.stop_event.set()
        if self.old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def run(self) -> None:
        if not self.enabled:
            self.logger.warn(
                "stdin is not a TTY; keyboard controls are disabled. "
                "Run with ros2 run for s/e/q controls."
            )
            return
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.logger.info("Keyboard controls: s=start, e=save episode, q=quit")
        try:
            while not self.stop_event.is_set():
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key:
                    self.on_key(key)
        finally:
            self.stop()


class LeRobotPiperRecorder(Node):
    def __init__(self) -> None:
        super().__init__("lerobot_piper_recorder")
        self._declare_parameters()
        self._load_parameters()

        self.lock = threading.Lock()
        self.dataset_lock = threading.RLock()
        self.joint_state = {"left": None, "right": None}
        self.joint_action = {"left": None, "right": None}
        self.ee_pose = {"left": None, "right": None}
        self.vr_pose = {"left": None, "right": None}
        self.vr_joy = {"left": None, "right": None}

        self.recording = False
        self.episode_frames = 0
        self.skip_count = 0
        self.last_missing: Tuple[str, ...] = ()
        self.last_log_time = time.monotonic()

        self.dataset = self._open_dataset()
        self.image_receivers = {}
        for camera_name in CAMERA_NAMES:
            receiver = UdpImageReceiver(
                camera_name,
                self.image_bind,
                self.camera_ports[camera_name],
                self.recv_buffer,
                self.max_datagram_size,
                self.frame_timeout,
                self.get_logger(),
            )
            self.image_receivers[camera_name] = receiver
            receiver.start()
        self.keyboard = KeyboardReader(self._on_key, self.get_logger())
        self.keyboard.start()

        self._subscribe_arm("left")
        self._subscribe_arm("right")
        self.timer = self.create_timer(1.0 / self.fps, self._on_timer)
        self.get_logger().info(
            "LeRobot recorder ready: repo_id=%s root=%s fps=%d"
            % (self.repo_id, self.root, self.fps)
        )
        self.get_logger().info(
            "camera mapping: main=%d, left_wrist=%d, right_wrist=%d"
            % (
                self.camera_ports["main"],
                self.camera_ports["left_wrist"],
                self.camera_ports["right_wrist"],
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("repo_id", "local/piper_teleoperation")
        self.declare_parameter(
            "root",
            os.path.expanduser("~/lerobot_datasets/piper_teleoperation_3cam"),
        )
        self.declare_parameter("fps", 30)
        self.declare_parameter("image_bind", "0.0.0.0")
        self.declare_parameter("image_port", DEFAULT_PORT)
        self.declare_parameter("main_image_port", 5020)
        self.declare_parameter("left_wrist_image_port", 5021)
        self.declare_parameter("right_wrist_image_port", 5022)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("main_image_width", 640)
        self.declare_parameter("main_image_height", 480)
        self.declare_parameter("left_wrist_image_width", 640)
        self.declare_parameter("left_wrist_image_height", 480)
        self.declare_parameter("right_wrist_image_width", 640)
        self.declare_parameter("right_wrist_image_height", 480)
        self.declare_parameter("task", "teleoperate dual Piper arms")
        self.declare_parameter("object_name", "")
        self.declare_parameter("box_position", "")
        self.declare_parameter("start_position", "")
        self.declare_parameter("success", True)
        self.declare_parameter("note", "")
        self.declare_parameter("robot_type", "piper_dual_vr")
        self.declare_parameter("use_videos", True)
        self.declare_parameter("vcodec", "h264")
        self.declare_parameter("recv_buffer", 4 * 1024 * 1024)
        self.declare_parameter("max_datagram_size", 65535)
        self.declare_parameter("frame_timeout", 1.0)
        self.declare_parameter("image_timeout", 1.0)

    def _load_parameters(self) -> None:
        self.repo_id = str(self.get_parameter("repo_id").value)
        self.root = Path(
            os.path.expanduser(str(self.get_parameter("root").value))
        )
        self.fps = int(self.get_parameter("fps").value)
        self.image_bind = str(self.get_parameter("image_bind").value)
        self.image_port = int(self.get_parameter("image_port").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.camera_ports = {
            "main": int(self.get_parameter("main_image_port").value),
            "left_wrist": int(self.get_parameter("left_wrist_image_port").value),
            "right_wrist": int(self.get_parameter("right_wrist_image_port").value),
        }
        self.camera_shapes = {
            "main": (
                int(self.get_parameter("main_image_height").value),
                int(self.get_parameter("main_image_width").value),
            ),
            "left_wrist": (
                int(self.get_parameter("left_wrist_image_height").value),
                int(self.get_parameter("left_wrist_image_width").value),
            ),
            "right_wrist": (
                int(self.get_parameter("right_wrist_image_height").value),
                int(self.get_parameter("right_wrist_image_width").value),
            ),
        }
        self.task = str(self.get_parameter("task").value)
        self.object_name = str(self.get_parameter("object_name").value)
        self.box_position = str(self.get_parameter("box_position").value)
        self.start_position = str(self.get_parameter("start_position").value)
        self.success = parse_bool(self.get_parameter("success").value)
        self.note = str(self.get_parameter("note").value)
        self.robot_type = str(self.get_parameter("robot_type").value)
        self.use_videos = parse_bool(self.get_parameter("use_videos").value)
        self.vcodec = str(self.get_parameter("vcodec").value)
        self.recv_buffer = int(self.get_parameter("recv_buffer").value)
        self.max_datagram_size = int(self.get_parameter("max_datagram_size").value)
        self.frame_timeout = float(self.get_parameter("frame_timeout").value)
        self.image_timeout = float(self.get_parameter("image_timeout").value)
        if self.fps <= 0:
            raise ValueError("fps must be greater than zero")

    def _open_dataset(self):
        LeRobotDataset = import_lerobot_dataset()
        features = self._features()
        info_path = self.root / "meta" / "info.json"
        if self._dataset_complete():
            self.get_logger().info("Loading existing LeRobot dataset at %s" % self.root)
            try:
                return LeRobotDataset(repo_id=self.repo_id, root=self.root)
            except TypeError:
                return LeRobotDataset(self.repo_id, root=self.root)
        if self.root.exists():
            backup = self._backup_incomplete_dataset()
            self.get_logger().warn(
                "Existing dataset at %s is incomplete or incompatible; moved to %s"
                % (self.root, backup)
            )

        self.get_logger().info("Creating LeRobot dataset at %s" % self.root)
        return LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=self.fps,
            features=features,
            root=self.root,
            robot_type=self.robot_type,
            use_videos=self.use_videos,
            vcodec=self.vcodec,
        )

    def _dataset_complete(self) -> bool:
        required = [
            self.root / "meta" / "info.json",
            self.root / "meta" / "tasks.parquet",
        ]
        if not all(path.exists() for path in required):
            return False
        episodes_dir = self.root / "meta" / "episodes"
        legacy_episodes = self.root / "meta" / "episodes.parquet"
        if not legacy_episodes.exists() and not any(
            episodes_dir.glob("chunk-*/*.parquet")
        ):
            return False
        try:
            with open(self.root / "meta" / "info.json", "r", encoding="utf-8") as f:
                info = json.load(f)
        except (OSError, json.JSONDecodeError):
            return False
        features = info.get("features", {})
        expected_image_dtype = "video" if self.use_videos else "image"
        for key in (
            "observation.images.main",
            "observation.images.left_wrist",
            "observation.images.right_wrist",
        ):
            if features.get(key, {}).get("dtype") != expected_image_dtype:
                return False
        return all(
            key in features
            for key in (
                "observation.images.main",
                "observation.images.left_wrist",
                "observation.images.right_wrist",
                "observation.state",
                "action",
            )
        )

    def _backup_incomplete_dataset(self) -> Path:
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        backup = self.root.with_name("%s_incomplete_%s" % (self.root.name, timestamp))
        suffix = 1
        while backup.exists():
            backup = self.root.with_name(
                "%s_incomplete_%s_%d" % (self.root.name, timestamp, suffix)
            )
            suffix += 1
        shutil.move(str(self.root), str(backup))
        return backup

    def _features(self) -> Dict[str, dict]:
        image_dtype = "video" if self.use_videos else "image"
        joint_names = [
            "left_joint1",
            "left_joint2",
            "left_joint3",
            "left_joint4",
            "left_joint5",
            "left_joint6",
            "left_joint7",
            "right_joint1",
            "right_joint2",
            "right_joint3",
            "right_joint4",
            "right_joint5",
            "right_joint6",
            "right_joint7",
        ]
        pose_names = [
            "left_x",
            "left_y",
            "left_z",
            "left_qx",
            "left_qy",
            "left_qz",
            "left_qw",
            "right_x",
            "right_y",
            "right_z",
            "right_qx",
            "right_qy",
            "right_qz",
            "right_qw",
        ]
        joy_names = (
            [f"left_axis{i}" for i in range(10)]
            + [f"left_button{i}" for i in range(10)]
            + [f"right_axis{i}" for i in range(10)]
            + [f"right_button{i}" for i in range(10)]
        )
        return {
            "observation.images.main": {
                "dtype": image_dtype,
                "shape": (*self.camera_shapes["main"], 3),
                "names": ["height", "width", "channel"],
            },
            "observation.images.left_wrist": {
                "dtype": image_dtype,
                "shape": (*self.camera_shapes["left_wrist"], 3),
                "names": ["height", "width", "channel"],
            },
            "observation.images.right_wrist": {
                "dtype": image_dtype,
                "shape": (*self.camera_shapes["right_wrist"], 3),
                "names": ["height", "width", "channel"],
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (14,),
                "names": joint_names,
            },
            "action": {
                "dtype": "float32",
                "shape": (14,),
                "names": joint_names,
            },
            "observation.ee_pose": {
                "dtype": "float32",
                "shape": (14,),
                "names": pose_names,
            },
            "observation.vr_pose": {
                "dtype": "float32",
                "shape": (14,),
                "names": pose_names,
            },
            "observation.vr_joy": {
                "dtype": "float32",
                "shape": (40,),
                "names": joy_names,
            },
        }

    def _subscribe_arm(self, side: str) -> None:
        self.create_subscription(
            JointState,
            f"/{side}/joint_states_single",
            lambda msg, arm_side=side: self._on_joint_state(arm_side, msg),
            20,
        )
        self.create_subscription(
            JointState,
            f"/{side}/joint_ctrl_single",
            lambda msg, arm_side=side: self._on_joint_action(arm_side, msg),
            20,
        )
        self.create_subscription(
            Pose,
            f"/{side}/end_pose",
            lambda msg, arm_side=side: self._on_ee_pose(arm_side, msg),
            20,
        )
        self.create_subscription(
            PoseStamped,
            f"/vr/{side}_controller/pose",
            lambda msg, arm_side=side: self._on_vr_pose(arm_side, msg),
            20,
        )
        self.create_subscription(
            Joy,
            f"/vr/{side}_controller/joy",
            lambda msg, arm_side=side: self._on_vr_joy(arm_side, msg),
            20,
        )

    def _on_joint_state(self, side: str, msg: JointState) -> None:
        value = joint_state_to_array(msg)
        if value is not None:
            with self.lock:
                self.joint_state[side] = value

    def _on_joint_action(self, side: str, msg: JointState) -> None:
        value = joint_state_to_array(msg)
        if value is not None:
            with self.lock:
                self.joint_action[side] = value

    def _on_ee_pose(self, side: str, msg: Pose) -> None:
        with self.lock:
            self.ee_pose[side] = pose_to_array(msg)

    def _on_vr_pose(self, side: str, msg: PoseStamped) -> None:
        with self.lock:
            self.vr_pose[side] = pose_stamped_to_array(msg)

    def _on_vr_joy(self, side: str, msg: Joy) -> None:
        with self.lock:
            self.vr_joy[side] = joy_to_array(msg)

    def _on_key(self, key: str) -> None:
        try:
            if key == "s":
                self._start_episode()
            elif key == "e":
                self._save_episode()
            elif key == "q":
                self.get_logger().info("Quit requested")
                if self.recording:
                    self.recording = False
                    self._clear_episode_buffer(delete_images=False)
                    self.get_logger().info("Unsaved recording discarded")
                rclpy.shutdown()
        except Exception as exc:
            self.recording = False
            self.get_logger().error("Keyboard command failed: %s" % exc)

    def _start_episode(self) -> None:
        if self.recording:
            self.get_logger().warn("Already recording; ignoring start command")
            return
        self.recording = True
        self.episode_frames = 0
        self.skip_count = 0
        self.last_log_time = time.monotonic()
        self.get_logger().info("Recording started")

    def _save_episode(self) -> None:
        if not self.recording:
            self.get_logger().warn("Not recording; ignoring save command")
            return
        if self.episode_frames == 0:
            self.recording = False
            self.get_logger().warn("Episode has no frames; discarded")
            self._clear_episode_buffer()
            return
        self.recording = False
        frames_to_save = self.episode_frames
        self.get_logger().info("Saving episode with %d frames" % frames_to_save)
        episode_index = int(getattr(self.dataset.meta, "total_episodes", 0))
        with self.dataset_lock:
            try:
                self.dataset.save_episode()
            except TypeError:
                self.dataset.save_episode(task=self.task)
            self._append_episode_metadata(episode_index, frames_to_save)
        self.recording = False
        self.episode_frames = 0
        self.skip_count = 0
        self.get_logger().info("Episode saved")

    def _append_episode_metadata(self, episode_index: int, length: int) -> None:
        metadata = {
            "episode_index": episode_index,
            "length": length,
            "task": self.task,
            "object_name": self.object_name,
            "box_position": self.box_position,
            "start_position": self.start_position,
            "success": self.success,
            "note": self.note,
            "saved_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        metadata_path = self.root / "meta" / "episode_metadata.jsonl"
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        with metadata_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(metadata, ensure_ascii=False, sort_keys=True) + "\n")

    def _clear_episode_buffer(self, delete_images: bool = True) -> None:
        with self.dataset_lock:
            if hasattr(self.dataset, "clear_episode_buffer"):
                self.dataset.clear_episode_buffer(delete_images=delete_images)
            elif hasattr(self.dataset, "episode_buffer"):
                self.dataset.episode_buffer = {}

    def _on_timer(self) -> None:
        if not self.recording:
            return
        with self.dataset_lock:
            if not self.recording:
                return
            frame, missing = self._make_frame()
            if frame is None:
                self.skip_count += 1
                if missing:
                    self.last_missing = missing
            else:
                self._ensure_image_dirs()
                self.dataset.add_frame(frame)
                self.episode_frames += 1
        self._log_progress()

    def _ensure_image_dirs(self) -> None:
        if not hasattr(self.dataset, "_get_image_file_dir"):
            return
        episode_buffer = getattr(self.dataset, "episode_buffer", None)
        if episode_buffer is None:
            episode_index = self.dataset.meta.total_episodes
        else:
            episode_index = episode_buffer.get(
                "episode_index",
                self.dataset.meta.total_episodes,
            )
            if isinstance(episode_index, np.ndarray):
                episode_index = (
                    episode_index.item()
                    if episode_index.size == 1
                    else episode_index[0]
                )
        for image_key in (
            "observation.images.main",
            "observation.images.left_wrist",
            "observation.images.right_wrist",
        ):
            self.dataset._get_image_file_dir(int(episode_index), image_key).mkdir(
                parents=True,
                exist_ok=True,
            )

    def _make_frame(self) -> Tuple[Optional[dict], Tuple[str, ...]]:
        images = {}
        missing = []
        now = time.monotonic()
        for camera_name, receiver in self.image_receivers.items():
            image, _, image_time = receiver.latest()
            if image is None or now - image_time > self.image_timeout:
                missing.append("image:%s" % camera_name)
                continue
            target_height, target_width = self.camera_shapes[camera_name]
            if image.shape[:2] != (target_height, target_width):
                image = cv2.resize(image, (target_width, target_height))
            images[camera_name] = image
        if len(images) != len(CAMERA_NAMES):
            return None, tuple(missing)

        with self.lock:
            if (
                self.joint_state["left"] is None
                or self.joint_state["right"] is None
                or self.joint_action["left"] is None
                or self.joint_action["right"] is None
            ):
                if self.joint_state["left"] is None:
                    missing.append("state:left")
                if self.joint_state["right"] is None:
                    missing.append("state:right")
                if self.joint_action["left"] is None:
                    missing.append("action:left")
                if self.joint_action["right"] is None:
                    missing.append("action:right")
                return None, tuple(missing)
            state = np.concatenate(
                (self.joint_state["left"], self.joint_state["right"])
            ).astype(np.float32)
            action = np.concatenate(
                (self.joint_action["left"], self.joint_action["right"])
            ).astype(np.float32)
            ee_pose = np.concatenate(
                (
                    self.ee_pose["left"]
                    if self.ee_pose["left"] is not None
                    else np.zeros(7, dtype=np.float32),
                    self.ee_pose["right"]
                    if self.ee_pose["right"] is not None
                    else np.zeros(7, dtype=np.float32),
                )
            ).astype(np.float32)
            vr_pose = np.concatenate(
                (
                    self.vr_pose["left"]
                    if self.vr_pose["left"] is not None
                    else np.zeros(7, dtype=np.float32),
                    self.vr_pose["right"]
                    if self.vr_pose["right"] is not None
                    else np.zeros(7, dtype=np.float32),
                )
            ).astype(np.float32)
            vr_joy = np.concatenate(
                (
                    self.vr_joy["left"]
                    if self.vr_joy["left"] is not None
                    else np.zeros(20, dtype=np.float32),
                    self.vr_joy["right"]
                    if self.vr_joy["right"] is not None
                    else np.zeros(20, dtype=np.float32),
                )
            ).astype(np.float32)

        return {
            "observation.images.main": images["main"],
            "observation.images.left_wrist": images["left_wrist"],
            "observation.images.right_wrist": images["right_wrist"],
            "observation.state": state,
            "action": action,
            "task": self.task,
            "observation.ee_pose": ee_pose,
            "observation.vr_pose": vr_pose,
            "observation.vr_joy": vr_joy,
        }, ()

    def _log_progress(self) -> None:
        now = time.monotonic()
        if now - self.last_log_time < 1.0:
            return
        self.last_log_time = now
        missing_text = ""
        if getattr(self, "last_missing", None):
            missing_text = " missing=%s" % ",".join(self.last_missing)
        self.get_logger().info(
            "recording frames=%d skipped=%d%s"
            % (self.episode_frames, self.skip_count, missing_text)
        )

    def destroy_node(self) -> bool:
        for receiver in getattr(self, "image_receivers", {}).values():
            receiver.stop()
        self.keyboard.stop()
        if hasattr(self.dataset, "finalize"):
            try:
                self.dataset.finalize()
            except Exception as exc:
                self.get_logger().warn("LeRobot dataset finalize failed: %s" % exc)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = LeRobotPiperRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from exc
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

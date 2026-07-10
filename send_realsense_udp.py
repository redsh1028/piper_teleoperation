import argparse
import json
import math
import socket
import time
from typing import Dict, Iterable, Optional, Tuple


DEFAULT_HOST = "172.16.64.161"
DEFAULT_PORT = 5020
DEFAULT_MAX_PACKET_SIZE = 1400
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30
DEFAULT_JPEG_QUALITY = 60
HEADER_SEPARATOR = b"\n"
MAGIC = "RSIMG1"

cv2 = None
np = None
rs = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send Intel RealSense camera images over UDP."
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_HOST,
        help="UDP receiver IP. Use comma-separated values to send to multiple receivers.",
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--width", type=int, default=DEFAULT_WIDTH)
    parser.add_argument("--height", type=int, default=DEFAULT_HEIGHT)
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS)
    parser.add_argument(
        "--serial",
        default="",
        help="RealSense serial number to open when using the realsense backend",
    )
    parser.add_argument(
        "--stream",
        choices=("color", "depth", "both"),
        default="color",
        help="Image stream to send",
    )
    parser.add_argument(
        "--backend",
        choices=("auto", "realsense", "v4l2"),
        default="auto",
        help="Camera backend. auto falls back to V4L2 for color if RealSense SDK sees no device.",
    )
    parser.add_argument(
        "--v4l2-device",
        default="0",
        help="OpenCV/V4L2 camera index used when --backend v4l2 or auto fallback is active.",
    )
    parser.add_argument(
        "--v4l2-fourcc",
        default="YUYV",
        help="FourCC requested for V4L2 capture. Use empty string to leave unchanged.",
    )
    parser.add_argument(
        "--v4l2-max-read-failures",
        type=int,
        default=30,
        help="Consecutive V4L2 read failures before reopening the camera.",
    )
    parser.add_argument(
        "--v4l2-reopen-delay",
        type=float,
        default=1.0,
        help="Seconds to wait before reopening V4L2 after repeated read failures.",
    )
    parser.add_argument(
        "--crop",
        choices=("none", "left-half", "right-half"),
        default="none",
        help="Optional crop applied before encoding. Useful for ZED stereo V4L2 frames.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=DEFAULT_JPEG_QUALITY,
        help="JPEG quality for color images, 1-100",
    )
    parser.add_argument(
        "--max-packet-size",
        type=int,
        default=DEFAULT_MAX_PACKET_SIZE,
        help="Maximum UDP payload size in bytes",
    )
    parser.add_argument(
        "--depth-visualize",
        action="store_true",
        help="Send depth as an 8-bit colorized JPEG instead of 16-bit PNG",
    )
    parser.add_argument(
        "--depth-scale-alpha",
        type=float,
        default=0.03,
        help="Alpha used by cv2.convertScaleAbs when --depth-visualize is enabled",
    )
    parser.add_argument(
        "--color-auto-exposure",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable RealSense color auto exposure. Use --no-color-auto-exposure for fixed shutter.",
    )
    parser.add_argument(
        "--color-exposure",
        type=float,
        default=-1.0,
        help="RealSense color exposure in microseconds when auto exposure is disabled. Example: 5000 for 5 ms.",
    )
    parser.add_argument(
        "--color-gain",
        type=float,
        default=-1.0,
        help="RealSense color gain. Use a higher value if short exposure is too dark.",
    )
    parser.add_argument(
        "--color-auto-white-balance",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable RealSense color auto white balance. Use --no-color-auto-white-balance for fixed color.",
    )
    parser.add_argument(
        "--color-white-balance",
        type=float,
        default=-1.0,
        help="RealSense color white balance in Kelvin when auto white balance is disabled.",
    )
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.width <= 0 or args.height <= 0:
        raise ValueError("--width and --height must be greater than zero")
    if args.fps <= 0:
        raise ValueError("--fps must be greater than zero")
    if not 1 <= args.jpeg_quality <= 100:
        raise ValueError("--jpeg-quality must be between 1 and 100")
    if args.max_packet_size < 512:
        raise ValueError("--max-packet-size must be at least 512 bytes")
    if args.backend == "v4l2" and args.stream != "color":
        raise ValueError("--backend v4l2 supports only --stream color")
    if args.backend == "v4l2" and args.serial:
        raise ValueError("--serial can only be used with the realsense backend")


def parse_addresses(host_text: str, port: int):
    hosts = [host.strip() for host in host_text.split(",") if host.strip()]
    if not hosts:
        raise ValueError("--host must contain at least one receiver IP")
    return [(host, port) for host in hosts]


def import_camera_modules() -> None:
    global cv2, np, rs
    try:
        import cv2 as cv2_module
        import numpy as np_module
    except (AttributeError, ImportError) as exc:
        raise RuntimeError(
            "RealSense UDP sender requires compatible cv2 and numpy. "
            "Try: pip3 install 'numpy<2' opencv-python"
        ) from exc

    try:
        import pyrealsense2 as rs_module
    except (AttributeError, ImportError):
        rs_module = None

    cv2 = cv2_module
    np = np_module
    rs = rs_module


def count_realsense_devices() -> int:
    if rs is None:
        return 0
    return len(rs.context().query_devices())


def available_realsense_devices() -> Dict[str, str]:
    if rs is None:
        return {}
    devices = {}
    for device in rs.context().query_devices():
        serial = device.get_info(rs.camera_info.serial_number)
        name = device.get_info(rs.camera_info.name)
        devices[serial] = name
    return devices


def select_backend(args: argparse.Namespace) -> str:
    if args.backend == "v4l2":
        return "v4l2"
    if args.serial:
        if rs is None:
            raise RuntimeError(
                "pyrealsense2 is required when --serial is provided. "
                "Try: pip3 install pyrealsense2"
            )
        return "realsense"
    if args.backend == "realsense":
        if rs is None:
            raise RuntimeError(
                "pyrealsense2 is required for --backend realsense. "
                "Try: pip3 install pyrealsense2"
            )
        return "realsense"

    if rs is not None and count_realsense_devices() > 0:
        return "realsense"
    if args.stream == "color":
        return "v4l2"
    raise RuntimeError(
        "RealSense SDK does not see a camera and V4L2 fallback only supports color."
    )


def configure_pipeline(args: argparse.Namespace):
    pipeline = rs.pipeline()
    config = rs.config()
    if args.serial:
        devices = available_realsense_devices()
        if args.serial not in devices:
            available = ", ".join(
                "%s (%s)" % (serial, name)
                for serial, name in sorted(devices.items())
            )
            if not available:
                available = "none"
            raise RuntimeError(
                "RealSense serial %s was not found. Available devices: %s"
                % (args.serial, available)
            )
        config.enable_device(args.serial)
    if args.stream in ("color", "both"):
        config.enable_stream(
            rs.stream.color,
            args.width,
            args.height,
            rs.format.bgr8,
            args.fps,
        )
    if args.stream in ("depth", "both"):
        config.enable_stream(
            rs.stream.depth,
            args.width,
            args.height,
            rs.format.z16,
            args.fps,
        )
    profile = pipeline.start(config)
    configure_realsense_color_options(profile, args)
    return pipeline, config


def configure_realsense_color_options(profile, args: argparse.Namespace) -> None:
    device = profile.get_device()
    color_sensor = None
    for sensor in device.query_sensors():
        try:
            sensor_name = sensor.get_info(rs.camera_info.name)
        except RuntimeError:
            sensor_name = ""
        if "RGB" in sensor_name or "Color" in sensor_name:
            color_sensor = sensor
            break

    if color_sensor is None:
        print("Warning: RealSense color sensor was not found; exposure unchanged.", flush=True)
        return

    if color_sensor.supports(rs.option.enable_auto_exposure):
        color_sensor.set_option(
            rs.option.enable_auto_exposure,
            1.0 if args.color_auto_exposure else 0.0,
        )

    if not args.color_auto_exposure:
        if args.color_exposure > 0.0:
            if color_sensor.supports(rs.option.exposure):
                color_sensor.set_option(rs.option.exposure, float(args.color_exposure))
            else:
                print("Warning: color exposure option is not supported.", flush=True)

        if args.color_gain >= 0.0:
            if color_sensor.supports(rs.option.gain):
                color_sensor.set_option(rs.option.gain, float(args.color_gain))
            else:
                print("Warning: color gain option is not supported.", flush=True)

    if color_sensor.supports(rs.option.enable_auto_white_balance):
        color_sensor.set_option(
            rs.option.enable_auto_white_balance,
            1.0 if args.color_auto_white_balance else 0.0,
        )

    if not args.color_auto_white_balance and args.color_white_balance > 0.0:
        if color_sensor.supports(rs.option.white_balance):
            color_sensor.set_option(rs.option.white_balance, float(args.color_white_balance))
        else:
            print("Warning: color white balance option is not supported.", flush=True)

    exposure_text = "auto"
    gain_text = "unchanged"
    white_balance_text = "auto"
    if color_sensor.supports(rs.option.exposure):
        exposure_text = "%.1f us" % color_sensor.get_option(rs.option.exposure)
    if color_sensor.supports(rs.option.gain):
        gain_text = "%.1f" % color_sensor.get_option(rs.option.gain)
    if color_sensor.supports(rs.option.white_balance):
        white_balance_text = "%.1f K" % color_sensor.get_option(rs.option.white_balance)
    print(
        "RealSense color exposure: auto=%s exposure=%s gain=%s white_balance_auto=%s white_balance=%s"
        % (
            args.color_auto_exposure,
            exposure_text,
            gain_text,
            args.color_auto_white_balance,
            white_balance_text,
        ),
        flush=True,
    )


def configure_v4l2(args: argparse.Namespace):
    device = parse_v4l2_device(args.v4l2_device)
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError("Failed to open V4L2 camera %s" % args.v4l2_device)
    if args.v4l2_fourcc:
        fourcc = cv2.VideoWriter_fourcc(*args.v4l2_fourcc[:4])
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def parse_v4l2_device(device_text: str):
    try:
        return int(device_text)
    except ValueError:
        return device_text


def encode_color_image(image, jpeg_quality: int) -> Tuple[bytes, Dict[str, object]]:
    ok, encoded = cv2.imencode(
        ".jpg",
        image,
        [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
    )
    if not ok:
        raise RuntimeError("Failed to JPEG-encode color frame")
    height, width = image.shape[:2]
    return encoded.tobytes(), {
        "stream": "color",
        "encoding": "jpeg",
        "width": int(width),
        "height": int(height),
        "format": "bgr8",
    }


def crop_image(image, crop: str):
    if crop == "none":
        return image
    height, width = image.shape[:2]
    half_width = width // 2
    if half_width <= 0:
        return image
    if crop == "left-half":
        return image[:, :half_width]
    if crop == "right-half":
        return image[:, width - half_width :]
    raise ValueError("Unsupported crop mode: %s" % crop)


def encode_color(frame, jpeg_quality: int) -> Tuple[bytes, Dict[str, object]]:
    image = np.asanyarray(frame.get_data())
    return encode_color_image(image, jpeg_quality)


def encode_depth(
    frame,
    visualize: bool,
    jpeg_quality: int,
    depth_scale_alpha: float,
) -> Tuple[bytes, Dict[str, object]]:
    depth = np.asanyarray(frame.get_data())
    if visualize:
        depth_8u = cv2.convertScaleAbs(depth, alpha=depth_scale_alpha)
        depth_color = cv2.applyColorMap(depth_8u, cv2.COLORMAP_JET)
        ok, encoded = cv2.imencode(
            ".jpg",
            depth_color,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)],
        )
        if not ok:
            raise RuntimeError("Failed to JPEG-encode depth visualization")
        return encoded.tobytes(), {
            "stream": "depth",
            "encoding": "jpeg_colormap",
            "width": int(frame.get_width()),
            "height": int(frame.get_height()),
            "format": "bgr8",
        }

    ok, encoded = cv2.imencode(".png", depth)
    if not ok:
        raise RuntimeError("Failed to PNG-encode depth frame")
    return encoded.tobytes(), {
        "stream": "depth",
        "encoding": "png",
        "width": int(frame.get_width()),
        "height": int(frame.get_height()),
        "format": "z16",
    }


def iter_chunks(data: bytes, chunk_size: int) -> Iterable[Tuple[int, bytes]]:
    for offset in range(0, len(data), chunk_size):
        yield offset // chunk_size, data[offset : offset + chunk_size]


def send_image(
    sock: socket.socket,
    address: Tuple[str, int],
    frame_id: int,
    timestamp: float,
    image_bytes: bytes,
    metadata: Dict[str, object],
    max_packet_size: int,
) -> None:
    total_size = len(image_bytes)
    chunk_size = max_packet_size - 512
    chunk_count = int(math.ceil(total_size / float(chunk_size)))

    for chunk_index, chunk in iter_chunks(image_bytes, chunk_size):
        header = {
            "magic": MAGIC,
            "frame_id": frame_id,
            "timestamp": timestamp,
            "chunk_index": chunk_index,
            "chunk_count": chunk_count,
            "total_size": total_size,
            "payload_size": len(chunk),
            **metadata,
        }
        header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
        packet = header_bytes + HEADER_SEPARATOR + chunk
        if len(packet) > max_packet_size:
            raise RuntimeError(
                "UDP packet is too large; lower --max-packet-size or metadata size"
            )
        sock.sendto(packet, address)


def main() -> None:
    args = parse_args()
    validate_args(args)
    import_camera_modules()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    addresses = parse_addresses(args.host, args.port)
    backend = select_backend(args)
    pipeline = None
    cap = None

    if backend == "realsense":
        pipeline, _ = configure_pipeline(args)
        serial_text = args.serial if args.serial else "default"
        print("RealSense pipeline started for serial %s." % serial_text, flush=True)
    else:
        cap = configure_v4l2(args)
        print(
            "V4L2 camera started on %s." % args.v4l2_device,
            flush=True,
        )
    print(
        "Sending %s stream with %s backend to %s at %dx%d %d FPS"
        % (
            args.stream,
            backend,
            ",".join("%s:%d" % address for address in addresses),
            args.width,
            args.height,
            args.fps,
        ),
        flush=True,
    )

    frame_id = 0
    v4l2_read_failures = 0
    try:
        while True:
            timestamp = time.time()

            if backend == "v4l2":
                ok, image = cap.read()
                if not ok:
                    v4l2_read_failures += 1
                    if v4l2_read_failures == 1 or (
                        v4l2_read_failures % args.v4l2_max_read_failures == 0
                    ):
                        print(
                            "Warning: failed to read frame from V4L2 camera %s (%d consecutive failures)"
                            % (args.v4l2_device, v4l2_read_failures),
                            flush=True,
                        )
                    if v4l2_read_failures >= args.v4l2_max_read_failures:
                        cap.release()
                        time.sleep(args.v4l2_reopen_delay)
                        cap = configure_v4l2(args)
                        print(
                            "Reopened V4L2 camera %s after read failures."
                            % args.v4l2_device,
                            flush=True,
                        )
                        v4l2_read_failures = 0
                    continue
                v4l2_read_failures = 0
                image = crop_image(image, args.crop)
                image_bytes, metadata = encode_color_image(
                    image,
                    args.jpeg_quality,
                )
                for address in addresses:
                    send_image(
                        sock,
                        address,
                        frame_id,
                        timestamp,
                        image_bytes,
                        metadata,
                        args.max_packet_size,
                    )
                frame_id += 1
                continue

            frames = pipeline.wait_for_frames()

            if args.stream in ("color", "both"):
                color_frame = frames.get_color_frame()
                if color_frame:
                    image_bytes, metadata = encode_color(
                        color_frame,
                        args.jpeg_quality,
                    )
                    for address in addresses:
                        send_image(
                            sock,
                            address,
                            frame_id,
                            timestamp,
                            image_bytes,
                            metadata,
                            args.max_packet_size,
                        )

            if args.stream in ("depth", "both"):
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    image_bytes, metadata = encode_depth(
                        depth_frame,
                        args.depth_visualize,
                        args.jpeg_quality,
                        args.depth_scale_alpha,
                    )
                    for address in addresses:
                        send_image(
                            sock,
                            address,
                            frame_id,
                            timestamp,
                            image_bytes,
                            metadata,
                            args.max_packet_size,
                        )

            frame_id += 1

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        if pipeline is not None:
            pipeline.stop()
        if cap is not None:
            cap.release()
        sock.close()


if __name__ == "__main__":
    main()

import argparse
import json
import select
import socket
import time
from typing import Dict, List, Tuple

import cv2
import numpy as np


DEFAULT_BIND = "0.0.0.0"
DEFAULT_PORT = 5020
DEFAULT_LEFT_PORT = 5021
DEFAULT_RIGHT_PORT = 5022
HEADER_SEPARATOR = b"\n"
MAGIC = "RSIMG1"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Receive RealSense UDP image chunks and display them."
    )
    parser.add_argument("--bind", default=DEFAULT_BIND, help="Local bind address")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--main-port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--left-port", type=int, default=DEFAULT_LEFT_PORT)
    parser.add_argument("--right-port", type=int, default=DEFAULT_RIGHT_PORT)
    parser.add_argument(
        "--three-camera",
        action="store_true",
        default=True,
        help="Display left/main/right camera ports in one combined window",
    )
    parser.add_argument(
        "--single-camera",
        action="store_false",
        dest="three_camera",
        help="Use the legacy single-port viewer controlled by --port",
    )
    parser.add_argument(
        "--tile-width",
        type=int,
        default=640,
        help="Display width for each camera tile in three-camera mode",
    )
    parser.add_argument(
        "--tile-height",
        type=int,
        default=480,
        help="Display height for each camera tile in three-camera mode",
    )
    parser.add_argument(
        "--recv-buffer",
        type=int,
        default=16 * 1024 * 1024,
        help="UDP socket receive buffer size",
    )
    parser.add_argument(
        "--max-packets-per-cycle",
        type=int,
        default=600,
        help="Maximum UDP packets to drain before each display refresh",
    )
    parser.add_argument(
        "--frame-timeout",
        type=float,
        default=1.0,
        help="Drop incomplete frames older than this many seconds",
    )
    parser.add_argument(
        "--max-datagram-size",
        type=int,
        default=65535,
        help="Maximum UDP datagram size to read",
    )
    return parser.parse_args()


def decode_image(image_bytes: bytes, encoding: str):
    data = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError("Failed to decode %s image" % encoding)
    if image.dtype == np.uint16:
        image = cv2.convertScaleAbs(image, alpha=0.03)
        image = cv2.applyColorMap(image, cv2.COLORMAP_JET)
    return image


def prune_old_frames(
    frames: Dict[Tuple[int, str], Dict[str, object]],
    timeout: float,
) -> None:
    now = time.time()
    stale = [
        key for key, frame in frames.items() if now - float(frame["created_at"]) > timeout
    ]
    for key in stale:
        frames.pop(key, None)


def open_socket(bind_address: str, port: int, recv_buffer: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buffer)
    sock.bind((bind_address, port))
    sock.setblocking(False)
    return sock


def receive_complete_image(
    sock: socket.socket,
    frames: Dict[Tuple[int, str], Dict[str, object]],
    max_datagram_size: int,
    frame_timeout: float,
):
    try:
        packet, addr = sock.recvfrom(max_datagram_size)
    except BlockingIOError:
        return None, None, None

    header_bytes, separator, payload = packet.partition(HEADER_SEPARATOR)
    if not separator:
        return None, None, None

    try:
        header = json.loads(header_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None, None, None

    if header.get("magic") != MAGIC:
        return None, None, None

    stream = str(header.get("stream", "unknown"))
    frame_id = int(header["frame_id"])
    chunk_index = int(header["chunk_index"])
    chunk_count = int(header["chunk_count"])
    key = (frame_id, stream)

    frame = frames.get(key)
    if frame is None:
        frame = {
            "created_at": time.time(),
            "header": header,
            "chunks": {},
            "addr": addr,
        }
        frames[key] = frame

    chunks = frame["chunks"]
    chunks[chunk_index] = payload

    if len(chunks) != chunk_count:
        prune_old_frames(frames, frame_timeout)
        return None, header, addr

    try:
        image_bytes = b"".join(chunks[i] for i in range(chunk_count))
    except KeyError:
        frames.pop(key, None)
        return None, header, addr

    frames.pop(key, None)
    image = decode_image(image_bytes, str(header.get("encoding", "")))
    return image, header, addr


def drain_socket(
    sock: socket.socket,
    frames: Dict[Tuple[int, str], Dict[str, object]],
    max_datagram_size: int,
    frame_timeout: float,
    max_packets: int,
):
    latest = (None, None, None)
    for _ in range(max_packets):
        image, header, addr = receive_complete_image(
            sock,
            frames,
            max_datagram_size,
            frame_timeout,
        )
        if image is None and header is None:
            break
        if image is not None:
            latest = (image, header, addr)
    return latest


def label_tile(image, label: str):
    output = image.copy()
    cv2.rectangle(output, (0, 0), (220, 34), (0, 0, 0), -1)
    cv2.putText(
        output,
        label,
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return output


def make_placeholder(width: int, height: int, label: str):
    image = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(
        image,
        "waiting for %s" % label,
        (30, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (180, 180, 180),
        2,
        cv2.LINE_AA,
    )
    return image


def resize_tile(image, width: int, height: int, label: str):
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    resized = cv2.resize(image, (width, height))
    return label_tile(resized, label)


def triangle_layout(left, main, right, tile_width: int, tile_height: int):
    left_tile = resize_tile(left, tile_width, tile_height, "left")
    right_tile = resize_tile(right, tile_width, tile_height, "right")
    main_tile = resize_tile(main, tile_width, tile_height, "main")
    canvas_width = tile_width * 2
    top_row = np.zeros((tile_height, canvas_width, 3), dtype=np.uint8)
    main_x = (canvas_width - tile_width) // 2
    top_row[:, main_x : main_x + tile_width] = main_tile
    bottom_row = np.hstack((left_tile, right_tile))
    return np.vstack((top_row, bottom_row))


def run_single_view(args: argparse.Namespace) -> None:
    sock = open_socket(args.bind, args.port, args.recv_buffer)
    print("Listening on %s:%d" % (args.bind, args.port), flush=True)
    print("Press q or ESC in an image window to quit.", flush=True)

    frames: Dict[Tuple[int, str], Dict[str, object]] = {}
    displayed = 0
    try:
        while True:
            readable, _, _ = select.select([sock], [], [], 0.1)
            if not readable:
                key_code = cv2.waitKey(1) & 0xFF
                if key_code in (27, ord("q")):
                    break
                continue
            image, header, addr = drain_socket(
                sock,
                frames,
                args.max_datagram_size,
                args.frame_timeout,
                args.max_packets_per_cycle,
            )
            if image is None:
                continue
            stream = str(header.get("stream", "unknown"))
            cv2.imshow("RealSense UDP %s" % stream, image)
            displayed += 1

            if displayed % 30 == 0:
                print(
                    "frame=%d stream=%s size=%dx%d chunks=%d from=%s:%d"
                    % (
                        int(header.get("frame_id", -1)),
                        stream,
                        int(header.get("width", 0)),
                        int(header.get("height", 0)),
                        int(header.get("chunk_count", 0)),
                        addr[0],
                        addr[1],
                    ),
                    flush=True,
                )

            key_code = cv2.waitKey(1) & 0xFF
            if key_code in (27, ord("q")):
                break
    finally:
        sock.close()


def run_three_camera_view(args: argparse.Namespace) -> None:
    camera_order = [
        ("left", args.left_port),
        ("main", args.main_port),
        ("right", args.right_port),
    ]
    sockets = {
        name: open_socket(args.bind, port, args.recv_buffer)
        for name, port in camera_order
    }
    frames = {name: {} for name, _ in camera_order}
    latest = {
        name: make_placeholder(args.tile_width, args.tile_height, name)
        for name, _ in camera_order
    }
    displayed = 0

    print(
        "Listening left=%d main=%d right=%d"
        % (args.left_port, args.main_port, args.right_port),
        flush=True,
    )
    print("Press q or ESC in the combined window to quit.", flush=True)

    try:
        while True:
            readable, _, _ = select.select(list(sockets.values()), [], [], 0.02)
            for sock in readable:
                name = next(
                    camera_name
                    for camera_name, camera_sock in sockets.items()
                    if camera_sock is sock
                )
                image, header, addr = drain_socket(
                    sock,
                    frames[name],
                    args.max_datagram_size,
                    args.frame_timeout,
                    args.max_packets_per_cycle,
                )
                if image is None:
                    continue
                latest[name] = image
                displayed += 1
                if displayed % 90 == 0:
                    print(
                        "%s frame=%d size=%dx%d from=%s:%d"
                        % (
                            name,
                            int(header.get("frame_id", -1)),
                            int(header.get("width", 0)),
                            int(header.get("height", 0)),
                            addr[0],
                            addr[1],
                        ),
                        flush=True,
                    )

            combined = triangle_layout(
                latest["left"],
                latest["main"],
                latest["right"],
                args.tile_width,
                args.tile_height,
            )
            cv2.imshow("RealSense UDP wrist pair over main", combined)
            key_code = cv2.waitKey(1) & 0xFF
            if key_code in (27, ord("q")):
                break
    finally:
        for sock in sockets.values():
            sock.close()


def main() -> None:
    args = parse_args()
    try:
        if args.three_camera:
            run_three_camera_view(args)
        else:
            run_single_view(args)
    finally:
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


if __name__ == "__main__":
    main()

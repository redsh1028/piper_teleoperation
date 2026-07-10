#!/usr/bin/env python3
"""Display the three Piper camera UDP streams in a local browser."""

import argparse
import json
import select
import socket
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Dict, Tuple
from urllib.parse import urlparse


HEADER_SEPARATOR = b"\n"
MAGIC = "RSIMG1"
CAMERAS = ("main", "left", "right")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Serve Piper UDP camera inputs in a local browser."
    )
    parser.add_argument("--bind", default="0.0.0.0", help="UDP bind address")
    parser.add_argument("--main-port", type=int, default=5020)
    parser.add_argument("--left-port", type=int, default=5021)
    parser.add_argument("--right-port", type=int, default=5022)
    parser.add_argument("--web-bind", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--web-port", type=int, default=8080)
    parser.add_argument("--frame-timeout", type=float, default=1.0)
    parser.add_argument("--recv-buffer", type=int, default=16 * 1024 * 1024)
    return parser.parse_args()


def open_socket(bind_address: str, port: int, recv_buffer: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, recv_buffer)
    sock.bind((bind_address, port))
    sock.setblocking(False)
    return sock


def content_type(encoding: str) -> str:
    if encoding == "png":
        return "image/png"
    return "image/jpeg"


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames = {
            name: {"data": None, "header": None, "received_at": None, "count": 0}
            for name in CAMERAS
        }

    def update(self, camera: str, data: bytes, header: dict) -> None:
        with self._lock:
            state = self._frames[camera]
            state["data"] = data
            state["header"] = header
            state["received_at"] = time.monotonic()
            state["count"] += 1

    def image(self, camera: str):
        with self._lock:
            state = self._frames[camera]
            if state["data"] is None:
                return None
            return state["data"], content_type(state["header"].get("encoding", "jpeg"))

    def status(self) -> dict:
        now = time.monotonic()
        with self._lock:
            result = {}
            for name, state in self._frames.items():
                header = state["header"]
                received_at = state["received_at"]
                result[name] = {
                    "online": received_at is not None and now - received_at < 1.0,
                    "age_ms": None if received_at is None else round((now - received_at) * 1000),
                    "width": None if header is None else header.get("width"),
                    "height": None if header is None else header.get("height"),
                    "frame_id": None if header is None else header.get("frame_id"),
                    "received_frames": state["count"],
                }
            return result


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Piper Camera Inputs</title>
<style>
* { box-sizing: border-box; } body { margin: 0; background: #15191d; color: #edf0f2; font: 14px Arial, sans-serif; }
header { height: 48px; display: flex; align-items: center; padding: 0 16px; border-bottom: 1px solid #3c454c; }
h1 { margin: 0; font-size: 16px; font-weight: 600; } #grid { max-width: 1440px; margin: 16px auto; padding: 0 16px; display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 12px; }
.camera { min-width: 0; background: #22292e; border: 1px solid #3c454c; } .main { grid-column: 1 / -1; max-width: 700px; justify-self: center; width: 100%; }
.bar { height: 34px; display: flex; align-items: center; justify-content: space-between; padding: 0 10px; background: #1b2126; border-bottom: 1px solid #3c454c; }
.name { font-weight: 600; } .status { color: #aeb8bf; font-variant-numeric: tabular-nums; } img { display: block; width: 100%; aspect-ratio: 4 / 3; object-fit: contain; background: #090b0d; }
.main img { aspect-ratio: 16 / 9; } @media (max-width: 700px) { #grid { grid-template-columns: 1fr; } .main { grid-column: auto; } }
</style></head><body><header><h1>Piper Camera Inputs</h1></header><main id="grid">
<section class="camera main"><div class="bar"><span class="name">Main ZED</span><span class="status" id="main-status">Waiting</span></div><img id="main-image" alt="Main camera input"></section>
<section class="camera"><div class="bar"><span class="name">Left Wrist</span><span class="status" id="left-status">Waiting</span></div><img id="left-image" alt="Left wrist camera input"></section>
<section class="camera"><div class="bar"><span class="name">Right Wrist</span><span class="status" id="right-status">Waiting</span></div><img id="right-image" alt="Right wrist camera input"></section>
</main><script>
const cameras = ['main', 'left', 'right'];
function updateImages() { const now = Date.now(); cameras.forEach(name => document.getElementById(name + '-image').src = '/frame/' + name + '?t=' + now); }
async function updateStatus() { const status = await fetch('/status').then(r => r.json()); cameras.forEach(name => { const s = status[name]; document.getElementById(name + '-status').textContent = s.online ? `Live ${s.width}x${s.height} ${s.age_ms}ms` : 'Waiting for UDP'; }); }
setInterval(updateImages, 150); setInterval(updateStatus, 500); updateImages(); updateStatus();
</script></body></html>"""


def make_handler(store: FrameStore):
    class ViewerHandler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:
            path = urlparse(self.path).path
            if path == "/":
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(PAGE.encode("utf-8"))))
                self.end_headers()
                self.wfile.write(PAGE.encode("utf-8"))
                return
            if path == "/status":
                body = json.dumps(store.status()).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path.startswith("/frame/"):
                image = store.image(path.removeprefix("/frame/"))
                if image is None:
                    self.send_error(HTTPStatus.NOT_FOUND, "No frame received")
                    return
                data, mime_type = image
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", mime_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def log_message(self, _format: str, *_args) -> None:
            return

    return ViewerHandler


def receive_frames(sockets: Dict[str, socket.socket], store: FrameStore, timeout: float) -> None:
    pending: Dict[str, Dict[Tuple[int, str], dict]] = {name: {} for name in CAMERAS}
    socket_names = {sock: name for name, sock in sockets.items()}
    while True:
        readable, _, _ = select.select(list(sockets.values()), [], [], 0.1)
        now = time.monotonic()
        for camera in CAMERAS:
            stale = [key for key, frame in pending[camera].items() if now - frame["created_at"] > timeout]
            for key in stale:
                pending[camera].pop(key, None)
        for sock in readable:
            camera = socket_names[sock]
            try:
                packet, _ = sock.recvfrom(65535)
            except BlockingIOError:
                continue
            header_bytes, separator, payload = packet.partition(HEADER_SEPARATOR)
            if not separator:
                continue
            try:
                header = json.loads(header_bytes.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if header.get("magic") != MAGIC:
                continue
            try:
                frame_id = int(header["frame_id"])
                chunk_index = int(header["chunk_index"])
                chunk_count = int(header["chunk_count"])
            except (KeyError, TypeError, ValueError):
                continue
            if chunk_index < 0 or chunk_index >= chunk_count or chunk_count <= 0:
                continue
            key = (frame_id, str(header.get("stream", "unknown")))
            frame = pending[camera].setdefault(
                key,
                {"created_at": now, "chunks": {}, "chunk_count": chunk_count, "header": header},
            )
            if frame["chunk_count"] != chunk_count:
                pending[camera].pop(key, None)
                continue
            frame["chunks"][chunk_index] = payload
            if len(frame["chunks"]) == chunk_count:
                try:
                    image_bytes = b"".join(frame["chunks"][index] for index in range(chunk_count))
                except KeyError:
                    pending[camera].pop(key, None)
                    continue
                store.update(camera, image_bytes, frame["header"])
                pending[camera].pop(key, None)


def main() -> None:
    args = parse_args()
    ports = {"main": args.main_port, "left": args.left_port, "right": args.right_port}
    sockets = {name: open_socket(args.bind, port, args.recv_buffer) for name, port in ports.items()}
    store = FrameStore()
    server = ThreadingHTTPServer((args.web_bind, args.web_port), make_handler(store))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    print("Camera viewer: http://%s:%d" % (args.web_bind, args.web_port), flush=True)
    print("Listening main=%d left=%d right=%d. Press Ctrl+C to quit." % (args.main_port, args.left_port, args.right_port), flush=True)
    try:
        receive_frames(sockets, store, args.frame_timeout)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        for sock in sockets.values():
            sock.close()


if __name__ == "__main__":
    main()

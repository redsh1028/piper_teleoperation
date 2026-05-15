import argparse
import json
import math
import socket
import time
from typing import Any, Dict, List, Optional, Tuple

import openvr


DEFAULT_WSL_IP = "172.25.190.24"
DEFAULT_PORT = 5005


def openvr_vector3_to_list(vector) -> List[float]:
    return [float(vector[0]), float(vector[1]), float(vector[2])]


def openvr_matrix_to_pose(matrix) -> Tuple[List[float], List[float]]:
    """Convert an OpenVR 3x4 pose matrix to position and quaternion."""
    m00, m01, m02, px = [float(matrix[0][i]) for i in range(4)]
    m10, m11, m12, py = [float(matrix[1][i]) for i in range(4)]
    m20, m21, m22, pz = [float(matrix[2][i]) for i in range(4)]

    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0.0:
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

    return [px, py, pz], [qx, qy, qz, qw]


def controller_name(vr_system, device_index: int) -> str:
    role = vr_system.getControllerRoleForTrackedDeviceIndex(device_index)
    if role == openvr.TrackedControllerRole_LeftHand:
        return "left_controller"
    if role == openvr.TrackedControllerRole_RightHand:
        return "right_controller"
    return f"controller_{device_index}"


def controller_role_name(vr_system, device_index: int) -> str:
    role = vr_system.getControllerRoleForTrackedDeviceIndex(device_index)
    if role == openvr.TrackedControllerRole_LeftHand:
        return "left"
    if role == openvr.TrackedControllerRole_RightHand:
        return "right"
    return "unknown"


def controller_axes(state) -> List[float]:
    axes: List[float] = []
    for axis_index in range(5):
        axis = state.rAxis[axis_index]
        axes.extend([float(axis.x), float(axis.y)])
    return axes


def controller_buttons(state) -> List[int]:
    button_names = [
        "k_EButton_SteamVR_Trigger",
        "k_EButton_Grip",
        "k_EButton_ApplicationMenu",
        "k_EButton_SteamVR_Touchpad",
        "k_EButton_A",
    ]

    buttons: List[int] = []
    pressed = int(state.ulButtonPressed)
    touched = int(state.ulButtonTouched)

    for button_name in button_names:
        button_id = getattr(openvr, button_name, None)
        mask = 0 if button_id is None else 1 << int(button_id)
        buttons.append(1 if pressed & mask else 0)

    for button_name in button_names:
        button_id = getattr(openvr, button_name, None)
        mask = 0 if button_id is None else 1 << int(button_id)
        buttons.append(1 if touched & mask else 0)

    return buttons


def controller_input(vr_system, device_index: int) -> Optional[Dict[str, Any]]:
    result = vr_system.getControllerState(device_index)
    if isinstance(result, tuple):
        ok, state = result
        if not ok:
            return None
    else:
        state = result

    return {
        "axes": controller_axes(state),
        "buttons": controller_buttons(state),
        "button_pressed_mask": int(state.ulButtonPressed),
        "button_touched_mask": int(state.ulButtonTouched),
        "input_packet_num": int(state.unPacketNum),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send SteamVR/OpenVR controller data to WSL2 over UDP."
    )
    parser.add_argument("--host", default=DEFAULT_WSL_IP, help="WSL2 receiver IP")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--rate", type=float, default=60.0, help="Send rate in Hz")
    parser.add_argument(
        "--prediction-seconds",
        type=float,
        default=0.0,
        help="OpenVR tracking prediction time",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.rate <= 0.0:
        raise ValueError("--rate must be greater than zero")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sleep_seconds = 1.0 / args.rate

    openvr.init(openvr.VRApplication_Other)
    vr_system = openvr.VRSystem()

    print("OpenVR initialized.")
    print(f"Sending UDP packets to {args.host}:{args.port} at {args.rate:.1f} Hz")

    try:
        while True:
            poses = vr_system.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding,
                args.prediction_seconds,
                openvr.k_unMaxTrackedDeviceCount,
            )

            packets = []
            timestamp = time.time()

            for device_index, pose in enumerate(poses):
                if not pose.bDeviceIsConnected or not pose.bPoseIsValid:
                    continue

                device_class = vr_system.getTrackedDeviceClass(device_index)
                if device_class != openvr.TrackedDeviceClass_Controller:
                    continue

                position, orientation = openvr_matrix_to_pose(
                    pose.mDeviceToAbsoluteTracking
                )
                packet = {
                    "name": controller_name(vr_system, device_index),
                    "role": controller_role_name(vr_system, device_index),
                    "device_index": device_index,
                    "position": position,
                    "orientation": orientation,
                    "velocity": openvr_vector3_to_list(pose.vVelocity),
                    "angular_velocity": openvr_vector3_to_list(pose.vAngularVelocity),
                    "tracking_valid": bool(pose.bPoseIsValid),
                    "tracking_result": int(pose.eTrackingResult),
                    "timestamp": timestamp,
                }

                inputs = controller_input(vr_system, device_index)
                if inputs is not None:
                    packet.update(inputs)

                packets.append(packet)

            if packets:
                message = json.dumps(
                    packets,
                    separators=(",", ":"),
                )
                sock.sendto(message.encode("utf-8"), (args.host, args.port))

            time.sleep(sleep_seconds)

    except KeyboardInterrupt:
        print("Stopped.")
    finally:
        openvr.shutdown()
        sock.close()


if __name__ == "__main__":
    main()

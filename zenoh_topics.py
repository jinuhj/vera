"""Shared Zenoh topic/service names + robot identity for the digital-twin bridge.

These match dexbot_utils' vega_1 (gripper variant) config exactly, so an
unmodified dexcontrol.Robot() resolves the same topics against the sim server.
Topics are auto-namespaced by the ROBOT_NAME env var on both sides.
"""

from __future__ import annotations

import os

# Both the sim server and the dexcontrol client must agree on these.
ROBOT_NAME = os.environ.setdefault("ROBOT_NAME", "vega_sim")
ZENOH_CONFIG_PATH = os.environ.setdefault("ZENOH_CONFIG", "/tmp/zenoh_peer.json5")

# Component -> raw control topic (client publishes JointCmd, server subscribes).
CONTROL_TOPICS: dict[str, str] = {
    "left_arm": "control/arm/left",
    "right_arm": "control/arm/right",
    "head": "control/head",
    "torso": "control/torso",
    "left_hand": "control/gripper/left",
    "right_hand": "control/gripper/right",
}

# Component -> state topic (server publishes JointState, client subscribes).
STATE_TOPICS: dict[str, str] = {
    "left_arm": "state/arm/left",
    "right_arm": "state/arm/right",
    "head": "state/head",
    "torso": "state/torso",
    "left_hand": "state/gripper/left",
    "right_hand": "state/gripper/right",
}

# Mode services the client calls during Robot() init (set_modes / set_mode).
MODE_SERVICES: dict[str, str] = {
    "left_arm": "mode/arm/left",
    "right_arm": "mode/arm/right",
    "head": "mode/head",
    "left_hand": "mode/gripper/left",
    "right_hand": "mode/gripper/right",
}

# Query services (from the vega_1 config `querables`).
HAND_INFO_SERVICE = "info/hand_type"
VERSION_INFO_SERVICE = "info/versions"

# Camera stream topics (match dexbot_utils ZedXCameraConfig / ZedXOneCameraConfig).
# RGB streams -> RGBImageCodec, depth -> DepthImageCodec.
CAMERA_RGB_TOPICS: dict[str, str] = {
    "head_left": "sensors/head_camera/left_rgb",
    "head_right": "sensors/head_camera/right_rgb",
    "left_wrist": "sensors/left_wrist_camera/rgb",
    "right_wrist": "sensors/right_wrist_camera/rgb",
}
CAMERA_DEPTH_TOPIC = "sensors/head_camera/depth"


def ensure_peer_config() -> str:
    """Write a minimal Zenoh peer config file if missing; return its path."""
    path = ZENOH_CONFIG_PATH
    if not os.path.exists(path):
        with open(path, "w") as f:
            f.write("{ mode: \"peer\" }\n")
    return path

"""Zenoh digital-twin server: Isaac Sim impersonating the Vega robot server.

Run this in one terminal; run `run_dexcontrol_client.py` in another. An
UNMODIFIED `dexcontrol.Robot()` in the client connects to THIS process over
Zenoh (the same transport + codecs the real robot uses) and drives the sim.
Swapping to the real robot is then just: don't run this server, point the
client's ZENOH_CONFIG at the real robot's network.

    export ISAACSIM_PATH=$HOME/IsaacSim/_build/linux-aarch64/release
    export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"
    "$ISAACSIM_PATH/python.sh" sim_server.py

What it serves (matching dexbot_utils vega_1 gripper config):
  - service info/hand_type  -> {"left":"DexGripper","right":"DexGripper"} (fatal at client init)
  - service info/versions   -> minimal compatible stub (avoids a 5s client stall)
  - services mode/arm|head|gripper -> {"success": True}
  - subscribers control/*   -> apply raw JointCmd setpoints to the sim
  - publishers state/*      -> JointState {pos, vel} every physics step
"""

# SimulationApp must be constructed before any other isaacsim.* import.
import os as _os

from isaacsim import SimulationApp

# GUI by default (so you can watch); set SIM_HEADLESS=1 for headless runs.
simulation_app = SimulationApp({"headless": _os.environ.get("SIM_HEADLESS", "0") == "1"})

import threading
import time

import numpy as np
from dexcomm.codecs import (
    DepthImageCodec,
    DictDataCodec,
    JointCmdCodec,
    JointModeCodec,
    JointStateCodec,
    RGBImageCodec,
)

import dexcomm
from action_chunks import COMPONENTS, HOME_POSE
from sim_robot import SimVegaRobot
from zenoh_topics import (
    CAMERA_DEPTH_TOPIC,
    CAMERA_RGB_TOPICS,
    CONTROL_TOPICS,
    HAND_INFO_SERVICE,
    MODE_SERVICES,
    ROBOT_NAME,
    STATE_TOPICS,
    VERSION_INFO_SERVICE,
    ensure_peer_config,
)

CONTROL_HZ = 60.0
CAMERA_EVERY_N_STEPS = 4  # publish camera frames ~15 Hz (every 4th 60 Hz step)


def _rgb3(frame) -> np.ndarray:
    """Coerce a camera RGB frame to contiguous HxWx3 uint8."""
    arr = np.asarray(frame)
    if arr.ndim == 3 and arr.shape[2] > 3:
        arr = arr[:, :, :3]
    return np.ascontiguousarray(arr.astype(np.uint8))


def main() -> None:
    cfg_path = ensure_peer_config()
    print(f"[sim_server] ROBOT_NAME={ROBOT_NAME}  ZENOH_CONFIG={cfg_path}", flush=True)

    # Bring up the sim robot (visible, standing). Ramp gently to the standing
    # home pose so the stiff drives don't fling the arms.
    robot = SimVegaRobot()
    robot.move_to_pose(dict(HOME_POSE), steps=120, render=True)
    robot.enable_live_camera_windows()

    # Shared latest-target buffer written by Zenoh callback threads, read by the
    # main sim loop (Isaac Sim is single-threaded, so apply only on main thread).
    latest_targets: dict[str, np.ndarray] = {c: HOME_POSE[c].copy() for c in COMPONENTS}
    lock = threading.Lock()

    # Zenoh node (namespace auto-resolved from ROBOT_NAME).
    dexcomm.get_session(dexcomm.ZenohConfig.from_file(cfg_path))
    node = dexcomm.Node(name="sim_server")

    # --- Services ---
    node.create_service(
        service_name=HAND_INFO_SERVICE,
        handler=lambda _req: {"left": "DexGripper", "right": "DexGripper"},
        response_encoder=DictDataCodec.encode,
    )
    node.create_service(
        service_name=VERSION_INFO_SERVICE,
        handler=lambda _req: {"client": {"minimal_version": "0.0.0"}, "server": {}},
        response_encoder=DictDataCodec.encode,
    )
    for svc in MODE_SERVICES.values():
        node.create_service(
            service_name=svc,
            handler=lambda _req: {"success": True},
            request_decoder=JointModeCodec.decode,
            response_encoder=DictDataCodec.encode,
        )

    # --- Control subscribers: store latest setpoint per component ---
    def make_cb(component: str):
        def _cb(msg):
            pos = msg.get("pos")
            if pos:
                with lock:
                    latest_targets[component] = np.asarray(pos, dtype=float)
        return _cb

    for component, topic in CONTROL_TOPICS.items():
        node.create_subscriber(topic=topic, callback=make_cb(component),
                               decoder=JointCmdCodec.decode)

    # --- State publishers ---
    state_pubs = {
        component: node.create_publisher(topic=topic, encoder=JointStateCodec.encode)
        for component, topic in STATE_TOPICS.items()
    }

    # --- Camera publishers (RGB streams + head depth) ---
    rgb_pubs = {
        key: node.create_publisher(topic=topic, encoder=RGBImageCodec.encode)
        for key, topic in CAMERA_RGB_TOPICS.items()
    }
    depth_pub = node.create_publisher(topic=CAMERA_DEPTH_TOPIC, encoder=DepthImageCodec.encode)

    def publish_cameras() -> None:
        head = robot.sensors.head_camera.get_obs(obs_keys=["left_rgb", "right_rgb", "depth"])
        lw = robot.sensors.left_wrist_camera.get_obs()
        rw = robot.sensors.right_wrist_camera.get_obs()
        rgb_frames = {
            "head_left": head.get("left_rgb", {}).get("data"),
            "head_right": head.get("right_rgb", {}).get("data"),
            "left_wrist": lw.get("data"),
            "right_wrist": rw.get("data"),
        }
        for key, frame in rgb_frames.items():
            if frame is None:
                continue
            img = _rgb3(frame)
            rgb_pubs[key].publish({"data": img, "height": img.shape[0], "width": img.shape[1]})
        depth = head.get("depth", {}).get("data")
        if depth is not None:
            d = np.nan_to_num(np.asarray(depth, dtype=np.float32), nan=0.0,
                              posinf=0.0, neginf=0.0)
            depth_pub.publish({"height": d.shape[0], "width": d.shape[1], "depth_values": d})

    node.spin_in_background()
    print("[sim_server] serving over Zenoh; waiting for dexcontrol client...", flush=True)

    dt = 1.0 / CONTROL_HZ
    step_i = 0
    try:
        while simulation_app.is_running():
            with lock:
                targets = {c: v.copy() for c, v in latest_targets.items()}
            robot.set_joint_targets(targets)
            t0 = time.time()
            robot.step_once(render=True)
            for component, (pos, vel) in robot.read_joint_states().items():
                state_pubs[component].publish(
                    {"pos": [float(x) for x in pos], "vel": [float(x) for x in vel]}
                )
            if step_i % CAMERA_EVERY_N_STEPS == 0:
                publish_cameras()
            step_i += 1
            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        print("[sim_server] interrupted", flush=True)
    finally:
        robot.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()

"""dexcontrol client: drive the sim (or the real Vega) over Zenoh.

This process imports NO Isaac Sim. It uses the real, unmodified
`dexcontrol.Robot()` API — the exact code that runs against a physical Vega.
Against the sim it connects to `sim_server.py` over Zenoh; against the real
robot you'd instead point ZENOH_CONFIG at the robot's network and skip the
sim server. Nothing in the control/sensor logic below changes.

Run (plain Python is fine; no Isaac needed), in a second terminal after
sim_server.py is up:

    export ISAACSIM_PATH=$HOME/IsaacSim/_build/linux-aarch64/release
    "$ISAACSIM_PATH/python.sh" run_dexcontrol_client.py
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
from PIL import Image

# Sets ROBOT_NAME + ZENOH_CONFIG (must match the server) and writes the peer config.
import zenoh_topics
zenoh_topics.ensure_peer_config()

# Sim server has no heartbeat/estop/battery; tell dexcontrol not to require them.
os.environ.setdefault("DEXCONTROL_DISABLE_HEARTBEAT", "1")
os.environ.setdefault("DEXCONTROL_DISABLE_ESTOP_CHECKING", "1")

from dexmate_urdf import robots as _robots
from dexbot_utils.configs.robots.vega_1 import Vega1DGripperConfig
from dexcontrol.robot import Robot

from action_chunks import COMPONENTS, HOME_POSE, generate_random_chunk, parse_joint_limits

URDF_PATH = str(_robots.humanoid.vega_1.vega_1_gripper.urdf)
NUM_CHUNKS = 20
CHUNK_HORIZON = 80
CONTROL_HZ = 60.0
HOLD_SECONDS = 0.5
OUTPUT_DIR = Path(__file__).parent / "output_zenoh"


def _build_config() -> Vega1DGripperConfig:
    """vega_1 gripper config: arms/head/torso/grippers + head & wrist cameras
    enabled; chassis/battery/estop/heartbeat and imu/lidar/ultrasonic disabled
    (the sim server emulates only the joints and the three cameras)."""
    cfg = Vega1DGripperConfig()
    for name in ("chassis", "battery", "estop", "heartbeat"):
        comp = cfg.components.get(name)
        if comp is not None:
            comp.enabled = False
    cameras_only = {}
    for nm in ("head_camera", "left_wrist_camera", "right_wrist_camera"):
        sc = cfg.sensors.get(nm)
        if sc is not None:
            sc.enabled = True
            sc.transport = "zenoh"
            cameras_only[nm] = sc
    cfg.sensors = cameras_only  # drop imu/lidar/ultrasonic
    return cfg


def _save_camera_obs(robot, i: int) -> str:
    """Fetch camera frames over Zenoh via the standard dexcontrol sensor API and
    save PNGs. Returns a short shape summary for logging."""
    head = robot.sensors.head_camera.get_obs(obs_keys=["left_rgb", "right_rgb", "depth"])
    lw = robot.sensors.left_wrist_camera.get_obs()
    rw = robot.sensors.right_wrist_camera.get_obs()

    def _save_rgb(arr, path):
        if arr is None:
            return None
        a = np.asarray(arr)
        Image.fromarray(a[..., :3].astype(np.uint8)).save(path)
        return a.shape

    shapes = {}
    shapes["head_left"] = _save_rgb(head.get("left_rgb"), OUTPUT_DIR / f"chunk{i:03d}_head_left.png")
    shapes["head_right"] = _save_rgb(head.get("right_rgb"), OUTPUT_DIR / f"chunk{i:03d}_head_right.png")
    shapes["left_wrist"] = _save_rgb(lw, OUTPUT_DIR / f"chunk{i:03d}_wrist_left.png")
    shapes["right_wrist"] = _save_rgb(rw, OUTPUT_DIR / f"chunk{i:03d}_wrist_right.png")
    depth = head.get("depth")
    if depth is not None:
        d = np.nan_to_num(np.asarray(depth, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
        d = np.clip(d, 0.0, 5.0)
        du8 = (255.0 * d / max(d.max(), 1e-6)).astype(np.uint8)
        Image.fromarray(du8, mode="L").save(OUTPUT_DIR / f"chunk{i:03d}_head_depth.png")
        shapes["depth"] = d.shape
    return ", ".join(f"{k}={v}" for k, v in shapes.items() if v is not None)


def _ramp(robot: Robot, target: dict[str, np.ndarray], horizon: int = 90) -> None:
    """Smoothly move from the current pose to `target` via execute_trajectory
    (streams raw setpoints -- bypasses the single-step safety guard)."""
    cur = {c: robot.get_joint_pos_dict(c) for c in COMPONENTS}
    start = {c: np.array(list(cur[c].values())) for c in COMPONENTS}
    t = np.linspace(0.0, 1.0, horizon)
    ease = 0.5 * (1.0 - np.cos(np.pi * t))
    traj = {c: start[c][None, :] + ease[:, None] * (target[c] - start[c])[None, :]
            for c in COMPONENTS}
    robot.execute_trajectory(traj, control_hz=CONTROL_HZ)


def _hold(robot: Robot, seconds: float) -> None:
    horizon = max(1, int(seconds * CONTROL_HZ))
    cur = {c: np.array(list(robot.get_joint_pos_dict(c).values())) for c in COMPONENTS}
    traj = {c: np.repeat(cur[c][None, :], horizon, axis=0) for c in COMPONENTS}
    robot.execute_trajectory(traj, control_hz=CONTROL_HZ)


def main() -> None:
    joint_limits = parse_joint_limits(URDF_PATH)
    rng = np.random.default_rng(0)

    OUTPUT_DIR.mkdir(exist_ok=True)
    print("Connecting to robot over Zenoh via dexcontrol.Robot() ...", flush=True)
    robot = Robot(configs=_build_config())
    print("Connected. Robot model:", robot.robot_model, flush=True)

    try:
        print("Ramping to home pose...", flush=True)
        _ramp(robot, HOME_POSE)
        _hold(robot, 1.0)
        print("Home reached. Streaming random action chunks over Zenoh.\n", flush=True)

        for i in range(NUM_CHUNKS):
            current = {c: np.array(list(robot.get_joint_pos_dict(c).values()))
                       for c in COMPONENTS}
            chunk = generate_random_chunk(
                current, joint_limits, horizon=CHUNK_HORIZON, band_frac=0.15,
                rng=rng, anchor_pos={"torso": HOME_POSE["torso"]}, anchor_band_frac=0.02,
            )
            robot.execute_trajectory(chunk, control_hz=CONTROL_HZ)
            _hold(robot, HOLD_SECONDS)

            js = robot.get_joint_pos_dict(COMPONENTS)  # <- joint data received over Zenoh
            cam_summary = _save_camera_obs(robot, i)   # <- camera data received over Zenoh
            print(f"=== chunk {i} === "
                  f"L_arm_j2={js['L_arm_j2']:+.2f} R_arm_j4={js['R_arm_j4']:+.2f} "
                  f"head_j1={js['head_j1']:+.2f} torso_j2={js['torso_j2']:+.2f} "
                  f"L_gripper_j1={js['L_gripper_j1']:+.2f}", flush=True)
            print(f"    cameras over Zenoh: {cam_summary}", flush=True)
    except KeyboardInterrupt:
        print("Interrupted by user", flush=True)
    finally:
        robot.shutdown()


if __name__ == "__main__":
    main()

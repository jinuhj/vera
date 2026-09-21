"""Phase-1 PoC entry point: random action chunks -> simulated Vega, joint/camera
data streamed back in real time.

Run with Isaac Sim's bundled interpreter, e.g.:

    export ISAACSIM_PATH=$HOME/IsaacSim/_build/linux-aarch64/release
    export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"
    "$ISAACSIM_PATH/python.sh" run_demo.py
"""

# SimulationApp must exist before any other isaacsim.* import, anywhere in the
# process — including transitively via sim_robot.py — so this import and
# construction has to come first, ahead of every other project import.
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import time
from pathlib import Path

import numpy as np
from PIL import Image
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.sensors.camera import Camera

from action_chunks import COMPONENTS, HOME_POSE, JOINT_NAMES, generate_random_chunk, parse_joint_limits
from sim_robot import URDF_PATH, SimVegaRobot

NUM_CHUNKS = 20
CHUNK_HORIZON = 80  # steps per chunk; longer = slower, more watchable motion
CONTROL_HZ = 60.0
HOLD_SECONDS = 0.5  # brief settle/pause between chunks so each move is legible
SHOW_LIVE_CAMS = True  # open live viewport windows for the robot's cameras (GUI)
OUTPUT_DIR = Path(__file__).parent / "output"


def _home_chunk(horizon: int = 90) -> dict[str, np.ndarray]:
    t = np.linspace(0.0, 1.0, horizon)
    ease = 0.5 * (1.0 - np.cos(np.pi * t))
    return {
        c: ease[:, None] * HOME_POSE[c][None, :]
        for c in COMPONENTS
    }


def _hold_pose(robot: SimVegaRobot, seconds: float) -> None:
    """Hold the current pose for `seconds`, keeping the sim stepping in real
    time (so the viewport stays live). Sending a constant-position chunk works
    identically on the sim and on real dexcontrol hardware."""
    horizon = max(1, int(seconds * CONTROL_HZ))
    current = {c: robot.get_joint_pos(c) for c in COMPONENTS}
    chunk = {c: np.repeat(current[c][None, :], horizon, axis=0) for c in COMPONENTS}
    robot.execute_trajectory(chunk, control_hz=CONTROL_HZ)


def _make_spectator_camera() -> Camera:
    """A fixed third-person camera framing the whole robot -- demo scaffolding
    so the full body is visible in saved snapshots (the robot's own head/wrist
    cameras are first-person)."""
    cam = Camera(prim_path="/World/SpectatorCam", name="spectator", resolution=(640, 480))
    cam.initialize()
    set_camera_view(eye=np.array([3.2, 2.6, 1.4]), target=np.array([0.15, 0.0, 0.5]),
                    camera_prim_path="/World/SpectatorCam")
    return cam


def _save_snapshot(robot: SimVegaRobot, chunk_idx: int, spectator: Camera) -> None:
    scene = spectator.get_rgb()
    if scene is not None:
        Image.fromarray(np.asarray(scene)[..., :3].astype(np.uint8)).save(
            OUTPUT_DIR / f"chunk{chunk_idx:03d}_scene.png"
        )

    head_obs = robot.sensors.head_camera.get_obs(obs_keys=["left_rgb", "right_rgb", "depth"])
    left_wrist_obs = robot.sensors.left_wrist_camera.get_obs()
    right_wrist_obs = robot.sensors.right_wrist_camera.get_obs()

    Image.fromarray(head_obs["left_rgb"]["data"]).save(OUTPUT_DIR / f"chunk{chunk_idx:03d}_head_left.png")
    Image.fromarray(head_obs["right_rgb"]["data"]).save(OUTPUT_DIR / f"chunk{chunk_idx:03d}_head_right.png")
    Image.fromarray(left_wrist_obs["data"]).save(OUTPUT_DIR / f"chunk{chunk_idx:03d}_wrist_left.png")
    Image.fromarray(right_wrist_obs["data"]).save(OUTPUT_DIR / f"chunk{chunk_idx:03d}_wrist_right.png")

    depth = np.nan_to_num(head_obs["depth"]["data"], nan=0.0, posinf=0.0, neginf=0.0)
    depth = np.clip(depth, 0.0, 5.0)  # meters; clip far background for contrast
    depth_u8 = (255.0 * depth / max(depth.max(), 1e-6)).astype(np.uint8)
    Image.fromarray(depth_u8, mode="L").save(OUTPUT_DIR / f"chunk{chunk_idx:03d}_head_depth.png")


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)
    joint_limits = parse_joint_limits(URDF_PATH)
    rng = np.random.default_rng(0)

    try:
        robot = SimVegaRobot()
        spectator = _make_spectator_camera()
        if SHOW_LIVE_CAMS:
            robot.enable_live_camera_windows()
        print("Moving to home pose...")
        robot.execute_trajectory(_home_chunk(), control_hz=CONTROL_HZ)
        _hold_pose(robot, 1.0)
        print("Home pose reached. Starting random action chunks "
              f"(watch the viewport). Ctrl+C to stop.\n")

        for chunk_idx in range(NUM_CHUNKS):
            current_pos = {c: robot.get_joint_pos(c) for c in COMPONENTS}
            # Keep the torso hovering near its upright home pose so the robot
            # stays standing (without anchoring, the torso random-walks into a
            # forward fold); arms/head/grippers still random-walk freely.
            chunk = generate_random_chunk(
                current_pos, joint_limits, horizon=CHUNK_HORIZON, band_frac=0.15,
                rng=rng, anchor_pos={"torso": HOME_POSE["torso"]}, anchor_band_frac=0.02,
            )

            t0 = time.time()
            robot.execute_trajectory(chunk, control_hz=CONTROL_HZ)
            _hold_pose(robot, HOLD_SECONDS)
            elapsed = time.time() - t0

            joint_state = robot.get_joint_pos_dict(COMPONENTS)
            print(f"=== chunk {chunk_idx} ({elapsed:.2f}s) ===")
            print("  arms/head/torso/grippers moved; sample joints: "
                  f"L_arm_j2={joint_state['L_arm_j2']:+.2f} "
                  f"R_arm_j4={joint_state['R_arm_j4']:+.2f} "
                  f"head_j1={joint_state['head_j1']:+.2f} "
                  f"torso_j2={joint_state['torso_j2']:+.2f} "
                  f"L_gripper_j1={joint_state['L_gripper_j1']:+.2f}")

            _save_snapshot(robot, chunk_idx, spectator)
            print(f"  saved camera snapshots -> {OUTPUT_DIR}/chunk{chunk_idx:03d}_*.png")

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        if "robot" in locals():
            robot.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()

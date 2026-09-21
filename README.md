# Dexmate Vega — Isaac Sim PoC (Zenoh digital twin)

A proof-of-concept that drives a simulated Dexmate **Vega** (vega_1, parallel-gripper
variant) in **NVIDIA Isaac Sim**, and — crucially — lets an **unmodified
`dexcontrol.Robot()` drive the sim over Zenoh**, using the exact same transport,
codecs, and API that run the real robot. Going from sim to the real Vega is a
config change, not a code change.

The PoC sends **random action chunks** to the robot and streams **sensor data
(joint states + head/wrist cameras + depth)** back in real time. In Phase 2 the
random action generator is meant to be swapped for a policy (e.g. GR00T) that
returns the same action-chunk shape.

---

## Contents

| File | Role |
|------|------|
| `action_chunks.py` | Joint layout/limits (from the URDF) + smooth random action-chunk generator. No Isaac/Zenoh deps. |
| `sim_robot.py` | `SimVegaRobot` — Isaac Sim backend (loads the official Vega USD, articulation control, cameras). |
| `run_demo.py` | **Phase 1** — single-process in-process demo (no Zenoh): drives `SimVegaRobot` directly. |
| `sim_server.py` | **Phase 2** — Isaac Sim impersonating the Vega **robot server** over Zenoh. |
| `run_dexcontrol_client.py` | **Phase 2** — real, unmodified `dexcontrol.Robot()` client (same code as the real robot). |
| `view_cameras.py` | **Phase 2** — standalone live camera viewer over Zenoh (browser). |
| `zenoh_topics.py` | Shared topic/service names + `ROBOT_NAME`/`ZENOH_CONFIG`. |
| `requirements.txt` | Python deps (install into Isaac Sim's bundled Python). |
| `assets/` | Cached official Vega USD (auto-downloaded on first run). |
| `output/`, `output_zenoh/` | Saved camera snapshots. |

---

## Prerequisites

- **NVIDIA DGX Spark** (GB10, aarch64) with **Isaac Sim 5.1 built from source** at
  `$HOME/IsaacSim/_build/linux-aarch64/release`. (Isaac Sim's pip package is
  x86_64-only; on DGX Spark it must be built from source.)
- A display attached (for the GUI viewport).

### One-time environment setup

Every terminal that runs this project needs:

```bash
export ISAACSIM_PATH=$HOME/IsaacSim/_build/linux-aarch64/release
export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"   # required on DGX Spark
cd ~/ws_dexmate
```

> Always run scripts with Isaac Sim's bundled interpreter: `"$ISAACSIM_PATH/python.sh" <script>.py`
> (not the system `python`).

### Install dependencies (once)

```bash
"$ISAACSIM_PATH/python.sh" -m pip install -r requirements.txt
```

> **Note:** `requirements.txt` pins `numpy==1.26.4`. `dexcontrol` otherwise upgrades
> numpy to 2.x, which breaks Isaac Sim (numba / USD need numpy < 2.0).

---

## Phase 1 — in-process demo (no Zenoh)

Simplest way to see the robot move. One process: the demo talks to `SimVegaRobot`
directly (in-process Python calls).

```bash
"$ISAACSIM_PATH/python.sh" run_demo.py
```

- The Isaac Sim viewport opens framed on the robot; it homes, then runs 20 random
  action chunks in real time.
- Live camera windows for the robot's cameras open in the Isaac GUI.
- Joint states print each chunk; camera snapshots are saved to `output/`.

---

## Phase 2 — Zenoh digital twin (dexcontrol drives the sim)

This is the main event: the **same `dexcontrol` code that runs the real robot**
drives the sim over Zenoh. Use **three terminals** (each with the environment
setup above).

**Terminal 1 — the sim "robot" server** (wait until it prints `serving over Zenoh`):
```bash
"$ISAACSIM_PATH/python.sh" sim_server.py
```

**Terminal 2 — live camera viewer over Zenoh** (open the printed `http://<host>:8988`
URL in a browser):
```bash
"$ISAACSIM_PATH/python.sh" view_cameras.py
```

**Terminal 3 — the dexcontrol client** (identical to real-robot code):
```bash
"$ISAACSIM_PATH/python.sh" run_dexcontrol_client.py
```

What you'll see: the client connects via `dexcontrol.Robot()`, homes, then streams
random action chunks over Zenoh; the sim (terminal 1) moves; joint + camera data
flow back over Zenoh (printed each chunk, saved to `output_zenoh/`, and shown live
in the browser viewer).

### Useful environment toggles

| Variable | Effect | Default |
|----------|--------|---------|
| `SIM_HEADLESS=1` | Run `sim_server.py` without the Isaac GUI window. | GUI shown |
| `SAVE_COMPOSITE=1` | `view_cameras.py` grabs a few seconds, saves `output_zenoh/live_composite.png`, exits (headless check). | live browser view |
| `WEBAGG_PORT` | Port for the browser camera viewer. | `8988` |
| `ROBOT_NAME` | Zenoh namespace shared by server + client. | `vega_sim` |
| `ZENOH_CONFIG` | Path to the Zenoh config file (auto-written as peer mode). | `/tmp/zenoh_peer.json5` |

---

## Running against the **real Vega**

The client code does not change. On the machine on your robot's network:

1. **Skip** `sim_server.py` — the real robot's firmware is the server.
2. Point `ZENOH_CONFIG` at the robot's Zenoh config and set `ROBOT_NAME` to the
   real robot's name (from your Dexmate setup) so topics resolve.
3. Run `run_dexcontrol_client.py` (and `view_cameras.py` for the live feeds).

Because `run_dexcontrol_client.py` uses the stock `dexcontrol.Robot()` API
(`execute_trajectory`, `get_joint_pos_dict`, `robot.sensors.*.get_obs`), the same
process that drove the sim now drives the real robot.

> The client currently disables chassis/battery/estop/heartbeat and enables only
> the three cameras (see `_build_config()` in `run_dexcontrol_client.py`). Re-enable
> components as needed for the real robot, and remove the
> `DEXCONTROL_DISABLE_HEARTBEAT` / `DEXCONTROL_DISABLE_ESTOP_CHECKING` env defaults
> once real safety systems are in the loop.

---

## Architecture notes

- **Transport:** Phase 2 uses `dexcomm` (the real robot's Zenoh pub/sub + RPC layer)
  with the same codecs (`JointStateCodec`, `JointCmdCodec`, `RGBImageCodec`,
  `DepthImageCodec`) and topic names as the real Vega. `sim_server.py` implements the
  robot-server side (state publishers, control subscribers, `info/hand_type` +
  `mode/*` services); `dexcontrol.Robot()` is the client.
- **Action chunks:** `{component: (T, DOF) ndarray}` streamed at a control rate.
  Components: `left_arm`/`right_arm` (7), `head` (3), `torso` (3),
  `left_hand`/`right_hand` gripper (1). Torso is anchored upright so the robot stays
  standing; arms/head/grippers move.
- **Sim asset:** the official pre-built Vega USD from the `dexmate-urdf` GitHub
  releases (auto-downloaded to `assets/`). A live URDF import is *not* used — Isaac
  Sim 5.1's URDF importer drops this robot's `.glb` visual meshes, so the robot would
  be invisible.

---

## Troubleshooting

- **`can't open file '.../sim_server.py'`** — you're not in the project dir.
  `cd ~/ws_dexmate` first.
- **Isaac Sim won't start / `libgomp` error** — set
  `export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"`.
- **Client hangs then `RobotConnectionError`** — the sim server isn't up yet. Start
  `sim_server.py` first and wait for `serving over Zenoh`.
- **`numpy` / numba / USD import errors after installing** — numpy got upgraded to
  2.x; reinstall the pin: `"$ISAACSIM_PATH/python.sh" -m pip install "numpy==1.26.4"`.
- **`ERROR Failed to query comm_helper.py` at client startup** — benign; an optional
  query with no handler on the sim side. The client connects and runs fine.
- **Camera RGB windows look empty/gray** — the scene has no objects and the head is
  level, so the cameras see floor/sky. The depth panel shows real geometry. Add
  props or tilt the head if you want richer RGB for a demo.
- **Browser viewer shows nothing** — make sure `sim_server.py` is running and you
  opened the exact `http://<host>:8988` URL it printed.

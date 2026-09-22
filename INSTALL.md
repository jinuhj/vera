# First-time installation on a DGX Spark (from scratch)

Foolproof, copy-paste setup for a **fresh NVIDIA DGX Spark** (GB10, ARM64/aarch64)
to run this Vega Isaac Sim + Zenoh digital-twin PoC. Follow top to bottom.

> Tested target: **DGX Spark, DGX OS (Ubuntu 24.04), NVIDIA GB10, aarch64**,
> **Isaac Sim 5.1.0**. Isaac Sim's *pip* package is x86_64-only, so on DGX Spark you
> use one of two methods in **Step 2**: **(A)** the prebuilt multi-arch **container**
> (fastest, no build), or **(B)** a **source build pinned to the exact commit this
> project was verified against**.

> ⚠️ **Do NOT `git clone` the latest Isaac Sim `main` and run `build.sh`.** Current
> `main` uses a `pixi`-based build that fails on DGX Spark (`pixi.lock`/`pixi.toml`
> mismatch, unresolved extension deps). Step 2B pins the known-good pre-`pixi`
> commit (`aa503a9`, Isaac Sim 5.1.0-rc.19) that builds cleanly here.

Estimated time: **~10 min (container)** or **~40–60 min (source build)**.
Disk needed: **~60 GB free**.

Throughout, we use these variables — adjust only if you want different locations:

```bash
export ISAAC_ROOT="$HOME/IsaacSim"
export ISAACSIM_PATH="$ISAAC_ROOT/_build/linux-aarch64/release"
export PROJECT_DIR="$HOME/ws_dexmate"
```

---

## Step 0 — Sanity checks

```bash
uname -m            # expect: aarch64
nvidia-smi          # expect: GB10, a driver version, no error
free -h             # expect: ~120Gi total
df -h "$HOME"       # expect: >= 60G available
```

If `uname -m` is not `aarch64`, this guide (and the source build) does not apply —
use the standard Isaac Sim pip install instead.

---

## Step 1 — System packages

```bash
sudo apt update
sudo apt install -y gcc-11 g++-11 git git-lfs build-essential cmake \
                    libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev \
                    libxi-dev libgl1-mesa-dev ffmpeg

# Make gcc/g++ 11 the default compiler (required by the Isaac Sim build)
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 200
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 200
git lfs install

gcc --version        # expect: 11.x
g++ --version        # expect: 11.x
git lfs version      # expect: prints a version
```

---

## Step 2 — Install Isaac Sim (choose ONE method)

### Method A — Prebuilt container (recommended, no build)

Fastest and avoids the source-build breakage entirely. The Isaac Sim container is
multi-arch and runs on DGX Spark (aarch64). Requires **Docker** + the **NVIDIA
Container Toolkit** installed (`sudo apt install -y nvidia-container-toolkit` then
`sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker`).

```bash
docker pull nvcr.io/nvidia/isaac-sim:5.1.0
```

Run the container with this project mounted and host networking (needed so Zenoh
reaches between the sim server, viewer, and client), then work *inside* it:

```bash
xhost +local:                              # allow the container to use your display
docker run --name vega-sim -it --gpus all --rm --network=host \
  -e "ACCEPT_EULA=Y" -e "PRIVACY_CONSENT=Y" \
  -e DISPLAY -v $HOME/.Xauthority:/root/.Xauthority \
  -v "$HOME/ws_dexmate":/root/ws_dexmate \
  --entrypoint bash nvcr.io/nvidia/isaac-sim:5.1.0
```

Inside the container, Isaac Sim's Python is `/isaac-sim/python.sh`. Set the project
env to match the rest of this guide, then continue at **Step 4**:

```bash
export ISAACSIM_PATH=/isaac-sim
export PROJECT_DIR=/root/ws_dexmate
cd "$PROJECT_DIR"
```

Then run the smoke test under **"Verify Isaac Sim actually starts"** below, and
continue at **Step 4**. (Step 3 — copying the project — is already covered by the
`-v $HOME/ws_dexmate:/root/ws_dexmate` mount above.)

> Notes: livestreaming is not supported on aarch64 (we don't use it). Because the
> container is started with `--rm`, `pip` installs (Step 4) live only for that
> session — for a permanent image, bake `requirements.txt` into a small
> `Dockerfile` (`FROM nvcr.io/nvidia/isaac-sim:5.1.0`). Ask if you want that added.
> This container path follows NVIDIA's documented commands; it hasn't been run
> end-to-end on this project's box — if anything differs, Method B is the verified one.

### Method B — Source build, pinned to the verified commit

This reproduces exactly what the reference machine runs (Isaac Sim 5.1.0-rc.19).
**Use a full clone and check out the pinned commit** — do not shallow-clone latest.

```bash
cd "$HOME"
git clone --recursive https://github.com/isaac-sim/IsaacSim "$ISAAC_ROOT"
cd "$ISAAC_ROOT"
git checkout aa503a9                        # known-good, pre-pixi (5.1.0-rc.19)
git submodule update --init --recursive
git lfs install
git lfs pull

# Build (accept the EULA when prompted). Takes ~10–15 min.
./build.sh
```

Success looks like: `BUILD (RELEASE) SUCCEEDED`.

Set (and verify) the Isaac Sim paths (Method B only — Method A already set
`ISAACSIM_PATH=/isaac-sim` inside the container):

```bash
export ISAACSIM_PATH="$ISAAC_ROOT/_build/linux-aarch64/release"
ls "$ISAACSIM_PATH/python.sh"     # must exist
```

### Verify Isaac Sim actually starts (either method)

Headless smoke test — the `LD_PRELOAD` is required on DGX Spark:

```bash
export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"
"$ISAACSIM_PATH/python.sh" -c "
from isaacsim import SimulationApp
app = SimulationApp({'headless': True})
print('ISAAC SIM OK')
app.close()
"
```

Expect `ISAAC SIM OK` near the end (lots of log lines before it are normal).

> If it crashes with a `libgomp` error, you forgot the `LD_PRELOAD` export above.

---

## Step 3 — Get the project onto this machine

The project is **not** committed to a shared git repo, so copy it from your other
DGX Spark (or a USB drive). The `assets/` and `output*/` folders regenerate
automatically — you only need the source files.

**Option A — copy from your other machine over the network** (run on the SOURCE machine):

```bash
# run this ON the machine that currently has the project (~/ws_dexmate):
rsync -av --exclude 'assets/' --exclude 'output*/' --exclude '__pycache__/' \
    ~/ws_dexmate/  <office-user>@<office-host>:~/ws_dexmate/
```

**Option B — USB / manual copy:** copy the whole `ws_dexmate` folder; it's fine to
include or drop `assets/` (it re-downloads on first run).

Confirm on the office machine:

```bash
export PROJECT_DIR="$HOME/ws_dexmate"
ls "$PROJECT_DIR"    # expect: sim_server.py run_dexcontrol_client.py view_cameras.py
                     #         sim_robot.py action_chunks.py zenoh_topics.py
                     #         run_demo.py requirements.txt README.md
```

---

## Step 4 — Install Python dependencies

Install into **Isaac Sim's bundled Python** (never the system Python):

```bash
cd "$PROJECT_DIR"
"$ISAACSIM_PATH/python.sh" -m pip install -r requirements.txt
```

This installs `dexmate-urdf`, `dexbot-utils`, `eclipse-zenoh`, `dexcomm`,
`dexcontrol`, and pins `numpy==1.26.4`.

> **Critical:** if you ever see numba / USD / numpy-2.x errors later, numpy got
> upgraded. Re-pin it:
> `"$ISAACSIM_PATH/python.sh" -m pip install "numpy==1.26.4"`

Verify the imports and that numpy stayed pinned:

```bash
"$ISAACSIM_PATH/python.sh" -c "
import numpy, dexcomm, dexcontrol, dexbot_utils, dexmate_urdf, zenoh
print('numpy', numpy.__version__)   # must be 1.26.4
print('dexcomm', dexcomm.__version__)
print('all imports OK')
"
```

---

## Step 5 — Make the environment permanent (recommended)

> **Method B (source build) only.** Container (Method A) users instead re-export
> `ISAACSIM_PATH=/isaac-sim` each `docker run` session (or bake it into a Dockerfile).

So every new terminal is ready without re-exporting:

```bash
cat >> "$HOME/.bashrc" <<'EOF'

# --- Vega Isaac Sim PoC ---
export ISAACSIM_PATH="$HOME/IsaacSim/_build/linux-aarch64/release"
export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"
alias isaacpy='"$ISAACSIM_PATH/python.sh"'
EOF
source "$HOME/.bashrc"
```

(After this you can run e.g. `isaacpy sim_server.py`.)

---

## Step 6 — First run / verification

### 6a. Quick headless end-to-end check (no GUI, ~1 min)

This proves Isaac Sim + Zenoh + dexcontrol all work together. Two terminals (or run
the server in the background):

```bash
cd "$PROJECT_DIR"

# Terminal 1 — sim server, headless. Wait for: "serving over Zenoh"
SIM_HEADLESS=1 "$ISAACSIM_PATH/python.sh" sim_server.py

# Terminal 2 — dexcontrol client (real robot API, over Zenoh)
"$ISAACSIM_PATH/python.sh" run_dexcontrol_client.py
```

Success = the client prints `Connected. Robot model: vega_1`, then `=== chunk N ===`
lines with changing joint values and `cameras over Zenoh: head_left=(480, 640, 3) ...`.
Camera snapshots land in `output_zenoh/`. Stop the server with `Ctrl+C`.

> First run downloads the official Vega USD (~13 MB) into `assets/` — needs internet.

### 6b. Full demo with the GUI + live cameras (3 terminals)

```bash
cd "$PROJECT_DIR"    # in every terminal

# Terminal 1 — sim "robot" server (GUI viewport). Wait for "serving over Zenoh".
"$ISAACSIM_PATH/python.sh" sim_server.py

# Terminal 2 — live camera viewer; open the printed http://<host>:8988 in a browser
"$ISAACSIM_PATH/python.sh" view_cameras.py

# Terminal 3 — dexcontrol client (same code that runs the real robot)
"$ISAACSIM_PATH/python.sh" run_dexcontrol_client.py
```

You should see the Vega standing and moving in the Isaac viewport, live camera
feeds in the browser, and joint/camera data printing in terminal 3.

### 6c. (Optional) Phase-1 in-process demo (no Zenoh)

```bash
"$ISAACSIM_PATH/python.sh" run_demo.py
```

---

## Step 7 — Point at the REAL Vega (when ready)

The client code is unchanged. On a machine on the robot's network:

1. Do **not** run `sim_server.py` (the robot firmware is the server).
2. Set `ZENOH_CONFIG` to the robot's Zenoh config file and `ROBOT_NAME` to the real
   robot's name (from your Dexmate setup):
   ```bash
   export ZENOH_CONFIG=~/.dexmate/comm/zenoh/<your_config>.dzcfg
   export ROBOT_NAME=<your_real_robot_name>
   ```
3. Run `run_dexcontrol_client.py` (and `view_cameras.py` for feeds).

> Also review `_build_config()` in `run_dexcontrol_client.py` and the
> `DEXCONTROL_DISABLE_HEARTBEAT` / `DEXCONTROL_DISABLE_ESTOP_CHECKING` defaults —
> those are sim conveniences; re-enable heartbeat/estop for real hardware.

See `README.md` for day-to-day usage, environment toggles, and architecture notes.

---

## Troubleshooting (first-time install)

| Symptom | Fix |
|---|---|
| Build fails with **`pixi.lock` / `pixi.toml` not matching** or unresolved extension deps | You cloned latest `main` (now a broken `pixi` build on DGX Spark). Use **Method A (container)**, or **Method B** which pins the pre-`pixi` commit `aa503a9`. Do not shallow-clone latest. |
| `uname -m` isn't `aarch64` | This guide is DGX-Spark-specific; on x86_64 use the standard Isaac Sim pip install or container. |
| Isaac Sim build fails (Method B) | Ensure `gcc --version` / `g++ --version` are **11.x** (Step 1 alternatives), and you're on commit `aa503a9`. Remove `.cache` in `$ISAAC_ROOT` and rebuild. |
| `git lfs pull` fails / models missing | Network issue; re-run `git lfs pull` in `$ISAAC_ROOT`. |
| Isaac Sim won't start, `libgomp` error | `export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"`. |
| `python.sh` not found | `export ISAACSIM_PATH="$HOME/IsaacSim/_build/linux-aarch64/release"` (matches your build location). |
| numba / USD / numpy 2.x import errors | `"$ISAACSIM_PATH/python.sh" -m pip install "numpy==1.26.4"`. |
| `can't open file '.../sim_server.py'` | `cd "$PROJECT_DIR"` first. |
| Client `RobotConnectionError` | Start `sim_server.py` first; wait for `serving over Zenoh`. |
| Browser camera viewer blank | Ensure `sim_server.py` is running; open the exact `http://<host>:8988` URL it printed. |
| `ERROR Failed to query comm_helper.py` at client start | Benign; optional query with no sim-side handler. It connects and runs fine. |

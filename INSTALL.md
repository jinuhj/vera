# First-time installation on a DGX Spark (from scratch)

Foolproof, copy-paste setup for a **fresh NVIDIA DGX Spark** (GB10, ARM64/aarch64)
to run this Vega Isaac Sim + Zenoh digital-twin PoC. Follow top to bottom.

> Tested target: **DGX Spark, DGX OS (Ubuntu 24.04), NVIDIA GB10, aarch64**,
> **Isaac Sim 5.1.0** built from source. Isaac Sim's pip package is x86_64-only, so
> on DGX Spark it **must be built from source** (done in Step 2).

Estimated time: **~40–60 min** (most of it the one-time Isaac Sim build ~15 min +
downloads). Disk needed: **~60 GB free**.

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

## Step 2 — Build Isaac Sim from source

```bash
cd "$HOME"
git clone --depth=1 --recursive https://github.com/isaac-sim/IsaacSim "$ISAAC_ROOT"
cd "$ISAAC_ROOT"
git lfs install
git lfs pull

# Build (accept the EULA when prompted). Takes ~10–15 min.
./build.sh
```

Success looks like: `BUILD (RELEASE) SUCCEEDED`.

Set (and verify) the Isaac Sim paths:

```bash
export ISAACSIM_PATH="$ISAAC_ROOT/_build/linux-aarch64/release"
ls "$ISAACSIM_PATH/python.sh"     # must exist
```

**Verify Isaac Sim actually starts** (headless smoke test — the `LD_PRELOAD` is
required on DGX Spark):

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
| `uname -m` isn't `aarch64` | This source-build guide is DGX-Spark-specific; use standard Isaac Sim pip install. |
| Isaac Sim build fails | Ensure `gcc --version` / `g++ --version` are **11.x** (Step 1 alternatives). Remove `.cache` in `$ISAAC_ROOT` and rebuild. |
| `git lfs pull` fails / models missing | Network issue; re-run `git lfs pull` in `$ISAAC_ROOT`. |
| Isaac Sim won't start, `libgomp` error | `export LD_PRELOAD="$LD_PRELOAD:/lib/aarch64-linux-gnu/libgomp.so.1"`. |
| `python.sh` not found | `export ISAACSIM_PATH="$HOME/IsaacSim/_build/linux-aarch64/release"` (matches your build location). |
| numba / USD / numpy 2.x import errors | `"$ISAACSIM_PATH/python.sh" -m pip install "numpy==1.26.4"`. |
| `can't open file '.../sim_server.py'` | `cd "$PROJECT_DIR"` first. |
| Client `RobotConnectionError` | Start `sim_server.py` first; wait for `serving over Zenoh`. |
| Browser camera viewer blank | Ensure `sim_server.py` is running; open the exact `http://<host>:8988` URL it printed. |
| `ERROR Failed to query comm_helper.py` at client start | Benign; optional query with no sim-side handler. It connects and runs fine. |

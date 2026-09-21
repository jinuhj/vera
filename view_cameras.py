"""Standalone live camera viewer over Zenoh.

Subscribes to the Vega camera streams (head L/R RGB + depth, both wrist RGB)
directly over Zenoh via dexcomm -- completely decoupled from the control client.
Run it in a third terminal while sim_server.py is up; it demonstrates the
pub/sub nature of the transport (any number of subscribers) and shows exactly
what the robot streams. The same viewer works against the real robot.

    export ISAACSIM_PATH=$HOME/IsaacSim/_build/linux-aarch64/release
    "$ISAACSIM_PATH/python.sh" view_cameras.py
    # then open the printed http://<host>:8988 URL in a browser

Set SAVE_COMPOSITE=1 to instead grab a few seconds of frames, save one
composite PNG to output_zenoh/live_composite.png, and exit (headless check).
"""

from __future__ import annotations

import os
import time

import numpy as np

# Sets ROBOT_NAME + ZENOH_CONFIG (must match the server).
import zenoh_topics
zenoh_topics.ensure_peer_config()

import matplotlib
_SAVE = os.environ.get("SAVE_COMPOSITE", "0") == "1"
matplotlib.use("Agg" if _SAVE else "webagg")
if not _SAVE:
    matplotlib.rcParams["webagg.open_in_browser"] = False
    matplotlib.rcParams["webagg.port"] = int(os.environ.get("WEBAGG_PORT", "8988"))

import matplotlib.pyplot as plt

import dexcomm
from dexcomm.codecs import DepthImageCodec, RGBImageCodec
from zenoh_topics import CAMERA_DEPTH_TOPIC, CAMERA_RGB_TOPICS

PANELS = ["head_left", "head_right", "left_wrist", "right_wrist", "depth"]
_latest: dict[str, np.ndarray] = {}


def _store(key):
    def _cb(msg):
        _latest[key] = msg.get("data") if isinstance(msg, dict) else msg
    return _cb


def _depth_to_rgb(d: np.ndarray) -> np.ndarray:
    d = np.nan_to_num(np.asarray(d, dtype=np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    d = np.clip(d, 0.0, 5.0)
    norm = d / max(float(d.max()), 1e-6)
    return (plt.get_cmap("viridis")(norm)[:, :, :3] * 255).astype(np.uint8)


def _panel_image(key: str) -> np.ndarray:
    frame = _latest.get(key)
    if frame is None:
        return np.full((480, 640, 3), 40, np.uint8)
    if key == "depth":
        return _depth_to_rgb(frame)
    return np.asarray(frame)[..., :3].astype(np.uint8)


def main() -> None:
    dexcomm.get_session(dexcomm.ZenohConfig.from_file(zenoh_topics.ZENOH_CONFIG_PATH))
    node = dexcomm.Node(name="camera_viewer")
    for key, topic in CAMERA_RGB_TOPICS.items():
        node.create_subscriber(topic=topic, callback=_store(key), decoder=RGBImageCodec.decode)
    node.create_subscriber(topic=CAMERA_DEPTH_TOPIC, callback=_store("depth"),
                           decoder=DepthImageCodec.decode)
    node.spin_in_background()
    print(f"[view_cameras] subscribed over Zenoh (ROBOT_NAME={zenoh_topics.ROBOT_NAME})",
          flush=True)

    titles = ["head L (rgb)", "head R (rgb)", "left wrist (rgb)", "right wrist (rgb)",
              "head depth (m)"]
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    fig.suptitle("Vega cameras over Zenoh", fontsize=15)
    axes = axes.ravel()
    ims = []
    for ax, key, title in zip(axes, PANELS, titles):
        ims.append(ax.imshow(_panel_image(key)))
        ax.set_title(title)
        ax.axis("off")
    axes[5].axis("off")

    if _SAVE:
        deadline = time.time() + 6.0
        while time.time() < deadline:
            time.sleep(0.2)
        for im, key in zip(ims, PANELS):
            im.set_array(_panel_image(key))
        os.makedirs("output_zenoh", exist_ok=True)
        out = "output_zenoh/live_composite.png"
        fig.savefig(out, dpi=80)
        got = {k: (None if _latest.get(k) is None else np.asarray(_latest[k]).shape)
               for k in PANELS}
        print(f"[view_cameras] received: {got}", flush=True)
        print(f"[view_cameras] saved {out}", flush=True)
        return

    import matplotlib.animation as animation

    def _update(_frame):
        for im, key in zip(ims, PANELS):
            im.set_array(_panel_image(key))
        return ims

    _anim = animation.FuncAnimation(fig, _update, interval=100, blit=False,
                                    cache_frame_data=False)
    print("[view_cameras] open the WebAgg URL printed below in your browser.", flush=True)
    plt.show()


if __name__ == "__main__":
    main()

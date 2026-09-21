"""Joint layout, limits, and random action-chunk generation for the Vega gripper variant.

This module has no Isaac Sim dependency (pure numpy + stdlib XML parsing) so it
can be imported and unit-tested outside the simulator, and reused unchanged
for the real-robot backend in phase 2.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

# Joint names per component, matching dexbot_utils.configs.components.vega_1
# exactly so this chunk format lines up with real dexcontrol joint order.
JOINT_NAMES: dict[str, list[str]] = {
    "left_arm": [f"L_arm_j{i}" for i in range(1, 8)],
    "right_arm": [f"R_arm_j{i}" for i in range(1, 8)],
    "head": ["head_j1", "head_j2", "head_j3"],
    "torso": ["torso_j1", "torso_j2", "torso_j3"],
    "left_hand": ["L_gripper_j1"],
    "right_hand": ["R_gripper_j1"],
}

COMPONENTS: list[str] = list(JOINT_NAMES.keys())

# Safe seed pose to move to before random motion starts (radians).
# Arms use dexbot_utils' "folded" pose_pool entry (compact, upright, arms tucked
# in front -- a clean standing posture); head/torso "home" (torso [0,0,0] is
# upright); grippers open. The right arm mirrors the left per dexbot_utils.
HOME_POSE: dict[str, np.ndarray] = {
    "left_arm": np.array([1.57079, 0.0, 0.0, -3.1, 0.0, 0.0, -0.69813]),
    "right_arm": np.array([-1.57079, 0.0, 0.0, -3.1, 0.0, 0.0, 0.69813]),
    "head": np.zeros(3),
    "torso": np.zeros(3),
    "left_hand": np.array([0.7854]),
    "right_hand": np.array([0.7854]),
}


def parse_joint_limits(urdf_path: str | Path) -> dict[str, tuple[float, float]]:
    """Parse (lower, upper) position limits for every joint in a URDF file."""
    root = ET.parse(str(urdf_path)).getroot()
    limits: dict[str, tuple[float, float]] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name is None or limit is None:
            continue
        lower = limit.get("lower")
        upper = limit.get("upper")
        if lower is None or upper is None:
            continue
        limits[name] = (float(lower), float(upper))
    return limits


def _ease_in_out(t: np.ndarray) -> np.ndarray:
    """Cosine ease: smooth 0->1 with zero velocity at both ends."""
    return 0.5 * (1.0 - np.cos(np.pi * t))


def generate_random_chunk(
    current_pos: dict[str, np.ndarray],
    joint_limits: dict[str, tuple[float, float]],
    horizon: int = 50,
    band_frac: float = 0.2,
    rng: np.random.Generator | None = None,
    anchor_pos: dict[str, np.ndarray] | None = None,
    anchor_band_frac: float | None = None,
) -> dict[str, np.ndarray]:
    """Generate a smooth random action chunk for every component in JOINT_NAMES.

    For each joint, samples one random target within a band of its full range
    around a *sampling center* (clipped to the joint's hard limits), then eases
    from the current position to that target over `horizon` steps. By default the
    sampling center is the current position (a random walk). Components listed in
    `anchor_pos` instead sample around a fixed anchor pose with `anchor_band_frac`
    -- use this to keep a component (e.g. the torso) hovering near an upright
    reference instead of drifting away over many chunks.

    Args:
        current_pos: component -> current joint positions, shape (dof,).
        joint_limits: joint_name -> (lower, upper), from `parse_joint_limits`.
        horizon: number of timesteps T in the returned chunk.
        band_frac: fraction of each joint's full range used as the max step
            away from the sampling center, keeping motion visible but sane.
        rng: optional numpy random Generator for reproducibility.
        anchor_pos: optional component -> fixed reference pose. Listed components
            sample around this pose instead of their current position, so they
            stay near it rather than random-walking.
        anchor_band_frac: band fraction for anchored components (defaults to
            `band_frac` if None). Use a small value to keep them near the anchor.

    Returns:
        component -> ndarray of shape (horizon, dof).
    """
    rng = rng or np.random.default_rng()
    anchor_pos = anchor_pos or {}
    t = np.linspace(0.0, 1.0, horizon)
    ease = _ease_in_out(t)  # (horizon,)

    chunk: dict[str, np.ndarray] = {}
    for component, joint_names in JOINT_NAMES.items():
        start = np.asarray(current_pos[component], dtype=np.float64)
        lowers = np.array([joint_limits[n][0] for n in joint_names])
        uppers = np.array([joint_limits[n][1] for n in joint_names])

        if component in anchor_pos:
            center = np.asarray(anchor_pos[component], dtype=np.float64)
            bf = anchor_band_frac if anchor_band_frac is not None else band_frac
        else:
            center = start
            bf = band_frac
        band = bf * (uppers - lowers)

        low_bound = np.maximum(lowers, center - band)
        high_bound = np.minimum(uppers, center + band)
        target = rng.uniform(low_bound, high_bound)

        # (horizon, dof) interpolation from current position to target.
        chunk[component] = start[None, :] + ease[:, None] * (target - start)[None, :]

    return chunk

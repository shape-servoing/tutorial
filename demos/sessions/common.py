"""
Shared pieces of the demo sessions.
===================================

`Session` is the state of one open demo page: its public `act_<name>`
methods are the actions the page can request (docs/js/sim/sessions/common.js
is the JavaScript port that the pages run). The helpers keep
the robot inside a safe workspace and package scenes for drawing.
"""
from __future__ import annotations

import numpy as np

from shape_servoing import Plant, Q_ARCH, target_from_grasp

# Workspace of the grippers: x and y in simulation units, theta in radians.
Q_LOWER = np.array([-4.2, -1.2, -1.45, 0.6, -1.2, -1.45])
Q_UPPER = np.array([-0.6, 3.8, 1.45, 4.2, 3.8, 1.45])
MIN_SEPARATION, MAX_SEPARATION = 1.2, 7.2      # at most 20 % stretch


class Session:
    """One open demo page. Subclasses implement `state` and `act_*` actions."""

    def dispatch(self, action: str, payload: dict):
        method = getattr(self, f"act_{action}", None)
        if method is None:
            raise KeyError(f"unknown action: {action}")
        return method(**payload)

    def state(self) -> dict:
        raise NotImplementedError


def admissible(q) -> np.ndarray:
    """Clip a grasp to the workspace and to the allowed gripper separation."""
    q = np.clip(np.asarray(q, float), Q_LOWER, Q_UPPER)
    left, right = q[:2], q[3:5]
    gap = np.linalg.norm(right - left)
    if gap > MAX_SEPARATION or gap < MIN_SEPARATION:
        middle, unit = 0.5 * (left + right), (right - left) / max(gap, 1e-9)
        half = 0.5 * np.clip(gap, MIN_SEPARATION, MAX_SEPARATION)
        q[:2], q[3:5] = middle - half * unit, middle + half * unit
    return q


def scene(plant: Plant) -> dict:
    """What the scene canvas draws for the current object."""
    return {"q": plant.q, "nodes": plant.nodes, "points": plant.points}


def target_scene(nodes, points) -> dict:
    return {"nodes": nodes, "points": points}


def random_target(rng, plant: Plant, min_distance: float = 1.2):
    """A feasible target recorded at a random grasp, away from the current shape."""
    for _ in range(200):
        separation = rng.uniform(3.4, 5.6)
        middle = np.array([rng.uniform(-0.7, 0.7), rng.uniform(0.4, 2.0)])
        tilt = rng.uniform(-0.25, 0.25)
        axis = np.array([np.cos(tilt), np.sin(tilt)])
        left, right = middle - 0.5 * separation * axis, middle + 0.5 * separation * axis
        bend = rng.uniform(-1.2, 1.3)
        q = admissible([*left, tilt + bend + rng.uniform(-0.3, 0.3),
                        *right, tilt - bend + rng.uniform(-0.3, 0.3)])
        nodes, points, s_star = target_from_grasp(q, plant.rod, plant.feature_map, plant.n_points)
        if np.linalg.norm(points.ravel() - plant.points.ravel()) >= min_distance:
            return q, nodes, points, s_star
    return (Q_ARCH.copy(), *target_from_grasp(Q_ARCH, plant.rod, plant.feature_map, plant.n_points))

"""
Shape features: the feature map s = f(y).
=========================================

Perception tracks P material points p_1..p_P on the rod. A feature map turns
them into the feature vector s that the controller regulates. The three maps
compared in the tutorial discard progressively more of the object's pose:

    point coordinates  s = [p_1, ..., p_P]          n = 2P      extrinsic
    edge vectors       d_i = p_{i+1} - p_i          n = 2(P-1)  translation-invariant
    turning angles     kappa_i = angle(d_i, d_{i+1}) n = P-2     rigid-motion-invariant
"""
from __future__ import annotations

import numpy as np


def point_coordinates(points: np.ndarray) -> np.ndarray:
    """s = [x_1, y_1, ..., x_P, y_P]."""
    return np.asarray(points, float).reshape(-1)


def edge_vectors(points: np.ndarray) -> np.ndarray:
    """Stacked d_i = p_{i+1} - p_i, for i = 1..P-1."""
    return np.diff(np.asarray(points, float), axis=0).reshape(-1)


def turning_angles(points: np.ndarray) -> np.ndarray:
    """Signed angles kappa_i between consecutive edges, wrapped to (-pi, pi]."""
    d = np.diff(np.asarray(points, float), axis=0)
    a, b = d[:-1], d[1:]
    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    return np.arctan2(cross, np.sum(a * b, axis=1))


FEATURE_MAPS = {
    "points": point_coordinates,
    "edges": edge_vectors,
    "angles": turning_angles,
}


def feature_vector(points: np.ndarray, feature_map: str = "points") -> np.ndarray:
    """Feature vector s = f(p) for the named feature map."""
    return FEATURE_MAPS[feature_map](points)


def feature_map_jacobian(points: np.ndarray, feature_map: str = "points") -> np.ndarray:
    """Derivative ds/ds_p of a feature map with respect to the point coordinates s_p.

    The map is known, so this derivative is exact. A rigid motion of the points
    lies in its null space for turning angles, and a translation for edge vectors.
    """
    points = np.asarray(points, float)
    P = len(points)
    if feature_map == "points":
        return np.eye(2 * P)
    if feature_map == "edges":
        return np.kron(np.eye(P - 1, P, 1) - np.eye(P - 1, P), np.eye(2))
    # kappa_i = angle(d_{i+1}) - angle(d_i), with d(angle(d))/dd = perp(d) / |d|^2
    d = np.diff(points, axis=0)
    g = np.column_stack([-d[:, 1], d[:, 0]]) / np.sum(d * d, axis=1)[:, None]
    D = np.zeros((P - 2, 2 * P))
    for i in range(P - 2):
        D[i, 2 * i:2 * i + 2] = g[i]
        D[i, 2 * i + 2:2 * i + 4] = -g[i] - g[i + 1]
        D[i, 2 * i + 4:2 * i + 6] = g[i + 1]
    return D


def feature_dimension(n_points: int, feature_map: str) -> int:
    """Dimension n of the feature vector computed from P tracked points."""
    return {"points": 2 * n_points, "edges": 2 * (n_points - 1),
            "angles": n_points - 2}[feature_map]


def shape_error(s_star: np.ndarray, s: np.ndarray) -> np.ndarray:
    """Shape error e = s* - s."""
    return np.asarray(s_star, float) - np.asarray(s, float)


def rigid_motion(points: np.ndarray, angle: float = 0.0, translation=(0.0, 0.0)) -> np.ndarray:
    """Rotate the points about their centroid by `angle` [rad], then translate."""
    points = np.asarray(points, float)
    centroid = points.mean(axis=0)
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    return (points - centroid) @ rotation.T + centroid + np.asarray(translation, float)

"""
The running example: two robot arms deform an elastic rod seen by a camera.
===========================================================================

Both grippers move in the image plane, so the robot coordinates are

    q = [x_L, y_L, theta_L, x_R, y_R, theta_R]      (m = 6)

and perception tracks P = 5 material points along the rod (n = 10 for point
coordinates). Lengths are in simulation units (rest length 6), angles in
radians, and the control period is Delta t = 1.

`Plant` groups the robot, the rod, the camera, and the feature map. The
controller only commands q and measures s.
"""
from __future__ import annotations

import numpy as np

from .features import feature_vector
from .rod import Rod

DOF_NAMES = ("x_L", "y_L", "theta_L", "x_R", "y_R", "theta_R")
N_POINTS = 5

# Grasps used throughout the tutorial figures.
Q_NOMINAL = np.array([-3.0, 1.00, 0.00, 3.0, 1.00, 0.00])     # initial configuration q_0
Q_ARCH = np.array([-1.7, 1.15, 1.25, 1.7, 1.15, -1.25])        # the arch target
Q_REACHABLE = np.array([-2.6, 1.25, 0.55, 2.6, 1.15, -0.55])   # feasible target
# Synthetic displacements beyond the rod's admissible deformation limits.  The
# target is therefore treated as unfeasible, rather than merely unreachable
# from one particular initial state.
UNFEASIBLE_SHIFTS = {1: (0.0, 0.6375), 2: (0.0, -0.525), 3: (0.0, 0.60)}

# Offline exploration: four signed perturbations of each gripper coordinate.
PROBE_AMPLITUDES = np.array([0.35, 0.35, 0.18, 0.35, 0.35, 0.18])
PROBE_FRACTIONS = (-1.0, -0.5, 0.5, 1.0)


def probe_motions(amplitudes=PROBE_AMPLITUDES, fractions=PROBE_FRACTIONS) -> np.ndarray:
    """Exploratory actions Delta Q (m x C): one coordinate perturbed at a time."""
    columns = [f * a * np.eye(len(amplitudes))[j]
               for j, a in enumerate(amplitudes) for f in fractions]
    return np.column_stack(columns)


class Plant:
    """Robot + rod + camera + feature map: command q, measure s.

    The object moves continuously, so each new equilibrium is computed from
    the previous one. `peek` returns the features that would be measured at q,
    starting from the current state, without moving the robot; it is used to
    probe the rod and to evaluate the exact Jacobian in simulation.
    """

    def __init__(self, rod: Rod | None = None, feature_map: str = "points",
                 n_points: int = N_POINTS, q=Q_NOMINAL):
        self.rod = rod if rod is not None else Rod()
        self.feature_map = feature_map
        self.n_points = n_points
        self.reset(q)

    def reset(self, q) -> np.ndarray:
        """Start an independent experiment at q, without a previous equilibrium."""
        self.q = np.array(q, float)
        self.nodes = self.rod.equilibrium(self.q)
        return self.s

    def move_to(self, q) -> np.ndarray:
        """Move the grippers to q, let the rod settle, and measure s."""
        self.q = np.array(q, float)
        self.nodes = self.rod.equilibrium(self.q, initial=self.nodes)
        return self.s

    def peek(self, q) -> np.ndarray:
        """Features at q, reached from the current equilibrium, without moving."""
        return self.features(self.rod.equilibrium(q, initial=self.nodes))

    def points_of(self, nodes) -> np.ndarray:
        return self.rod.sample_points(nodes, self.n_points)

    def features(self, nodes) -> np.ndarray:
        return feature_vector(self.points_of(nodes), self.feature_map)

    @property
    def points(self) -> np.ndarray:
        """Tracked points p_i of the current shape, (P, 2)."""
        return self.points_of(self.nodes)

    @property
    def s(self) -> np.ndarray:
        """Current feature vector s."""
        return self.features(self.nodes)


def target_from_grasp(q_star, rod: Rod | None = None, feature_map: str = "points",
                      n_points: int = N_POINTS):
    """Target recorded by demonstration: the equilibrium at a grasp q*.

    Returns (target nodes, target points, s*). Such a target is feasible by
    construction, s* = s(q*).
    """
    plant = Plant(rod, feature_map, n_points, q_star)
    return plant.nodes, plant.points, plant.s


def unfeasible_points(points, shifts=UNFEASIBLE_SHIFTS) -> np.ndarray:
    """Make a synthetic target that exceeds the rod's deformation limits."""
    points = np.array(points, float)
    for i, shift in shifts.items():
        points[i] += shift
    return points


# --------------------------------------------------------------------------- #
#  The looped rod of Section VI and Appendix E (feature choice, combined feature tasks)
# --------------------------------------------------------------------------- #
# Bringing the grippers close together and turning the wrists by -pi and +pi
# winds the rod into a hanging ring: the tangent turns by 2 pi along the rod.
# The ring does not intersect itself, so it is a valid planar configuration.
Q_RING = np.array([-0.4, 2.5, 0.0, 0.4, 2.5, 0.0])           # grasp before winding
Q_STRAIGHT = np.array([-2.7, 2.5, 0.0, 2.7, 2.5, 0.0])       # the same rod, unlooped


def ring_plant(feature_map: str = "points", n_points: int = N_POINTS,
               steps: int = 200) -> Plant:
    """Plant holding the looped rod, wound continuously from Q_RING."""
    plant = Plant(feature_map=feature_map, n_points=n_points, q=Q_RING)
    for t in np.linspace(0.0, 1.0, steps + 1)[1:]:
        q = Q_RING.copy()
        q[2] -= t * np.pi
        q[5] += t * np.pi
        plant.move_to(q)
    return plant


def rigid_grasp(q, angle: float = 0.0, shift=(0.0, 0.0), center=None) -> np.ndarray:
    """The grasp q moved rigidly: rotated by `angle` about `center` (default: the
    midpoint of the grippers), then translated by `shift`."""
    q = np.array(q, float)
    center = 0.5 * (q[:2] + q[3:5]) if center is None else np.asarray(center, float)
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    for i in (0, 3):
        q[i:i + 2] = rotation @ (q[i:i + 2] - center) + center + np.asarray(shift, float)
        q[i + 2] += angle
    return q


# Target of the combined example: the unlooped rod at another pose.
Q_STRAIGHT_MOVED = rigid_grasp(Q_STRAIGHT, np.radians(20.0), (1.2, 0.6))


# Backwards-compatible name used by earlier releases of the tutorial code.
UNREACHABLE_SHIFTS = UNFEASIBLE_SHIFTS


def unreachable_points(points, shifts=UNFEASIBLE_SHIFTS) -> np.ndarray:
    """Deprecated alias for :func:`unfeasible_points`."""
    return unfeasible_points(points, shifts)

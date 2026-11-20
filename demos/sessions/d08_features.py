"""
Demo 09 - Choosing the shape features.
======================================

Point coordinates, edge vectors, and curvature-based features (turning angles) computed from the same
tracked points. Three experiments, as in the tutorial's feature figure:

    gestures      rigid translation, rigid rotation, and a non-rigid
                  deformation applied to the tracked points: which features change?
    closed loop   the loop with each representation, from a common start to the
                  arch target, while a temporary disturbance pulls the grippers
                  during the first 20 iterations: each loop undoes only the drift
                  that its features register
    null space    robot motions in Null(J) of the chosen representation
    looped rod    the loop with each representation, from a looped rod (a hanging
                  ring) to the same rod unlooped: shape error and the largest
                  axial strain imposed on the rod along the way
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (FEATURE_MAPS, Plant, Q_ARCH, Q_STRAIGHT, ShapeServoingLoop,
                            feature_dimension, feature_vector, null_space, numerical_jacobian,
                            numerical_rank, offline_estimate, probe_motions, rigid_motion,
                            ring_plant, target_from_grasp)

from .common import Session, scene

START = np.array([-2.45, 1.10, 0.55, 2.45, 1.10, -0.55])     # a shallower arch


def transformed_grasp(q, angle, shift, center=(0.0, 1.15)):
    """The grasp q moved rigidly: rotated by `angle` about `center`, then shifted."""
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    q = np.array(q, float)
    for i in (0, 3):
        q[i:i + 2] = rotation @ (q[i:i + 2] - center) + center + np.asarray(shift)
        q[i + 2] += angle
    return q


GAINS = {"points": 0.06, "edges": 0.08, "angles": 0.12}

# The temporary disturbance of the tutorial's feature figure, the same in every
# run: for the first DISTURBANCE_STEPS iterations it pulls the grippers toward a
# shifted and rotated copy of the target grasp at DISTURBANCE_RATE, then stops.
DISTURBANCE_TARGET = transformed_grasp(Q_ARCH, np.radians(-24.0), (2.0, 0.15))
DISTURBANCE_RATE = 0.03
DISTURBANCE_STEPS = 20
COMPARE_HORIZON = 80


def wiggle(t, terms):
    """Sum of a_k sin(k pi t): zero at t = 0 and t = 1, oscillating in between."""
    return sum(a * np.sin(k * np.pi * t) for k, a in terms)


LOOP_GAINS = GAINS             # as in the closed-loop runs
LOOP_HORIZON = 200


class FeaturesSession(Session):

    def __init__(self):
        self.arch_nodes, self.arch_points, _ = target_from_grasp(Q_ARCH)
        self.straight_nodes, self.straight_points, _ = target_from_grasp(Q_STRAIGHT)
        self.rep = "points"
        self.ring_rep = "points"
        self._setup()
        self._setup_ring()

    def _make_loop(self, rep):
        plant = Plant(feature_map=rep, q=START)
        J0, _, _ = offline_estimate(plant.peek, START, probe_motions())
        loop = ShapeServoingLoop(plant, feature_vector(self.arch_points, rep), J0, alpha=GAINS[rep],
                                 damping=0.05, max_response=25.0, relaxation=0.4)
        # The disturbance acts on the robot motion, outside the controller: added, not projected.
        loop.secondary = lambda q: (-DISTURBANCE_RATE * (q - DISTURBANCE_TARGET)
                                    if loop.k < DISTURBANCE_STEPS else np.zeros_like(q))
        loop.project_secondary = False
        return plant, J0, loop

    def _setup(self):
        self.plant, self.J0, self.loop = self._make_loop(self.rep)
        self.initial_error = float(np.linalg.norm(self.loop.error))

    def state(self) -> dict:
        rank = numerical_rank(self.J0)
        return {"scene": scene(self.plant), "target": {"nodes": self.arch_nodes, "points": self.arch_points},
                "rep": self.rep, "n": feature_dimension(5, self.rep), "rank": rank, "null_dim": 6 - rank,
                "k": self.loop.k, "error": float(np.linalg.norm(self.loop.error)),
                "initial_error": self.initial_error, "disturbance_steps": DISTURBANCE_STEPS,
                "horizon": COMPARE_HORIZON}

    # ------------------------------------------------------------- closed loop
    def act_set_rep(self, rep):
        self.rep = rep
        self._setup()
        return self.state()

    def act_reset(self):
        self._setup()
        return self.state()

    def act_step(self, n: int = 1):
        samples = []
        for _ in range(max(1, int(n))):
            self.loop.step()
            samples.append({"k": self.loop.k, "error": float(np.linalg.norm(self.loop.error))})
        return {**self.state(), "samples": samples}

    def act_compare(self, iterations: int = COMPARE_HORIZON):
        """Run every representation from the common start; return the final shapes."""
        out = {}
        for rep in FEATURE_MAPS:
            plant, _, loop = self._make_loop(rep)
            e0 = np.linalg.norm(loop.error)
            for _ in range(iterations):
                loop.step()
            out[rep] = {"nodes": plant.nodes, "points": plant.points, "q": plant.q,
                        "ratio": float(np.linalg.norm(loop.error) / e0)}
        return {"runs": out, "start": Plant(q=START).nodes}

    # ------------------------------------------------------------- looped rod
    def _make_ring_loop(self, rep):
        plant = ring_plant(rep)
        J0, _, _ = offline_estimate(plant.peek, plant.q, probe_motions())
        loop = ShapeServoingLoop(plant, feature_vector(self.straight_points, rep), J0,
                                 alpha=LOOP_GAINS[rep], damping=0.05, max_response=25.0,
                                 relaxation=0.4)
        return plant, loop

    def _setup_ring(self):
        self.ring, self.ring_loop = self._make_ring_loop(self.ring_rep)
        self.ring_initial_error = float(np.linalg.norm(self.ring_loop.error))
        self.ring_peak = float(self.ring.rod.axial_strain(self.ring.nodes).max())

    def _ring_state(self) -> dict:
        strain = float(self.ring.rod.axial_strain(self.ring.nodes).max())
        return {"scene": scene(self.ring), "rep": self.ring_rep, "k": self.ring_loop.k,
                "target": {"nodes": self.straight_nodes, "points": self.straight_points},
                "error": float(np.linalg.norm(self.ring_loop.error)) / self.ring_initial_error,
                "strain": strain, "peak_strain": self.ring_peak,
                "energy": self.ring.rod.elastic_energy(self.ring.nodes)}

    def act_ring_state(self):
        return {"ring": self._ring_state()}

    def act_set_ring_rep(self, rep):
        self.ring_rep = rep
        self._setup_ring()
        return {"ring": self._ring_state()}

    def act_reset_ring(self):
        self._setup_ring()
        return {"ring": self._ring_state()}

    def act_step_ring(self, n: int = 1):
        samples = []
        for _ in range(max(1, int(n))):
            self.ring_loop.step()
            state = self._ring_state()
            self.ring_peak = max(self.ring_peak, state["strain"])
            samples.append({"k": state["k"], "error": state["error"], "strain": state["strain"]})
        return {"ring": self._ring_state(), "samples": samples}

    def act_compare_ring(self, iterations: int = LOOP_HORIZON):
        """Run every representation from the looped rod; return error and strain histories."""
        out = {}
        for rep in FEATURE_MAPS:
            plant, loop = self._make_ring_loop(rep)
            e0 = np.linalg.norm(loop.error)
            error, strain = [1.0], [float(plant.rod.axial_strain(plant.nodes).max())]
            peak_nodes, peak = plant.nodes, strain[0]
            for _ in range(int(iterations)):
                loop.step()
                error.append(float(np.linalg.norm(loop.error) / e0))
                strain.append(float(plant.rod.axial_strain(plant.nodes).max()))
                if strain[-1] > peak:
                    peak, peak_nodes = strain[-1], plant.nodes
            out[rep] = {"error": error, "strain": strain, "nodes": plant.nodes,
                        "points": plant.points, "peak_nodes": peak_nodes, "peak": peak}
        return {"runs": out, "start": ring_plant().nodes,
                "target": {"nodes": self.straight_nodes, "points": self.straight_points}}

    # ------------------------------------------------------------- gestures
    def act_gesture(self, kind: str, frames: int = 121):
        """Features of all three maps while the tracked points translate, rotate, or deform."""
        points, nodes = self.arch_points, self.arch_nodes
        xi = np.linspace(-1.0, 1.0, len(points))
        bow = np.column_stack([0.10 * np.sin(np.pi * xi), np.cos(0.5 * np.pi * xi)])
        ess = np.column_stack([np.zeros_like(xi), np.sin(np.pi * xi)])
        base = {rep: feature_vector(points, rep) for rep in FEATURE_MAPS}
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(nodes, axis=0), axis=1))]
        point_arc = np.linspace(0.12, 0.88, len(points)) * arc[-1]
        out = []
        for t in np.linspace(0.0, 1.0, frames):
            if kind == "translate":
                shift = (wiggle(t, [(2, 0.45), (5, 0.18)]), wiggle(t, [(3, 0.30), (6, -0.14)]))
                moved, moved_nodes = rigid_motion(points, 0.0, shift), nodes + np.asarray(shift)
            elif kind == "rotate":
                angle = np.radians(wiggle(t, [(2, 22.0), (5, 9.0)]))
                moved = rigid_motion(points, angle)
                c, s = np.cos(angle), np.sin(angle)
                centroid = points.mean(axis=0)
                moved_nodes = (nodes - centroid) @ np.array([[c, -s], [s, c]]).T + centroid
            else:
                moved = points + wiggle(t, [(3, 0.55)]) * bow + wiggle(t, [(4, 0.32), (7, 0.10)]) * ess
                shift = moved - points        # drawn on the rod by interpolating along its length
                moved_nodes = nodes + np.column_stack([np.interp(arc, point_arc, shift[:, 0]),
                                                       np.interp(arc, point_arc, shift[:, 1])])
            out.append({"nodes": moved_nodes, "points": moved,
                        "features": {rep: feature_vector(moved, rep) - base[rep] for rep in FEATURE_MAPS}})
        return {"kind": kind, "frames": out}

    # ------------------------------------------------------------- null space
    def act_null_motion(self, frames: int = 48, amplitude: float = 0.5):
        """Robot motion along the first direction of Null(J) for the chosen features."""
        J = numerical_jacobian(self.plant.peek, self.plant.q)
        basis = null_space(J)
        if basis.shape[1] == 0:
            return {"frames": [], "null_dim": 0}
        direction = basis[:, 0] * amplitude / np.max(np.abs(basis[:, 0]))
        s0, p0 = self.plant.s, self.plant.points
        nodes, out = self.plant.nodes, []
        for t in np.linspace(0.0, 1.0, frames + 1):
            q = self.plant.q + np.sin(2 * np.pi * t) * direction
            nodes = self.plant.rod.equilibrium(q, initial=nodes)
            points = self.plant.points_of(nodes)
            out.append({"q": q, "nodes": nodes, "points": points,
                        "feature_change": float(np.linalg.norm(self.plant.features(nodes) - s0)),
                        "point_change": float(np.linalg.norm(points - p0))})
        return {"frames": out, "null_dim": basis.shape[1], "direction": direction}

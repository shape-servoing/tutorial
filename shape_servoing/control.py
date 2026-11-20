"""
The control law and the complete shape-servoing loop (Algorithm 1).
===================================================================

    u = alpha J_hat^+ e                                 (pseudoinverse law)
    u = alpha J_hat^T (J_hat J_hat^T + mu^2 I)^{-1} e   (damped least squares)

`ShapeServoingLoop.step` runs one iteration of Algorithm 1: error, command,
motion, measurement, and the gated Broyden update of the estimate. Two
extensions of Appendix E are optional: a feedforward term J_hat^+ ds*/dt for a
moving target, and a secondary objective added to the command, optionally
projected onto Null(J_hat). `FeatureTask` is a secondary objective defined by a
second feature map, with its own target and Jacobian estimate, e.g. point
coordinates that transport the rod while turning angles regulate its shape. The
turning angles are a known function of the points, so their Jacobian can then be
derived from the point estimate by the chain rule instead of being estimated
separately (`ShapeServoingLoop.derive_jacobian_from`).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .features import feature_map_jacobian, feature_vector
from .jacobian import broyden_update, offline_estimate


def control_law(J_hat, e, alpha: float, damping: float = 0.0) -> np.ndarray:
    """Robot velocity u for the error e (pseudoinverse, or damped if damping > 0)."""
    J_hat = np.asarray(J_hat, float)
    if damping <= 0.0:
        return alpha * np.linalg.pinv(J_hat) @ e
    n = J_hat.shape[0]
    return alpha * J_hat.T @ np.linalg.solve(J_hat @ J_hat.T + damping ** 2 * np.eye(n), e)


@dataclass
class Iteration:
    """Signals of one loop iteration k."""
    k: int
    e: np.ndarray          # e_k = s* - s_k
    u: np.ndarray          # u_k
    dq: np.ndarray         # dq_k = u_k dt
    ds: np.ndarray         # ds_{k+1} = s_{k+1} - s_k
    r: np.ndarray          # prediction error of J_hat_k on this pair
    updated: bool          # whether the Broyden update was applied


class ShapeServoingLoop:
    """Online Jacobian-based shape servoing (Algorithm 1)."""

    def __init__(self, plant, s_star, J0, alpha: float = 0.05, dt: float = 1.0,
                 online_update: bool = True, damping: float = 0.0,
                 min_step: float = 1e-6, max_response: float = np.inf,
                 relaxation: float = 1.0):
        self.plant = plant
        self.s_star = np.asarray(s_star, float)
        self.J_hat = np.array(J0, float)
        self.alpha, self.dt = alpha, dt
        self.online_update, self.damping = online_update, damping
        # Remark on gating: skip updates from tiny steps, and reject pairs whose
        # response ratio ||ds|| / ||dq|| is implausibly large.
        self.min_step, self.max_response = min_step, max_response
        self.relaxation = relaxation
        self.secondary = None              # callable q -> robot velocity
        self.project_secondary = True      # pursue it within Null(J_hat)
        self.jacobian_from = None          # FeatureTask giving J_hat by the chain rule
        self.s = plant.s                   # line 1: measure s_0
        self.k = 0

    @property
    def error(self) -> np.ndarray:
        return self.s_star - self.s

    def derive_jacobian_from(self, task) -> None:
        """Obtain J_hat from a point-coordinate FeatureTask by the chain rule,
        J_hat = (ds/ds_p) J_p_hat, at every iteration, instead of updating it
        with Broyden's rule. Only J_p_hat is estimated (from the same robot motions)."""
        self.jacobian_from = task
        self.J_hat = task.chained_jacobian(self.plant.feature_map)

    def command(self, e, s_star_rate=None) -> np.ndarray:
        """u_k = alpha J_hat^+ e_k, plus the optional extensions of Appendix E."""
        u = control_law(self.J_hat, e, self.alpha, self.damping)
        if s_star_rate is not None:                                   # moving target
            u = u + np.linalg.pinv(self.J_hat) @ s_star_rate
        if self.secondary is not None:                                # secondary task
            u2 = self.secondary(self.plant.q)
            if self.project_secondary:
                J_pinv = np.linalg.pinv(self.J_hat)
                u2 = (np.eye(len(u2)) - J_pinv @ self.J_hat) @ u2
            u = u + u2
        return u

    def step(self, s_star_rate=None) -> Iteration:
        if self.jacobian_from is not None:                            # chain rule
            self.J_hat = self.jacobian_from.chained_jacobian(self.plant.feature_map)
        e = self.s_star - self.s                                      # line 3
        u = self.command(e, s_star_rate)                              # line 4
        dq = u * self.dt                                              # line 5
        s_next = self.plant.move_to(self.plant.q + dq)                # line 6
        ds = s_next - self.s                                          # line 7
        r = ds - self.J_hat @ dq                                      # line 8
        step = np.linalg.norm(dq)
        updated = (self.online_update and self.jacobian_from is None and step > self.min_step
                   and np.linalg.norm(ds) <= self.max_response * step)
        if updated:
            self.J_hat, r = broyden_update(self.J_hat, dq, ds, self.relaxation)
        tasks = [self.secondary] if hasattr(self.secondary, "observe") else []   # FeatureTask
        if self.jacobian_from is not None and self.jacobian_from is not self.secondary:
            tasks.append(self.jacobian_from)
        for task in tasks:
            task.observe(dq)
        record = Iteration(self.k, e, u, dq, ds, r, updated)
        self.s, self.k = s_next, self.k + 1                           # line 9
        return record


class FeatureTask:
    """Secondary objective defined by a second feature map.

    It requests u_2 = beta J2_hat^+ (s2* - s2), computed with the damped inverse,
    from its own feature vector s2 (e.g. point coordinates) and its own Jacobian
    estimate J2_hat. J2_hat is initialized by offline regression around the
    current grasp and updated with Broyden's rule from the same robot motions as
    the primary estimate. Assign it to `ShapeServoingLoop.secondary`; with
    `project_secondary = True` the loop pursues it within Null(J_hat) of the
    primary features. For point coordinates, `chained_jacobian` gives the
    Jacobian of any other feature map of the same points.
    """

    def __init__(self, plant, feature_map: str, s_star, gain: float = 0.05,
                 damping: float = 0.05, relaxation: float = 1.0,
                 max_response: float = 25.0, probes=None):
        from .running_example import probe_motions
        self.plant, self.feature_map = plant, feature_map
        self.s_star = np.asarray(s_star, float)
        self.gain, self.damping = gain, damping
        self.relaxation, self.max_response = relaxation, max_response
        measure = lambda q: self._features(plant.rod.equilibrium(q, initial=plant.nodes))
        self.J_hat, _, _ = offline_estimate(measure, plant.q,
                                            probe_motions() if probes is None else probes)
        self.s = self._features(plant.nodes)

    def _features(self, nodes) -> np.ndarray:
        return feature_vector(self.plant.points_of(nodes), self.feature_map)

    @property
    def error(self) -> np.ndarray:
        return self.s_star - self.s

    def __call__(self, q) -> np.ndarray:
        return control_law(self.J_hat, self.error, self.gain, self.damping)

    def chained_jacobian(self, feature_map: str) -> np.ndarray:
        """(ds/ds_p) J_p_hat: Jacobian of another feature map of the tracked points."""
        if self.feature_map != "points":
            raise ValueError("the chain rule needs a point-coordinate task")
        return feature_map_jacobian(self.plant.points, feature_map) @ self.J_hat

    def observe(self, dq) -> None:
        """Measure s2 after the robot moved by dq and update J2_hat (gated)."""
        s_next = self._features(self.plant.nodes)
        ds, step = s_next - self.s, np.linalg.norm(dq)
        if step > 1e-6 and np.linalg.norm(ds) <= self.max_response * step:
            self.J_hat, _ = broyden_update(self.J_hat, dq, ds, self.relaxation)
        self.s = s_next

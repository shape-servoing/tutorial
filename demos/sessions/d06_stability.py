"""
Demo 07 - Local stability: the descent condition.
=================================================

For the running example n > m, the paper's sufficient condition is positivity
of Sym(J_hat^+ J) on Range(J^T). The page builds J_hat by corrupting the exact
J (scaling, mixing two robot coordinates, random perturbation, a flipped
column), evaluates the restricted eigenvalues and direct descent angles, and
runs the loop with J_hat held fixed.
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (Plant, Q_NOMINAL, ShapeServoingLoop, descent_angles, descent_margin,
                            numerical_jacobian, singular_triplets)

from .common import Session, scene

# Robot-coordinate planes in which the estimate "rotates" the columns of J.
PLANES = {"xL-xR": (0, 3), "yL-thetaL": (1, 2), "thetaL-thetaR": (2, 5)}
ANGLE_BINS = np.linspace(0.0, 180.0, 37)
MAX_GROWTH = 6.0       # stop a diverging run when ||e_k|| exceeds this multiple of ||e_0||


class StabilitySession(Session):

    def __init__(self):
        rng = np.random.default_rng(7)
        self.plant = Plant(q=Q_NOMINAL)
        self.q_c = Q_NOMINAL.copy()
        self.J = numerical_jacobian(self.plant.peek, self.q_c)
        self.perturbation = rng.standard_normal(self.J.shape)
        # Random unit errors in Range(J), for the descent-angle histogram.
        _, Phi, _ = singular_triplets(self.J)
        z = rng.standard_normal((Phi.shape[1], 600))
        self.samples = Phi @ (z / np.linalg.norm(z, axis=0))
        self.rng = np.random.default_rng(11)
        self.params = {"scale": 1.0, "angle": 0.0, "plane": "yL-thetaL", "noise": 0.0, "flip": -1}
        self.alpha, self.distance = 0.3, 0.4
        self.direction = np.array([0.5, 0.6, 0.3, -0.4, 0.2, -0.3])
        self._estimate()
        self._new_target()

    # ------------------------------------------------------------- model
    def _estimate(self):
        p = self.params
        i, j = PLANES[p["plane"]]
        R = np.eye(6)
        c, s = np.cos(np.radians(p["angle"])), np.sin(np.radians(p["angle"]))
        R[[i, i, j, j], [i, j, i, j]] = [c, -s, s, c]
        J_hat = p["scale"] * self.J @ R
        J_hat = J_hat + p["noise"] * np.linalg.norm(self.J) * self.perturbation / np.linalg.norm(self.perturbation)
        if p["flip"] >= 0:
            J_hat[:, p["flip"]] *= -1
        self.J_hat = J_hat

    def _new_target(self):
        """s* = s(q_c + delta): locally reachable by construction."""
        dq = self.distance * self.direction / np.linalg.norm(self.direction)
        nodes = self.plant.rod.equilibrium(self.q_c + dq, initial=self.plant.nodes)
        self.target = {"nodes": nodes, "points": self.plant.points_of(nodes)}
        self.s_star = self.plant.features(nodes)
        self._restart()

    def _restart(self):
        self.plant.reset(self.q_c)
        self.loop = ShapeServoingLoop(self.plant, self.s_star, self.J_hat, alpha=self.alpha,
                                      online_update=False)
        self.initial_error = float(np.linalg.norm(self.loop.error))

    def state(self) -> dict:
        margin, eigenvalues = descent_margin(self.J, self.J_hat)
        angles = np.degrees(descent_angles(self.J, self.J_hat, self.samples))
        e = self.loop.error
        velocity = self.J @ np.linalg.pinv(self.J_hat) @ e      # feature velocity the command produces
        rate = float(e @ velocity / max(e @ e, 1e-300))
        return {"scene": scene(self.plant), "target": self.target, "k": self.loop.k,
                "error": float(np.linalg.norm(e)), "initial_error": self.initial_error,
                "e": e, "velocity": velocity,
                "J": self.J, "J_hat": self.J_hat,
                "matrix_error": float(np.linalg.norm(self.J - self.J_hat) / np.linalg.norm(self.J)),
                "margin": margin, "eigenvalues": eigenvalues,
                "angle_counts": np.histogram(angles, ANGLE_BINS)[0], "angle_bins": ANGLE_BINS,
                "descent_fraction": float(np.mean(angles < 90.0)), "rate": rate,
                "params": {**self.params, "alpha": self.alpha, "distance": self.distance}}

    # ------------------------------------------------------------- actions
    def act_set_estimate(self, **params):
        self.params.update({k: v for k, v in params.items() if k in self.params})
        self._estimate()
        self._restart()
        return self.state()

    def act_set_alpha(self, alpha):
        self.alpha = float(alpha)
        self._restart()
        return self.state()

    def act_new_target(self, distance=None):
        self.distance = self.distance if distance is None else float(distance)
        self.direction = self.rng.standard_normal(6) * np.array([1, 1, 0.5, 1, 1, 0.5])
        self._new_target()
        return self.state()

    def act_reset(self):
        self._restart()
        return self.state()

    def act_step(self, n: int = 1):
        samples = []
        for _ in range(max(1, int(n))):
            if np.linalg.norm(self.loop.error) > MAX_GROWTH * self.initial_error:
                break
            self.loop.step()
            e = self.loop.error
            samples.append({"k": self.loop.k, "error": float(np.linalg.norm(e)),
                            "rate": float(e @ self.J @ np.linalg.pinv(self.J_hat) @ e / max(e @ e, 1e-300))})
        diverged = bool(np.linalg.norm(self.loop.error) > MAX_GROWTH * self.initial_error)
        return {**self.state(), "samples": samples, "diverged": diverged}

"""
Demo 03 - Offline estimation by regression.
===========================================

The robot probes the rod with C small motions around q0 and fits
J0 = dS dQ^+. The page varies the probing amplitude, the number of probes,
and the measurement noise, and compares the estimate with the exact Jacobian
(simulation only) and with a model-based estimate from a mismatched rod model.
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (PROBE_AMPLITUDES, Plant, Q_NOMINAL, Rod, condition_number,
                            numerical_jacobian, offline_estimate, probe_motions,
                            singular_triplets)

from .common import Session, admissible, scene

FRACTIONS = {1: (1.0,), 2: (-1.0, 1.0), 4: (-1.0, -0.5, 0.5, 1.0)}
SWEEP_SCALES = np.geomspace(0.02, 4.0, 14)


class OfflineSession(Session):

    def __init__(self):
        self.plant = Plant(q=Q_NOMINAL)
        self.scale, self.noise, self.per_dof, self.seed = 1.0, 0.0, 4, 0
        self.model = {"bend": 1.0, "gravity": 1.0}
        self._configuration_changed()

    def _configuration_changed(self):
        self.J = numerical_jacobian(self.plant.peek, self.plant.q)
        self.estimate = None

    def _noisy_measure(self, rng):
        """Perception with additive Gaussian noise on every feature coordinate."""
        def measure(q):
            s = self.plant.peek(q)
            return s + self.noise * rng.standard_normal(s.size)
        return measure

    def _estimate(self, scale, seed):
        dQ = probe_motions(scale * PROBE_AMPLITUDES, FRACTIONS[self.per_dof])
        J0, dS, residual = offline_estimate(self._noisy_measure(np.random.default_rng(seed)),
                                            self.plant.q, dQ)
        return J0, dQ, dS, residual

    def _relative_error(self, J_hat) -> float:
        return float(np.linalg.norm(J_hat - self.J) / np.linalg.norm(self.J))

    def state(self) -> dict:
        return {"scene": scene(self.plant), "J": self.J, "estimate": self.estimate,
                "params": {"scale": self.scale, "noise": self.noise, "per_dof": self.per_dof},
                "model": self._model_estimate()}

    def _model_estimate(self) -> dict:
        """J_g = dg/dq of a rod model whose bending stiffness and weight may be wrong."""
        rod = Rod(k_bend=4.0 * self.model["bend"], gravity=0.45 * self.model["gravity"])
        model = Plant(rod=rod, q=self.plant.q)
        J_g = numerical_jacobian(model.peek, self.plant.q)
        return {**self.model, "J": J_g, "error": self._relative_error(J_g)}

    # ------------------------------------------------------------- actions
    def act_move(self, q):
        self.plant.move_to(admissible(q))
        self._configuration_changed()
        return self.state()

    def act_set_params(self, scale=None, noise=None, per_dof=None):
        self.scale = self.scale if scale is None else float(scale)
        self.noise = self.noise if noise is None else float(noise)
        self.per_dof = self.per_dof if per_dof is None else int(per_dof)
        self.estimate = None
        return self.state()

    def act_set_model(self, bend=None, gravity=None):
        self.model["bend"] = self.model["bend"] if bend is None else float(bend)
        self.model["gravity"] = self.model["gravity"] if gravity is None else float(gravity)
        return {"model": self._model_estimate()}

    def act_probe(self):
        """Run the C exploratory motions and fit J0. Returns each probe for animation."""
        self.seed += 1
        J0, dQ, dS, residual = self._estimate(self.scale, self.seed)
        sigma, _, _ = singular_triplets(J0)
        self.estimate = {"J0": J0, "dQ": dQ, "dS": dS, "residual": residual,
                         "dS_norm": float(np.linalg.norm(dS)), "error": self._relative_error(J0),
                         "condition": condition_number(J0), "sigma": sigma}
        probes = []
        for dq in dQ.T:
            nodes = self.plant.rod.equilibrium(self.plant.q + dq, initial=self.plant.nodes)
            probes.append({"q": self.plant.q + dq, "nodes": nodes, "points": self.plant.points_of(nodes)})
        return {**self.state(), "probes": probes}

    def act_sweep(self):
        """Residual and estimation error as the probing amplitude varies."""
        ratio, error = [], []
        for scale in SWEEP_SCALES:
            runs = [self._estimate(scale, seed) for seed in (101, 202)]
            ratio.append(np.mean([r / np.linalg.norm(dS) for _, _, dS, r in runs]))
            error.append(np.mean([self._relative_error(J0) for J0, _, _, _ in runs]))
        return {"scales": SWEEP_SCALES, "residual_ratio": ratio, "error": error}

"""
Demo 04 - Closing the loop (Algorithm 1).
=========================================

The shape-servoing loop runs from q0 toward a target, starting from the
offline estimate J0. Each `step` executes Algorithm 1 iterations; a single
step also returns the intermediate signals (e_k, u_k, dq_k, ds_{k+1}, the
prediction error, and the rank-one Broyden correction) for step-by-step display.
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (Plant, Q_ARCH, Q_NOMINAL, ShapeServoingLoop, condition_number,
                            offline_estimate, probe_motions, target_from_grasp)

from .common import Session, admissible, random_target, scene, target_scene

GATE = 25.0      # default largest accepted response ratio ||ds|| / ||dq|| when gating is on


class LoopSession(Session):

    def __init__(self):
        self.rng = np.random.default_rng(4)
        self.plant = Plant(q=Q_NOMINAL)
        self.J0, _, _ = offline_estimate(self.plant.peek, Q_NOMINAL, probe_motions())
        self.alpha, self.online, self.gate = 0.05, True, False
        self.gate_ratio = GATE
        self._set_target(*target_from_grasp(Q_ARCH))

    def _set_target(self, nodes, points, s_star):
        self.target = target_scene(nodes, points)
        self.s_star = s_star
        self._restart()

    def _restart(self):
        """Back to q0 with the offline estimate J0: a new run of Algorithm 1."""
        self.plant.reset(Q_NOMINAL)
        self.loop = ShapeServoingLoop(self.plant, self.s_star, self.J0, alpha=self.alpha,
                                      online_update=self.online,
                                      max_response=self.gate_ratio if self.gate else np.inf)
        self.initial_error = float(np.linalg.norm(self.loop.error))

    def state(self) -> dict:
        return {"scene": scene(self.plant), "target": self.target, "k": self.loop.k,
                "e": self.loop.error, "error": float(np.linalg.norm(self.loop.error)),
                "initial_error": self.initial_error, "J_hat": self.loop.J_hat,
                "condition": condition_number(self.loop.J_hat),
                "params": {"alpha": self.alpha, "online": self.online, "gate": self.gate,
                           "gate_ratio": self.gate_ratio}}

    # ------------------------------------------------------------- actions
    def act_step(self, n: int = 1):
        """Run n iterations. Returns the new samples, and the details of the last one."""
        before = {"nodes": self.plant.nodes.copy(), "points": self.plant.points, "q": self.plant.q.copy()}
        J_before = self.loop.J_hat.copy()
        samples = []
        for _ in range(int(n)):
            it = self.loop.step()
            dq_norm = float(np.linalg.norm(it.dq))
            samples.append({"k": self.loop.k, "error": float(np.linalg.norm(self.loop.error)),
                            "u": it.u, "updated": it.updated,
                            "correction": float(np.linalg.norm(it.r) / max(dq_norm, 1e-12)),
                            "ratio": float(np.linalg.norm(it.ds) / max(dq_norm, 1e-12)),
                            "rejected": bool(self.online and dq_norm > self.loop.min_step
                                             and not it.updated)})
        detail = {"before": before, "e": it.e, "u": it.u, "dq": it.dq, "ds": it.ds, "r": it.r,
                  "updated": it.updated, "J_before": J_before,
                  "correction": self.loop.J_hat - J_before}
        return {**self.state(), "samples": samples, "detail": detail}

    def act_set_params(self, alpha=None, online=None, gate=None, gate_ratio=None):
        self.alpha = self.alpha if alpha is None else float(alpha)
        self.online = self.online if online is None else bool(online)
        self.gate = self.gate if gate is None else bool(gate)
        self.gate_ratio = self.gate_ratio if gate_ratio is None else float(gate_ratio)
        self.loop.alpha, self.loop.online_update = self.alpha, self.online
        self.loop.max_response = self.gate_ratio if self.gate else np.inf
        return self.state()

    def act_reset(self):
        self._restart()
        return self.state()

    def act_set_target(self, kind: str):
        """'arch', 'random', or 'current': the shape the rod has now (by demonstration)."""
        if kind == "arch":
            self._set_target(*target_from_grasp(Q_ARCH))
        elif kind == "random":
            self.plant.reset(Q_NOMINAL)
            _, nodes, points, s_star = random_target(self.rng, self.plant)
            self._set_target(nodes, points, s_star)
        else:
            self._set_target(self.plant.nodes.copy(), self.plant.points, self.plant.s)
        return self.state()

    def act_move(self, q):
        """Move the grippers by hand (paused): no robot command, so no Broyden pair."""
        self.loop.s = self.plant.move_to(admissible(q))
        return self.state()

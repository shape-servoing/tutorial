"""
Demo 05 - Target feasibility and reachability.
================================================

A feasible and reached target (the equilibrium at another grasp) and a
synthetic infeasible one (three points exceed the deformation limits). The page splits the error into the part the
controller acts on, J J^+ e in Range(J), and the rest, which no small robot
motion changes. With the exact Jacobian (simulation only), the loop settles
where the remaining error is orthogonal to Range(J).
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (Plant, Q_NOMINAL, Q_REACHABLE, ShapeServoingLoop, unfeasible_points,
                            numerical_jacobian, offline_estimate, probe_motions, range_split,
                            target_from_grasp)

from .common import Session, scene


class TargetsSession(Session):

    def __init__(self):
        self.plant = Plant(q=Q_NOMINAL)
        self.J0, _, _ = offline_estimate(self.plant.peek, Q_NOMINAL, probe_motions())
        self.feasible_nodes, self.feasible_points, _ = target_from_grasp(Q_REACHABLE)
        self.exact, self.alpha = False, 0.06
        self.kind, self.points = "reachable", self.feasible_points.copy()
        self._restart()

    def _restart(self):
        self.plant.reset(Q_NOMINAL)
        self.loop = ShapeServoingLoop(self.plant, self.points.ravel(), self.J0, alpha=self.alpha,
                                      damping=0.02, max_response=25.0, online_update=not self.exact)
        self.initial_error = float(np.linalg.norm(self.loop.error))

    def _split(self) -> dict:
        """Split e into its component in Range(J_hat) and the orthogonal component."""
        J = numerical_jacobian(self.plant.peek, self.plant.q)
        if self.exact:
            self.loop.J_hat = J
        e = self.loop.error
        e_range, e_orth = range_split(self.loop.J_hat, e)
        exact_range, exact_orth = range_split(J, e)
        norm = max(np.linalg.norm(e), 1e-12)
        return {"e": e, "e_range": e_range, "e_orth": e_orth,
                "range_fraction": float(np.linalg.norm(e_range) / norm),
                "orth_fraction": float(np.linalg.norm(e_orth) / norm),
                "exact_range_fraction": float(np.linalg.norm(exact_range) / norm),
                "exact_orth_fraction": float(np.linalg.norm(exact_orth) / norm)}

    def state(self) -> dict:
        nodes = self.feasible_nodes if self.kind == "reachable" else None
        return {"scene": scene(self.plant), "target": {"nodes": nodes, "points": self.points},
                "kind": self.kind, "k": self.loop.k, "error": float(np.linalg.norm(self.loop.error)),
                "initial_error": self.initial_error, "split": self._split(),
                "params": {"exact": self.exact, "alpha": self.alpha}}

    # ------------------------------------------------------------- actions
    def act_step(self, n: int = 1):
        samples = []
        for _ in range(max(1, int(n))):
            if self.exact:
                self.loop.J_hat = numerical_jacobian(self.plant.peek, self.plant.q)
            it = self.loop.step()
            samples.append({"k": self.loop.k, "error": float(np.linalg.norm(self.loop.error)),
                            "command": float(np.linalg.norm(it.u))})
        return {**self.state(), "samples": samples}

    def act_set_target(self, kind: str):
        if kind not in ("reachable", "unfeasible"):
            raise ValueError(f"unknown target class: {kind}")
        self.kind = kind
        self.points = (self.feasible_points.copy() if kind == "reachable"
                       else unfeasible_points(self.feasible_points))
        self._restart()
        return self.state()

    def act_move_point(self, index: int, xy):
        """Drag one target point: the target is no longer a recorded shape."""
        self.kind = "custom"
        self.points[int(index)] = np.asarray(xy, float)
        self.loop.s_star = self.points.ravel().copy()
        if self.loop.k == 0:
            self.initial_error = float(np.linalg.norm(self.loop.error))
        return self.state()

    def act_set_params(self, exact=None, alpha=None):
        self.exact = self.exact if exact is None else bool(exact)
        self.alpha = self.alpha if alpha is None else float(alpha)
        self._restart()
        return self.state()

    def act_reset(self):
        self._restart()
        return self.state()

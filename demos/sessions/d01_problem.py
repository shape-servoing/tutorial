"""
Demo 01 - The shape-servoing problem.
=====================================

The user moves the grippers by hand. The rod settles to its quasistatic
equilibrium, perception returns the tracked points, and the page shows the
feature vector s, the target s*, and the shape error e = s* - s.
"""
from __future__ import annotations

import numpy as np

from shape_servoing import Plant, Q_ARCH, Q_NOMINAL, target_from_grasp

from .common import Session, admissible, random_target, scene, target_scene


class ProblemSession(Session):

    def __init__(self):
        self.rng = np.random.default_rng(1)
        self.plant = Plant(q=Q_NOMINAL)
        self._set_target(*target_from_grasp(Q_ARCH))

    def _set_target(self, nodes, points, s_star):
        self.target = target_scene(nodes, points)
        self.s_star = s_star
        self.initial_error = float(np.linalg.norm(s_star - self.plant.s))

    def state(self) -> dict:
        s = self.plant.s
        e = self.s_star - s
        return {"scene": scene(self.plant), "target": self.target,
                "s": s, "s_star": self.s_star, "e": e,
                "error_norm": float(np.linalg.norm(e)),
                "initial_error": self.initial_error}

    def act_move(self, q):
        """Move the grippers to q (a drag or a slider) and let the rod settle."""
        self.plant.move_to(admissible(q))
        return self.state()

    def act_new_target(self):
        _, nodes, points, s_star = random_target(self.rng, self.plant)
        self._set_target(nodes, points, s_star)
        return self.state()

    def act_reset(self):
        self.plant.reset(Q_NOMINAL)
        self._set_target(*target_from_grasp(Q_ARCH))
        return self.state()

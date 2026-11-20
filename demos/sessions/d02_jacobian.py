"""
Demo 02 - The deformation Jacobian.
===================================

At the current grasp, the exact Jacobian J = ds/dq (available in simulation
by finite differences of the equilibrium) predicts the feature change of a
small robot motion, ds ~ J dq. The page compares this prediction with the
rod's actual response, and shows how the agreement degrades with step size.
"""
from __future__ import annotations

import numpy as np

from shape_servoing import Plant, Q_ARCH, Q_NOMINAL, numerical_jacobian
from shape_servoing.failures import stretched_grasp

from .common import Session, admissible, scene

PRESETS = {
    "nominal": Q_NOMINAL,
    "slack": np.array([-2.3, 1.0, 0.0, 2.3, 1.0, 0.0]),
    "taut": stretched_grasp(6.6),
    "arch": Q_ARCH,
}
STEP_SCALES = np.linspace(0.1, 2.0, 12)


class JacobianSession(Session):

    def __init__(self):
        self.plant = Plant(q=Q_NOMINAL)
        self.dq = np.array([0.0, 0.5, 0.0, 0.0, 0.0, 0.0])
        self.J = numerical_jacobian(self.plant.peek, self.plant.q)

    def _response(self, dq):
        """Actual feature change of the motion dq from the current equilibrium."""
        nodes = self.plant.rod.equilibrium(self.plant.q + dq, initial=self.plant.nodes)
        return nodes, self.plant.features(nodes) - self.plant.s

    def state(self) -> dict:
        nodes_after, actual = self._response(self.dq)
        predicted = self.J @ self.dq
        steps, errors = [], []
        if np.linalg.norm(self.dq) > 1e-9:
            for beta in STEP_SCALES:
                ds = self._response(beta * self.dq)[1]
                steps.append(beta * np.linalg.norm(self.dq))
                errors.append(np.linalg.norm(ds - beta * predicted) / max(np.linalg.norm(ds), 1e-12))
        return {"scene": scene(self.plant), "J": self.J, "dq": self.dq,
                "predicted": predicted, "actual": actual,
                "after": {"nodes": nodes_after, "points": self.plant.points_of(nodes_after)},
                "relative_error": float(np.linalg.norm(actual - predicted) / max(np.linalg.norm(actual), 1e-12)),
                "curve": {"steps": steps, "errors": errors}}

    def act_move(self, q):
        self.plant.move_to(admissible(q))
        self.J = numerical_jacobian(self.plant.peek, self.plant.q)
        return self.state()

    def act_set_dq(self, dq):
        self.dq = np.asarray(dq, float)
        return self.state()

    def act_preset(self, name):
        self.plant.reset(PRESETS[name])
        self.J = numerical_jacobian(self.plant.peek, self.plant.q)
        return self.state()

    def act_play(self, frames: int = 48):
        """The robot oscillates along dq: q(t) = q + sin(2 pi t) dq, rod re-solved at each frame."""
        nodes, out = self.plant.nodes, []
        for t in np.linspace(0.0, 1.0, frames + 1):
            w = np.sin(2 * np.pi * t)
            nodes = self.plant.rod.equilibrium(self.plant.q + w * self.dq, initial=nodes)
            out.append({"q": self.plant.q + w * self.dq, "nodes": nodes, "points": self.plant.points_of(nodes),
                        "predicted": self.plant.points + w * (self.J @ self.dq).reshape(-1, 2)})
        return {"frames": out}

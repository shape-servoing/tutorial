"""
Demo 08 - Singular directions of the Jacobian.
==============================================

J_hat = Phi Sigma Psi^T: a unit robot velocity along psi_l produces the
feature velocity sigma_l phi_l. The page lists the singular triplets of the
offline estimate (or of the exact Jacobian), plays the robot motion along a
chosen psi_l on the rod, and shows how the singular values change with the
units of the robot coordinates and with the configuration.
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (Plant, Q_NOMINAL, condition_number, manipulability, numerical_jacobian,
                            numerical_rank, offline_estimate, probe_motions, singular_triplets)

from .common import Session, admissible, scene

DEGREES = np.array([1.0, 1.0, 180.0 / np.pi, 1.0, 1.0, 180.0 / np.pi])   # W_q for angles in degrees


class SvdSession(Session):

    def __init__(self):
        self.plant = Plant(q=Q_NOMINAL)
        self.source, self.units, self.selected = "estimate", "rad", 5
        self._configuration_changed()

    def _configuration_changed(self):
        if self.source == "estimate":
            self.J_rad, _, _ = offline_estimate(self.plant.peek, self.plant.q, probe_motions())
        else:
            self.J_rad = numerical_jacobian(self.plant.peek, self.plant.q)

    @property
    def W_q(self):
        return DEGREES if self.units == "deg" else np.ones(6)

    def state(self) -> dict:
        J = self.J_rad / self.W_q            # W_s J W_q^{-1} with W_s = I
        sigma, Phi, Psi = singular_triplets(J)
        return {"scene": scene(self.plant), "J": J, "sigma": sigma, "Phi": Phi, "Psi": Psi,
                "manipulability": manipulability(J), "condition": condition_number(J),
                "rank": numerical_rank(J), "selected": self.selected,
                "params": {"source": self.source, "units": self.units}}

    # ------------------------------------------------------------- actions
    def act_move(self, q):
        self.plant.move_to(admissible(q))
        self._configuration_changed()
        return self.state()

    def act_set(self, source=None, units=None, selected=None):
        if source is not None and source != self.source:
            self.source = source
            self._configuration_changed()
        self.units = self.units if units is None else units
        self.selected = self.selected if selected is None else int(selected)
        return self.state()

    def act_reset(self):
        """Back to the nominal grasp, the offline estimate, radians, and l = 6."""
        self.plant.reset(Q_NOMINAL)
        self.source, self.units, self.selected = "estimate", "rad", 5
        self._configuration_changed()
        return self.state()

    def act_play(self, index: int, frames: int = 48, amplitude: float = 0.3):
        """Robot oscillates along psi_l (in physical units); the rod is re-solved each frame."""
        sigma, Phi, Psi = singular_triplets(self.J_rad / self.W_q)
        direction = Psi[:, index] / self.W_q                 # physical motion for unit psi_l
        scale = amplitude / np.max(np.abs(direction))
        nodes, out = self.plant.nodes, []
        for t in np.linspace(0.0, 1.0, frames + 1):
            w = scale * np.sin(2 * np.pi * t)
            q = self.plant.q + w * direction
            nodes = self.plant.rod.equilibrium(q, initial=nodes)
            out.append({"q": q, "nodes": nodes, "points": self.plant.points_of(nodes),
                        "predicted": self.plant.points + w * sigma[index] * Phi[:, index].reshape(-1, 2)})
        return {"frames": out, "index": index}

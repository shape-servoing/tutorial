"""
Demo 12 - Shape control in 3-D.
================================

The same Jacobian/pseudoinverse loop controls a spatial rod, an ARAP surface
mesh, and an ARAP volumetric lattice.  In each case q has three generalized
actuation coordinates and s stacks selected 3-D vertex positions.
"""
from __future__ import annotations

import numpy as np

from shape_servoing.spatial import SpatialModel

from .common import Session


class SpatialSession(Session):

    def __init__(self):
        self.gain = 0.22
        self.severity = 1.0
        self._select("rod")

    def _select(self, kind: str):
        self.model = SpatialModel(kind)
        self.kind = kind
        self._restart()

    def _restart(self):
        self.q = np.zeros(3)
        self.k = 0
        self.vertices = self.model.deform(self.q)
        self.target_q = self.severity * self.model.spec.q_target
        self.target = self.model.deform(self.target_q)
        self.initial_error = max(self._error(), 1e-12)
        self.command_norm = 0.0
        self.command_vector = np.zeros(3)
        self.J = self.model.jacobian(self.q)
        self._update_jacobian_diagnostics()

    def _update_jacobian_diagnostics(self):
        self.singular_values = np.linalg.svd(self.J, compute_uv=False)
        threshold = max(self.J.shape) * np.finfo(float).eps * self.singular_values[0]
        self.rank = int(np.sum(self.singular_values > threshold))
        self.condition = float(self.singular_values[0] / max(self.singular_values[-1], 1e-12))

    def _error_vector(self):
        return self.model.features(self.target) - self.model.features(self.vertices)

    def _error(self) -> float:
        return float(np.linalg.norm(self._error_vector()))

    def state(self) -> dict:
        spec = self.model.spec
        return {
            "kind": self.kind,
            "label": spec.label,
            "deformation": spec.deformation,
            "vertices": self.vertices,
            "target": self.target,
            "rest": spec.rest,
            "faces": spec.faces,
            "edges": spec.display_edges,
            "model_edges": spec.model_edges,
            "fixed": spec.fixed,
            "driven": spec.driven,
            "fixed_anchor": spec.fixed_anchor,
            "driven_anchor": spec.driven_anchor,
            "grasp_radius": spec.grasp_radius,
            "features": spec.features,
            "q": self.q,
            "q_target": self.target_q,
            "q_labels": spec.q_labels,
            "jacobian": self.J,
            "singular_values": self.singular_values,
            "rank": self.rank,
            "condition": self.condition,
            "k": self.k,
            "error": self._error(),
            "initial_error": self.initial_error,
            "command": self.command_norm,
            "command_vector": self.command_vector,
            "params": {"gain": self.gain, "severity": self.severity},
        }

    def act_step(self, n: int = 1):
        samples = []
        for _ in range(min(3, max(1, int(n)))):
            error = self._error_vector()
            J = self.model.jacobian(self.q)
            command = self.gain * np.linalg.pinv(J, rcond=1e-4) @ error
            norm = np.linalg.norm(command)
            if norm > 0.30:
                command *= 0.30 / norm
            self.q += command
            self.vertices = self.model.deform(self.q)
            self.J = J
            self._update_jacobian_diagnostics()
            self.command_vector = command.copy()
            self.command_norm = float(np.linalg.norm(command))
            self.k += 1
            samples.append({
                "k": self.k,
                "error": self._error() / self.initial_error,
                "command": self.command_norm,
                "command_vector": self.command_vector,
                "q": self.q.copy(),
                "singular_values": self.singular_values.copy(),
                "condition": self.condition,
            })
        return {**self.state(), "samples": samples}

    def act_set_object(self, kind: str):
        self._select(kind)
        return self.state()

    def act_set_params(self, gain=None, severity=None):
        if gain is not None:
            self.gain = float(gain)
        if severity is not None and not np.isclose(float(severity), self.severity):
            self.severity = float(severity)
            self._restart()
        return self.state()

    def act_reset(self):
        self._restart()
        return self.state()

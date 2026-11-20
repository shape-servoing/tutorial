"""
Demo 06 - Range, null spaces, and residual error.
================================================

Two complementary local decompositions of the deformation Jacobian:

* point features (n > m): e = e_parallel + e_perp, with the reducible part in
  Range(J) and the local residual in Null(J^T);
* two tracked points (n < m): dq = dq_task + dq_null, with task motion in
  Range(J^T) and robot motion that preserves their positions in Null(J).
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (Plant, Q_NOMINAL, Q_REACHABLE, null_space,
                            numerical_jacobian, numerical_rank,
                            range_split, target_from_grasp, unfeasible_points)

from .common import Session, scene


class SubspacesSession(Session):

    def __init__(self):
        self.case = "error"
        self.params = {"range_weight": 1.0, "residual_weight": 1.0,
                       "task_weight": 0.75, "null_weight": 0.75}
        self._prepare_examples()

    def _prepare_examples(self):
        # Overdetermined point-coordinate example: construct the same synthetic
        # beyond-limit error used by the feasibility demo, then split it exactly.
        self.point_plant = Plant(q=Q_NOMINAL)
        self.J_points = numerical_jacobian(self.point_plant.peek, Q_NOMINAL)
        _, recorded, _ = target_from_grasp(Q_REACHABLE)
        raw_error = unfeasible_points(recorded).ravel() - self.point_plant.s
        self.error_range_base, self.error_orth_base = range_split(self.J_points, raw_error)

        # Underdetermined point-coordinate example: tracking two (x, y) points
        # gives a full-row-rank 4x6 Jacobian, hence two robot null directions.
        self.motion_plant = Plant(n_points=2, q=Q_NOMINAL)
        self.J_motion = numerical_jacobian(self.motion_plant.peek, Q_NOMINAL)
        _, _, vt = np.linalg.svd(self.J_motion, full_matrices=True)
        rank = numerical_rank(self.J_motion)
        self.task_direction = self._scaled_step(vt[0], 0.38)
        self.null_direction = self._scaled_step(null_space(self.J_motion)[:, 0], 0.25)
        assert rank == 4

    @staticmethod
    def _scaled_step(direction, amplitude):
        direction = np.asarray(direction, float)
        return amplitude * direction / max(np.max(np.abs(direction)), 1e-12)

    def _dimensions(self, J):
        n, m = J.shape
        rank = numerical_rank(J)
        return {"n": n, "m": m, "rank": rank, "range_dim": rank,
                "null_jt_dim": n - rank, "range_jt_dim": rank,
                "null_j_dim": m - rank}

    def _error_state(self):
        e_range = self.params["range_weight"] * self.error_range_base
        e_orth = self.params["residual_weight"] * self.error_orth_base
        e = e_range + e_orth
        target_points = self.point_plant.points + e.reshape(-1, 2)
        command = np.linalg.pinv(self.J_points) @ e
        response = self.J_points @ command
        norm = max(np.linalg.norm(e), 1e-12)
        return {
            "case": "error", "scene": scene(self.point_plant),
            "target": {"nodes": None, "points": target_points},
            "J": self.J_points, "e": e, "e_range": e_range, "e_orth": e_orth,
            "command": command, "response": response,
            "range_fraction": float(np.linalg.norm(e_range) / norm),
            "orth_fraction": float(np.linalg.norm(e_orth) / norm),
            "dimensions": self._dimensions(self.J_points), "params": self.params,
        }

    def _shape_at(self, plant, q):
        nodes = plant.rod.equilibrium(q, initial=plant.nodes)
        return {"q": q, "nodes": nodes, "points": plant.points_of(nodes)}

    def _motion_state(self):
        dq_task = self.params["task_weight"] * self.task_direction
        dq_null = self.params["null_weight"] * self.null_direction
        base = scene(self.motion_plant)
        task = self._shape_at(self.motion_plant, Q_NOMINAL + dq_task)
        null = self._shape_at(self.motion_plant, Q_NOMINAL + dq_null)
        combined = self._shape_at(self.motion_plant, Q_NOMINAL + dq_task + dq_null)
        s0 = self.motion_plant.s
        task_change = np.asarray(task["points"]).ravel() - s0
        null_change = np.asarray(null["points"]).ravel() - s0
        return {
            "case": "motion", "scene": base, "task_scene": task,
            "null_scene": null, "combined_scene": combined,
            "J": self.J_motion, "dq_task": dq_task, "dq_null": dq_null,
            "linear_task_change": self.J_motion @ dq_task,
            "linear_null_change": self.J_motion @ dq_null,
            "task_change": task_change, "null_change": null_change,
            "dimensions": self._dimensions(self.J_motion), "params": self.params,
        }

    def state(self):
        return self._error_state() if self.case == "error" else self._motion_state()

    def act_set_case(self, case: str):
        if case not in ("error", "motion"):
            raise ValueError(f"unknown subspace case: {case}")
        self.case = case
        return self.state()

    def act_set_params(self, **params):
        for key, value in params.items():
            if key in self.params:
                self.params[key] = float(value)
        return self.state()

    def act_reset(self):
        self.params.update({"range_weight": 1.0, "residual_weight": 1.0,
                            "task_weight": 0.75, "null_weight": 0.75})
        return self.state()

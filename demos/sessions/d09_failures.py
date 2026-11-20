"""
Demo 10 - Three ways the loop fails.
====================================

    controllability   pull the rod taut: sigma_min and w of the exact Jacobian
                      fall, and the pseudoinverse turns errors along the weak
                      direction into large commands (damped inverse: bounded)
    observability     a motion the features do not register gives the same
                      measured pair (dq != 0, ds ~ 0) as a lost direction
    instability       compress the rod (the straight state loses stability at
                      lambda_min(Pi_xx) = 0), or rotate the wrists of an arch
                      until it snaps through: small dq, large ds
"""
from __future__ import annotations

import numpy as np

from shape_servoing import Plant, numerical_jacobian, singular_triplets
from shape_servoing.failures import (buckling_onset, response_ratio, straight_rod_stiffness,
                                     stretched_grasp, taut_sweep)

from .common import Session, scene

REST = 6.0


def arch_grasp(theta):
    """Grippers 4.8 apart, tangents tilted by +/- theta (an arch for theta > 0)."""
    return np.array([-2.4, 1.0, theta, 2.4, 1.0, -theta])


class FailuresSession(Session):

    def __init__(self):
        extensions = np.linspace(0.0, 20.0, 25)
        self.sweep = {"extension": extensions,
                      **taut_sweep(Plant(q=stretched_grasp(REST)), REST * (1 + extensions / 100))}
        compressions = np.linspace(-10.0, 20.0, 31)
        self.taut = Plant(q=stretched_grasp(REST))
        self.buckling = {"compression": compressions,
                         "lambda": [straight_rod_stiffness(self.taut.rod, REST * (1 - c / 100)) for c in compressions],
                         "onset": 100 * (REST - buckling_onset(self.taut.rod, REST * (1 - compressions / 100))) / REST}
        self.mu = 0.05
        self.extension = 0.0
        self._reset_compression()
        self._reset_snap()

    def state(self) -> dict:
        return {"taut": self._taut_state(), "sweep": self.sweep, "buckling": self.buckling,
                "compress": self._compress_state(), "snap": self._snap_state()}

    # ------------------------------------------------------------- controllability
    def _taut_state(self) -> dict:
        J = numerical_jacobian(self.taut.peek, self.taut.q)
        sigma, Phi, Psi = singular_triplets(J)
        psi_min = Psi[:, -1]
        return {"scene": scene(self.taut), "extension": self.extension, "sigma": sigma,
                "psi_min": psi_min * np.sign(psi_min[2] or 1.0), "phi_min": Phi[:, -1],
                "wrist_share": float(psi_min[2] ** 2 + psi_min[5] ** 2), "mu": self.mu,
                "pinv_gain": 1.0 / sigma, "damped_gain": sigma / (sigma ** 2 + self.mu ** 2)}

    def act_taut(self, extension):
        self.extension = float(extension)
        self.taut.move_to(stretched_grasp(REST * (1 + self.extension / 100)))
        return {"taut": self._taut_state()}

    def act_set_mu(self, mu):
        self.mu = float(mu)
        return {"taut": self._taut_state()}

    # ------------------------------------------------------------- observability
    def act_observe(self, scenario: str, frames: int = 40):
        """Robot motion and measured response for three cases with the same question:
        does ds follow dq?"""
        if scenario == "controllability":       # taut rod, motion along the weakest direction
            plant = Plant(q=stretched_grasp(REST * 1.2))
            J = numerical_jacobian(plant.peek, plant.q)
            direction = singular_triplets(J)[2][:, -1]
        elif scenario == "observability":       # rigid translation, turning-angle features
            plant = Plant(feature_map="angles", q=stretched_grasp(5.4))
            direction = np.array([1.0, 0.6, 0.0, 1.0, 0.6, 0.0])
        else:                                   # a healthy case: lift the left gripper
            plant = Plant(q=stretched_grasp(REST))
            direction = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
        direction = 0.5 * direction / np.max(np.abs(direction))
        s0, q0, nodes, out = plant.s, plant.q.copy(), plant.nodes, []
        for t in np.linspace(0.0, 1.0, frames + 1):
            q = q0 + np.sin(2 * np.pi * t) * direction
            nodes = plant.rod.equilibrium(q, initial=nodes)
            ds = plant.features(nodes) - s0
            out.append({"q": q, "nodes": nodes, "points": plant.points_of(nodes),
                        "dq": float(np.linalg.norm(q - q0)), "ds": float(np.linalg.norm(ds))})
        peak = out[frames // 4]
        return {"scenario": scenario, "feature_map": plant.feature_map, "frames": out,
                "ratio": peak["ds"] / max(peak["dq"], 1e-12)}

    # ------------------------------------------------------------- instability
    def _reset_compression(self):
        self.compress = Plant(q=stretched_grasp(REST * 1.05))
        self.compression, self.compress_ratio = -5.0, None

    def _compress_state(self) -> dict:
        q = self.compress.q
        straight = self.compress.rod.straight_nodes(q)
        return {"scene": scene(self.compress), "straight": straight, "compression": self.compression,
                "lambda_straight": straight_rod_stiffness(self.compress.rod, REST * (1 - self.compression / 100)),
                "lambda_equilibrium": float(np.linalg.eigvalsh(self.compress.rod.free_hessian(self.compress.nodes))[0]),
                "ratio": self.compress_ratio}

    def act_compress(self, compression):
        s_before, q_before = self.compress.s, self.compress.q.copy()
        self.compression = float(compression)
        s = self.compress.move_to(stretched_grasp(REST * (1 - self.compression / 100)))
        if np.linalg.norm(self.compress.q - q_before) > 1e-9:
            self.compress_ratio = response_ratio(self.compress.q - q_before, s - s_before)
        return {"compress": self._compress_state()}

    def _reset_snap(self):
        self.snap = Plant(q=arch_grasp(0.8))
        self.snap_path = {"theta": [0.8], "lambda": [self._snap_lambda()], "ratio": [None]}

    def _snap_lambda(self) -> float:
        return float(np.linalg.eigvalsh(self.snap.rod.free_hessian(self.snap.nodes))[0])

    def _snap_state(self) -> dict:
        return {"scene": scene(self.snap), "path": self.snap_path}

    def act_wrist(self, theta):
        s_before, q_before = self.snap.s, self.snap.q.copy()
        s = self.snap.move_to(arch_grasp(float(theta)))
        if np.linalg.norm(self.snap.q - q_before) > 1e-9:
            self.snap_path["theta"].append(float(theta))
            self.snap_path["lambda"].append(self._snap_lambda())
            self.snap_path["ratio"].append(response_ratio(self.snap.q - q_before, s - s_before))
        return {"snap": self._snap_state()}

    def act_reset_instability(self):
        self._reset_compression()
        self._reset_snap()
        return {"compress": self._compress_state(), "snap": self._snap_state()}

"""
Three ways the loop fails.
==========================

The loop relies on J = (ds/dx)(dx/dq) and on a well-defined equilibrium map:

    loss of controllability   dx/dq loses rank: sigma_min -> 0, w -> 0
    loss of observability     ds/dx does not register a motion of the object
    mechanical instability    lambda_min(Pi_xx) -> 0: the equilibrium loses
                              stiffness along one deformation mode (buckling)

Three quantities that the loop already computes can indicate these failures:
the singular values of J_hat, the prediction error r_k, and the response ratio
||ds_k|| / ||dq_{k-1}||.
"""
from __future__ import annotations

import numpy as np

from .analysis import condition_number, manipulability, singular_triplets
from .jacobian import numerical_jacobian
from .rod import Rod


def stretched_grasp(separation: float, height: float = 1.0) -> np.ndarray:
    """Horizontal grasp with the grippers `separation` apart and tangents aligned."""
    return np.array([-separation / 2, height, 0.0, separation / 2, height, 0.0])


def taut_sweep(plant, separations):
    """Exact Jacobian diagnostics as the grippers pull the rod apart.

    Returns sigma_min, w, cond, and the wrist share of the weakest robot
    direction psi_min (the fraction of its squared norm on theta_L and theta_R).
    """
    out = {"sigma_min": [], "manipulability": [], "condition": [], "wrist_share": []}
    for separation in separations:
        q = stretched_grasp(separation)
        plant.move_to(q)
        J = numerical_jacobian(plant.peek, q)
        sigma, _, Psi = singular_triplets(J)
        out["sigma_min"].append(sigma[-1])
        out["manipulability"].append(manipulability(J))
        out["condition"].append(condition_number(J))
        out["wrist_share"].append(Psi[2, -1] ** 2 + Psi[5, -1] ** 2)
    return {key: np.array(value) for key, value in out.items()}


def straight_rod_stiffness(rod: Rod, separation: float, height: float = 1.0) -> float:
    """Smallest eigenvalue of the energy Hessian Pi_xx on the straight rod."""
    nodes = rod.straight_nodes(stretched_grasp(separation, height))
    return float(np.linalg.eigvalsh(rod.free_hessian(nodes))[0])


def buckling_onset(rod: Rod, separations) -> float:
    """Separation at which lambda_min(Pi_xx) of the straight rod crosses zero."""
    separations = np.asarray(separations, float)
    lam = np.array([straight_rod_stiffness(rod, d) for d in separations])
    order = np.argsort(lam)
    return float(np.interp(0.0, lam[order], separations[order]))


def response_ratio(dq, ds) -> float:
    """||ds|| / ||dq||: near zero for a lost direction, large at an instability."""
    return float(np.linalg.norm(ds) / max(np.linalg.norm(dq), 1e-12))

"""
The deformation Jacobian J = ds/dq and its estimation.
======================================================

    numerical_jacobian   J(q) by central differences of any map q -> s. Applied
                         to the simulated rod, it gives the exact Jacobian,
                         which is unavailable on a real system; applied to a
                         mechanical model g, it gives the model-based estimate.
    offline_estimate     regression over exploratory motions,
                         J0 = argmin ||dS - J dQ||_F^2 = dS dQ^+.
    broyden_update       online secant correction from the latest
                         action-response pair (dq_{k-1}, ds_k).
"""
from __future__ import annotations

import numpy as np


def numerical_jacobian(measure, q, step: float = 1e-3) -> np.ndarray:
    """Column j is the feature velocity produced by a unit velocity of q_j."""
    q = np.asarray(q, float)
    columns = []
    for j in range(q.size):
        dq = np.zeros_like(q)
        dq[j] = step
        columns.append((measure(q + dq) - measure(q - dq)) / (2 * step))
    return np.column_stack(columns)


def offline_estimate(measure, q0, dQ):
    """Least-squares Jacobian from the exploratory actions dQ (m x C).

    Returns J0, the responses dS (n x C) with ds_c = s(q0 + dq_c) - s(q0),
    and the regression residual ||dS - J0 dQ||_F.
    """
    q0 = np.asarray(q0, float)
    s0 = measure(q0)
    dS = np.column_stack([measure(q0 + dq) - s0 for dq in np.asarray(dQ, float).T])
    J0 = dS @ np.linalg.pinv(dQ)
    return J0, dS, float(np.linalg.norm(dS - J0 @ dQ))


def broyden_update(J_hat, dq, ds, relaxation: float = 1.0):
    """Rank-one correction enforcing the secant relation J_k dq = ds.

    Returns the updated estimate and the prediction error r = ds - J_hat dq.
    The correction changes the estimate only along dq. `relaxation` < 1 applies
    only part of the correction (1 is Broyden's rule as in the tutorial; the
    feature-comparison runs of Section VI use 0.4).
    """
    dq = np.asarray(dq, float)
    r = np.asarray(ds, float) - J_hat @ dq
    return J_hat + relaxation * np.outer(r, dq) / (dq @ dq), r

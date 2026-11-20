"""
Subspaces, singular directions, and local stability of the Jacobian.
====================================================================

    J = Phi Sigma Psi^T = sum_l sigma_l phi_l psi_l^T     (singular value decomposition)
    J^+ = sum_{sigma_l > 0} (1 / sigma_l) psi_l phi_l^T   (pseudoinverse, gain by gain)
    w = prod_l sigma_l,   cond = sigma_max / sigma_min    (responsiveness, anisotropy)

    dL/dt = -alpha e^T J J_hat^+ e                        (Lyapunov rate, L = ||e||^2 / 2)
    e^T J J_hat^+ e >= lambda ||e||^2 on Range(J)         (local stability condition)
"""
from __future__ import annotations

import numpy as np


# --------------------------------------------------------------------------- #
#  Singular directions and gains
# --------------------------------------------------------------------------- #
def singular_triplets(J):
    """Gains sigma_l (descending), feature directions phi_l (columns of Phi),
    and robot directions psi_l (columns of Psi)."""
    Phi, sigma, Psi_T = np.linalg.svd(np.asarray(J, float), full_matrices=False)
    return sigma, Phi, Psi_T.T


def manipulability(J) -> float:
    """w = product of the singular values."""
    return float(np.prod(np.linalg.svd(np.asarray(J, float), compute_uv=False)))


def condition_number(J) -> float:
    """cond = sigma_max / sigma_min (infinite if J is rank deficient)."""
    sigma = np.linalg.svd(np.asarray(J, float), compute_uv=False)
    return float(sigma[0] / sigma[-1]) if sigma[-1] > 0 else np.inf


def numerical_rank(J, rtol: float = 1e-3) -> int:
    """Number of singular values above rtol * sigma_max."""
    sigma = np.linalg.svd(np.asarray(J, float), compute_uv=False)
    return int(np.sum(sigma > rtol * sigma[0]))


def null_space(J, rtol: float = 1e-3) -> np.ndarray:
    """Orthonormal basis (columns) of the robot velocities that J maps to ~zero."""
    J = np.asarray(J, float)
    _, sigma, Psi_T = np.linalg.svd(J)
    rank = int(np.sum(sigma > rtol * sigma[0]))
    return Psi_T[rank:].T


# --------------------------------------------------------------------------- #
#  The two subspaces of the control law
# --------------------------------------------------------------------------- #
def range_split(J, e):
    """Split e into J J^+ e, its component in Range(J), and the orthogonal component."""
    e = np.asarray(e, float)
    e_range = J @ np.linalg.pinv(J) @ e
    return e_range, e - e_range


def null_space_projector(J) -> np.ndarray:
    """Orthogonal projector I - J^+ J onto Null(J)."""
    J = np.asarray(J, float)
    return np.eye(J.shape[1]) - np.linalg.pinv(J) @ J


def damped_inverse(J, mu: float) -> np.ndarray:
    """Damped least-squares inverse J^T (J J^T + mu^2 I)^{-1}; gains capped at 1/(2 mu)."""
    J = np.asarray(J, float)
    return J.T @ np.linalg.inv(J @ J.T + mu ** 2 * np.eye(J.shape[0]))


def scaled(J, feature_scale, robot_scale) -> np.ndarray:
    """W_s J W_q^{-1} for diagonal scalings of the features and robot coordinates."""
    return np.diag(feature_scale) @ np.asarray(J, float) @ np.diag(1.0 / np.asarray(robot_scale, float))


# --------------------------------------------------------------------------- #
#  Local stability
# --------------------------------------------------------------------------- #
def lyapunov_rate(J, J_hat, e, alpha: float) -> float:
    """dL/dt = -alpha e^T J J_hat^+ e for L = ||e||^2 / 2."""
    e = np.asarray(e, float)
    return float(-alpha * e @ J @ np.linalg.pinv(J_hat) @ e)


def descent_margin(J, J_hat, rtol: float = 1e-6):
    """Smallest restricted eigenvalue in the paper's stability condition.

    For n <= m, this is Sym(J J_hat^+) on Range(J).  For n > m, it is
    Sym(J_hat^+ J) on Range(J^T).  A positive result is the corresponding
    sufficient local condition for all controllable modes.
    """
    J = np.asarray(J, float)
    sigma, Phi, Psi = singular_triplets(J)
    keep = sigma > rtol * sigma[0]
    if J.shape[0] <= J.shape[1]:
        basis = Phi[:, keep]
        M = J @ np.linalg.pinv(J_hat)
    else:
        basis = Psi[:, keep]
        M = np.linalg.pinv(J_hat) @ J
    eigenvalues = np.linalg.eigvalsh(basis.T @ (0.5 * (M + M.T)) @ basis)
    return float(eigenvalues[0]), eigenvalues


def descent_angles(J, J_hat, errors) -> np.ndarray:
    """Angle [rad] between each error e (columns) and the feature velocity
    J J_hat^+ e that the command produces on the object. Below pi/2: descent."""
    errors = np.asarray(errors, float)
    velocity = J @ np.linalg.pinv(J_hat) @ errors
    cosine = np.sum(errors * velocity, axis=0) / (
        np.linalg.norm(errors, axis=0) * np.linalg.norm(velocity, axis=0) + 1e-300)
    return np.arccos(np.clip(cosine, -1.0, 1.0))

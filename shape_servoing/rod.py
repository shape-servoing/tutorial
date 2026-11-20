"""
The elastic rod of the running example.
=======================================

A planar Kirchhoff rod with stretching and bending stiffness, discretized
with 25 nodes and subject to gravity. Each gripper clamps one end node and the
tangent of the adjacent edge, so the robot coordinates

    q = [x_L, y_L, theta_L, x_R, y_R, theta_R]

fix four nodes. For each q, the quasistatic equilibrium is the configuration
x of the remaining (free) nodes that minimizes the total potential energy
Pi(x; q). The controller does not use this model: it only commands q and
measures the resulting features.

Newton's method with the analytic Hessian finds each equilibrium in a few
iterations, which allows the demos to re-solve the rod at every frame.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

CLAMPED = np.array([0, 1, -2, -1])          # node indices fixed by the grippers
_ROT90 = np.array([[0.0, -1.0], [1.0, 0.0]])  # perp(v) = _ROT90 @ v


def _perp(v):
    """Rotate 2-vectors by +90 degrees: (x, y) -> (-y, x)."""
    return np.stack([-v[..., 1], v[..., 0]], axis=-1)


def _turning_angles(a, b):
    """Signed angle from edge a to edge b, wrapped to (-pi, pi]."""
    cross = a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]
    return np.arctan2(cross, np.sum(a * b, axis=-1))


@dataclass(frozen=True)
class Rod:
    """Discrete planar rod: rest length, stiffnesses, and gravity load."""
    n_nodes: int = 25
    length: float = 6.0
    k_stretch: float = 80.0
    k_bend: float = 4.0
    gravity: float = 0.45

    @property
    def rest_edge(self) -> float:
        return self.length / (self.n_nodes - 1)

    # ------------------------------------------------------------------ #
    #  Boundary conditions
    # ------------------------------------------------------------------ #
    def clamped_nodes(self, q) -> np.ndarray:
        """Positions of the four nodes fixed by the grasp q, shape (4, 2)."""
        xl, yl, tl, xr, yr, tr = np.asarray(q, float)
        h = self.rest_edge
        left, right = np.array([xl, yl]), np.array([xr, yr])
        return np.array([left, left + h * np.array([np.cos(tl), np.sin(tl)]),
                         right - h * np.array([np.cos(tr), np.sin(tr)]), right])

    def straight_nodes(self, q) -> np.ndarray:
        """Free nodes evenly spaced on the chord between the inner clamps:
        the straight configuration whose stability decides buckling."""
        c = self.clamped_nodes(q)
        t = np.linspace(0.0, 1.0, self.n_nodes - 2)[:, None]
        nodes = np.empty((self.n_nodes, 2))
        nodes[1:-1] = (1 - t) * c[1] + t * c[2]
        nodes[CLAMPED] = c
        return nodes

    def tangent_curve_nodes(self, q) -> np.ndarray:
        """Starting guess of a solve without history.

        A cubic Hermite curve leaves each inner clamp along the clamped
        tangent; a bump along its own bulge (or along gravity, if it is
        straight) lengthens it to the rod's rest length. A slack rod thus
        starts on the branch its grasp suggests, e.g. an arch for tangents
        tilted up and inward, rather than on an arbitrary one.
        """
        c = self.clamped_nodes(q)
        h = self.rest_edge
        tangent_l, tangent_r = (c[1] - c[0]) / h, (c[3] - c[2]) / h
        chord = max(np.linalg.norm(c[2] - c[1]), h)
        free_length = self.length - 2 * h
        t = np.linspace(0.0, 1.0, 241)[:, None]
        hermite = ((2 * t**3 - 3 * t**2 + 1) * c[1] + (t**3 - 2 * t**2 + t) * chord * tangent_l
                   + (-2 * t**3 + 3 * t**2) * c[2] + (t**3 - t**2) * chord * tangent_r)
        bulge = hermite[120] - 0.5 * (c[1] + c[2])
        direction = (bulge / np.linalg.norm(bulge) if np.linalg.norm(bulge) > 1e-3 * chord
                     else np.array([0.0, -1.0]))
        bump = np.sin(np.pi * t) * direction

        def arclength(points):
            return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]

        lo, hi = 0.0, free_length
        if arclength(hermite)[-1] < free_length:
            for _ in range(40):                      # bisection on the bump height
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if arclength(hermite + mid * bump)[-1] < free_length else (lo, mid)
        points = hermite + lo * bump
        arc = arclength(points)
        s = np.linspace(0.0, arc[-1], self.n_nodes - 2)
        nodes = np.empty((self.n_nodes, 2))
        nodes[1:-1] = np.column_stack([np.interp(s, arc, points[:, 0]),
                                       np.interp(s, arc, points[:, 1])])
        nodes[CLAMPED] = c
        return nodes

    # ------------------------------------------------------------------ #
    #  Deformation imposed on the rod (the cost compared in Section VI)
    # ------------------------------------------------------------------ #
    def axial_strain(self, nodes) -> np.ndarray:
        """Axial strain of every edge, |e_j| / h - 1 (positive when stretched)."""
        return np.linalg.norm(np.diff(nodes, axis=0), axis=1) / self.rest_edge - 1.0

    def elastic_energy(self, nodes) -> float:
        """Stretching plus bending energy: the potential energy without gravity."""
        h = self.rest_edge
        e = np.diff(nodes, axis=0)
        kappa = _turning_angles(e[:-1], e[1:])
        return float(0.5 * self.k_stretch * h * np.sum(self.axial_strain(nodes) ** 2)
                     + 0.5 * self.k_bend / h * np.sum(kappa ** 2))

    # ------------------------------------------------------------------ #
    #  Total potential energy Pi, its gradient, and its Hessian
    # ------------------------------------------------------------------ #
    def energy(self, nodes) -> float:
        h = self.rest_edge
        e = np.diff(nodes, axis=0)
        stretch = np.linalg.norm(e, axis=1) - h
        kappa = _turning_angles(e[:-1], e[1:])
        return (0.5 * self.k_stretch / h * np.sum(stretch ** 2)
                + 0.5 * self.k_bend / h * np.sum(kappa ** 2)
                + self.gravity * h * np.sum(nodes[:, 1]))

    def gradient(self, nodes) -> np.ndarray:
        """dPi/dnodes, shape (n_nodes, 2)."""
        h = self.rest_edge
        e = np.diff(nodes, axis=0)
        length = np.linalg.norm(e, axis=1)
        grad = np.zeros_like(nodes)

        f = (self.k_stretch / h) * ((length - h) / length)[:, None] * e
        grad[:-1] -= f
        grad[1:] += f

        a, b = e[:-1], e[1:]
        c = (self.k_bend / h) * _turning_angles(a, b)[:, None]
        dk_da = -_perp(a) / np.sum(a * a, axis=1)[:, None]
        dk_db = _perp(b) / np.sum(b * b, axis=1)[:, None]
        grad[:-2] -= c * dk_da
        grad[1:-1] += c * (dk_da - dk_db)
        grad[2:] += c * dk_db

        grad[:, 1] += self.gravity * h
        return grad

    def hessian(self, nodes) -> np.ndarray:
        """d^2 Pi / dnodes^2 over all 2 n_nodes coordinates (dense)."""
        h, n = self.rest_edge, self.n_nodes
        e = np.diff(nodes, axis=0)
        H = np.zeros((2 * n, 2 * n))

        # Stretch: one 2x2 block per edge, coupling its two nodes.
        length = np.linalg.norm(e, axis=1)
        t = e / length[:, None]
        Hs = (self.k_stretch / h) * (
            (1 - h / length)[:, None, None] * np.eye(2)
            + (h / length)[:, None, None] * t[:, :, None] * t[:, None, :])
        Hs_nodes = np.block([[Hs, -Hs], [-Hs, Hs]])               # (n-1, 4, 4)
        _scatter(H, Hs_nodes, np.arange(n - 1)[:, None] + np.arange(2))

        # Bend: kappa_j depends on a = e_{j-1}, b = e_j, i.e. on three nodes.
        a, b = e[:-1], e[1:]
        kappa = _turning_angles(a, b)
        a2, b2 = np.sum(a * a, axis=1), np.sum(b * b, axis=1)
        g = np.concatenate([-_perp(a) / a2[:, None], _perp(b) / b2[:, None]], axis=1)

        def angle_hessian(v, v2):    # d/dv [perp(v) / |v|^2], symmetric
            return (_ROT90 / v2[:, None, None]
                    - 2 * _perp(v)[:, :, None] * v[:, None, :] / (v2 ** 2)[:, None, None])

        Hab = g[:, :, None] * g[:, None, :]
        Hab[:, :2, :2] -= kappa[:, None, None] * angle_hessian(a, a2)
        Hab[:, 2:, 2:] += kappa[:, None, None] * angle_hessian(b, b2)
        B = np.kron(np.array([[-1.0, 1.0, 0.0], [0.0, -1.0, 1.0]]), np.eye(2))
        Hb_nodes = (self.k_bend / h) * np.einsum("ia,kab,bj->kij", B.T, Hab, B)
        _scatter(H, Hb_nodes, np.arange(n - 2)[:, None] + np.arange(3))
        return H

    # ------------------------------------------------------------------ #
    #  Quasistatic equilibrium
    # ------------------------------------------------------------------ #
    @property
    def free_coordinates(self) -> np.ndarray:
        """Indices of the free node coordinates in the flattened node array."""
        return np.arange(4, 2 * self.n_nodes - 4)

    def free_hessian(self, nodes) -> np.ndarray:
        """Hessian Pi_xx of the energy with respect to the free coordinates x."""
        free = self.free_coordinates
        return self.hessian(nodes)[np.ix_(free, free)]

    def equilibrium(self, q, initial=None, tol: float = 1e-10,
                    max_iterations: int = 200) -> np.ndarray:
        """Equilibrium nodes (n_nodes, 2) for grasp q.

        `initial` is the previous equilibrium when the object moves
        continuously (the physical history that selects the equilibrium
        branch); without it, the solve starts from `tangent_curve_nodes`.
        """
        nodes = self.tangent_curve_nodes(q) if initial is None else np.array(initial, float)
        nodes[CLAMPED] = self.clamped_nodes(q)
        free = self.free_coordinates
        for _ in range(max_iterations):
            g = self.gradient(nodes).ravel()[free]
            if np.max(np.abs(g)) < tol:
                break
            step = _descent_step(self.free_hessian(nodes), g)
            nodes = self._line_search(nodes, step, g)
        return nodes

    def _line_search(self, nodes, step, g, max_move: float = 0.5):
        """Backtracking on the energy, with each node moving at most max_move."""
        free = self.free_coordinates
        step = step * min(1.0, max_move / max(np.max(np.abs(step)), 1e-12))
        energy0, slope = self.energy(nodes), float(g @ step)
        t = 1.0
        for _ in range(40):
            trial = nodes.copy()
            trial.ravel()[free] += t * step
            # The tolerance term lets the final Newton steps through when
            # the energy change is below floating-point resolution.
            if self.energy(trial) <= energy0 + 1e-4 * t * slope + 1e-13 * abs(energy0):
                return trial
            t *= 0.5
        return trial

    # ------------------------------------------------------------------ #
    #  Perception: the tracked points
    # ------------------------------------------------------------------ #
    @staticmethod
    def sample_points(nodes, n_points: int = 5) -> np.ndarray:
        """P tracked points, evenly spaced in arclength from 12% to 88% of the rod."""
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(nodes, axis=0), axis=1))]
        s = np.linspace(0.12, 0.88, n_points) * arc[-1]
        return np.column_stack([np.interp(s, arc, nodes[:, 0]),
                                np.interp(s, arc, nodes[:, 1])])


def _scatter(H, blocks, element_nodes):
    """Add per-element node blocks into the global Hessian."""
    dofs = (2 * element_nodes[:, :, None] + np.arange(2)).reshape(len(blocks), -1)
    np.add.at(H, (dofs[:, :, None], dofs[:, None, :]), blocks)


def _descent_step(H, g):
    """Newton step, shifted toward gradient descent where H is not positive definite."""
    shift = 0.0
    scale = max(np.max(np.abs(np.diag(H))), 1.0)
    for _ in range(30):
        try:
            L = np.linalg.cholesky(H + shift * np.eye(len(g)))
            return -np.linalg.solve(L.T, np.linalg.solve(L, g))
        except np.linalg.LinAlgError:
            shift = max(10 * shift, 1e-6 * scale)
    return -g / scale

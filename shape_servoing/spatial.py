"""
Small 3-D deformation models used by the spatial shape-control demo.
===================================================================

The spatial rod uses a cubic centerline between two finite grasp regions.  The
sheet and solid use the local-global as-rigid-as-possible (ARAP) algorithm with
uniform edge weights and local positional grasp constraints.  The solid is a
volumetric lattice; its exterior is triangulated only for rendering.

These models are intentionally compact rather than general-purpose geometry
libraries.  They expose the same map used throughout the tutorial,

    q -> x(q) -> s(q),

so the demo can form a numerical deformation Jacobian and close the loop.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SpatialSpec:
    kind: str
    label: str
    deformation: str
    rest: np.ndarray
    model_edges: np.ndarray
    display_edges: np.ndarray
    faces: np.ndarray
    fixed: np.ndarray
    driven: np.ndarray
    fixed_anchor: int
    driven_anchor: int
    grasp_radius: int
    features: np.ndarray
    q_target: np.ndarray
    q_labels: tuple[str, str, str]


def _unique_edges(faces) -> np.ndarray:
    edges = {tuple(sorted((int(a), int(b))))
             for face in faces for a, b in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0]))}
    return np.asarray(sorted(edges), dtype=int)


def topological_ball(edges, seed: int, radius: int) -> np.ndarray:
    """Vertices within the given graph-edge radius of a grasp anchor."""
    adjacency: dict[int, set[int]] = {}
    for a, b in np.asarray(edges, int):
        adjacency.setdefault(int(a), set()).add(int(b))
        adjacency.setdefault(int(b), set()).add(int(a))
    reached, frontier = {int(seed)}, {int(seed)}
    for _ in range(max(0, int(radius))):
        frontier = {j for i in frontier for j in adjacency.get(i, ())} - reached
        reached.update(frontier)
    return np.asarray(sorted(reached), dtype=int)


def _rod_spec(grasp_radius: int) -> SpatialSpec:
    t = np.linspace(0.0, 1.0, 17)
    rest = np.column_stack((6.0 * (t - 0.5), np.zeros_like(t), np.zeros_like(t)))
    edges = np.column_stack((np.arange(16), np.arange(1, 17)))
    return SpatialSpec(
        kind="rod", label="Spatial rod", deformation="grasped cubic centerline",
        rest=rest, model_edges=edges, display_edges=edges, faces=np.empty((0, 3), int),
        fixed=topological_ball(edges, 0, grasp_radius),
        driven=topological_ball(edges, 16, grasp_radius),
        fixed_anchor=0, driven_anchor=16, grasp_radius=grasp_radius,
        features=np.arange(2, 15, 2), q_target=np.array([1.65, 1.35, 0.72]),
        q_labels=("right lift", "right depth", "tip angle"),
    )


def _sheet_spec(grasp_radius: int, nx: int = 7, ny: int = 5) -> SpatialSpec:
    xs, ys = np.linspace(-3.0, 3.0, nx), np.linspace(-1.7, 1.7, ny)
    rest = np.array([[x, y, 0.0] for y in ys for x in xs], float)
    faces = []
    for j in range(ny - 1):
        for i in range(nx - 1):
            a, b, c, d = j * nx + i, j * nx + i + 1, (j + 1) * nx + i, (j + 1) * nx + i + 1
            faces.extend(((a, b, d), (a, d, c)) if (i + j) % 2 == 0 else ((a, b, c), (b, d, c)))
    faces = np.asarray(faces, int)
    fixed_anchor = (ny // 2) * nx
    driven_anchor = (ny // 2) * nx + nx - 1
    features = np.array([j * nx + i for j in (0, 2, 4) for i in (1, 3, 5)], int)
    edges = _unique_edges(faces)
    return SpatialSpec(
        kind="sheet", label="Planar sheet", deformation="triangular mesh · ARAP",
        rest=rest, model_edges=edges, display_edges=edges, faces=faces,
        fixed=topological_ball(edges, fixed_anchor, grasp_radius),
        driven=topological_ball(edges, driven_anchor, grasp_radius),
        fixed_anchor=fixed_anchor, driven_anchor=driven_anchor, grasp_radius=grasp_radius,
        features=features,
        q_target=np.array([-0.90, 0.75, 1.65]),
        q_labels=("right pull", "right lift", "edge twist"),
    )


def _grid_index(i: int, j: int, k: int, nx: int, ny: int) -> int:
    return k * nx * ny + j * nx + i


def _solid_spec(grasp_radius: int, nx: int = 6, ny: int = 5, nz: int = 5) -> SpatialSpec:
    xs, ys, zs = np.linspace(-2.4, 2.4, nx), np.linspace(-1.3, 1.3, ny), np.linspace(-1.1, 1.1, nz)
    rest = np.array([[x, y, z] for z in zs for y in ys for x in xs], float)

    # Tetrahedral-lattice connectivity: axes, face diagonals, and one body
    # diagonal per grid cell.  ARAP acts on all vertices, including the interior.
    offsets = ((1, 0, 0), (0, 1, 0), (0, 0, 1), (1, 1, 0),
               (1, 0, 1), (0, 1, 1), (1, 1, 1))
    model_edges = []
    for k in range(nz):
        for j in range(ny):
            for i in range(nx):
                a = _grid_index(i, j, k, nx, ny)
                for di, dj, dk in offsets:
                    if i + di < nx and j + dj < ny and k + dk < nz:
                        model_edges.append((a, _grid_index(i + di, j + dj, k + dk, nx, ny)))
    model_edges = np.asarray(model_edges, int)

    faces = []
    def quad(a, b, c, d, flip=False):
        faces.extend(((a, b, d), (a, d, c)) if not flip else ((a, d, b), (a, c, d)))
    for k in range(nz - 1):
        for j in range(ny - 1):
            quad(_grid_index(0, j, k, nx, ny), _grid_index(0, j + 1, k, nx, ny),
                 _grid_index(0, j, k + 1, nx, ny), _grid_index(0, j + 1, k + 1, nx, ny), True)
            quad(_grid_index(nx - 1, j, k, nx, ny), _grid_index(nx - 1, j + 1, k, nx, ny),
                 _grid_index(nx - 1, j, k + 1, nx, ny), _grid_index(nx - 1, j + 1, k + 1, nx, ny))
    for k in range(nz - 1):
        for i in range(nx - 1):
            quad(_grid_index(i, 0, k, nx, ny), _grid_index(i + 1, 0, k, nx, ny),
                 _grid_index(i, 0, k + 1, nx, ny), _grid_index(i + 1, 0, k + 1, nx, ny))
            quad(_grid_index(i, ny - 1, k, nx, ny), _grid_index(i + 1, ny - 1, k, nx, ny),
                 _grid_index(i, ny - 1, k + 1, nx, ny), _grid_index(i + 1, ny - 1, k + 1, nx, ny), True)
    for j in range(ny - 1):
        for i in range(nx - 1):
            quad(_grid_index(i, j, 0, nx, ny), _grid_index(i + 1, j, 0, nx, ny),
                 _grid_index(i, j + 1, 0, nx, ny), _grid_index(i + 1, j + 1, 0, nx, ny), True)
            quad(_grid_index(i, j, nz - 1, nx, ny), _grid_index(i + 1, j, nz - 1, nx, ny),
                 _grid_index(i, j + 1, nz - 1, nx, ny), _grid_index(i + 1, j + 1, nz - 1, nx, ny))
    faces = np.asarray(faces, int)

    fixed_anchor = _grid_index(0, ny // 2, nz // 2, nx, ny)
    driven_anchor = _grid_index(nx - 1, ny // 2, nz // 2, nx, ny)
    # Sample the visible boundary regularly without making the controller or
    # plot dimension grow with every refinement of the volumetric lattice.
    sample_i = sorted({1, nx // 2, nx - 2})
    features = np.array([
        _grid_index(i, j, k, nx, ny)
        for k in range(0, nz, 2)
        for j in range(0, ny, 2)
        for i in sample_i
        if j in (0, ny - 1) or k in (0, nz - 1)
    ], int)
    display_edges = _unique_edges(faces)
    return SpatialSpec(
        kind="solid", label="Volumetric solid", deformation="volumetric lattice · ARAP",
        rest=rest, model_edges=model_edges, display_edges=display_edges, faces=faces,
        fixed=topological_ball(model_edges, fixed_anchor, grasp_radius),
        driven=topological_ball(model_edges, driven_anchor, grasp_radius),
        fixed_anchor=fixed_anchor, driven_anchor=driven_anchor, grasp_radius=grasp_radius,
        features=features,
        q_target=np.array([-1.00, 1.00, 0.85]),
        q_labels=("compression", "right lift", "face twist"),
    )


def spatial_spec(kind: str, grasp_radius: int = 1) -> SpatialSpec:
    if kind == "rod":
        return _rod_spec(grasp_radius)
    if kind == "sheet":
        return _sheet_spec(grasp_radius)
    if kind == "solid":
        return _solid_spec(grasp_radius)
    raise KeyError(f"unknown spatial object: {kind}")


class SpatialModel:
    """Equilibrium and feature maps for one object in the spatial demo."""

    def __init__(self, kind: str, grasp_radius: int = 1):
        self.spec = spatial_spec(kind, grasp_radius)
        if kind != "rod":
            self._prepare_arap()

    def _prepare_arap(self):
        n = len(self.spec.rest)
        adjacency = [[] for _ in range(n)]
        for a, b in self.spec.model_edges:
            adjacency[a].append(int(b))
            adjacency[b].append(int(a))
        self.adjacency = adjacency
        edges = np.asarray(self.spec.model_edges, int)
        self.edge_i = np.r_[edges[:, 0], edges[:, 1]]
        self.edge_j = np.r_[edges[:, 1], edges[:, 0]]
        self.edge_rest = self.spec.rest[self.edge_i] - self.spec.rest[self.edge_j]
        L = np.zeros((n, n))
        for i, neighbours in enumerate(adjacency):
            L[i, i] = len(neighbours)
            L[i, neighbours] = -1.0
        constrained = np.unique(np.r_[self.spec.fixed, self.spec.driven])
        self.free = np.setdiff1d(np.arange(n), constrained)
        self.constrained = constrained
        self.Lfc = L[np.ix_(self.free, constrained)]
        self.Lff_inv = np.linalg.inv(L[np.ix_(self.free, self.free)])

    def _transformed(self, q) -> np.ndarray:
        p = self.spec.rest.copy()
        if self.spec.kind == "sheet":
            y0, y1 = p[:, 1].min(), p[:, 1].max()
            eta = (p[:, 1] - y0) / (y1 - y0)
            p[:, 0] += q[0]
            p[:, 1] += q[1]
            p[:, 2] += q[2] * (0.25 + 0.75 * eta)
        elif self.spec.kind == "solid":
            c, s = np.cos(q[2]), np.sin(q[2])
            y, z = p[:, 1].copy(), p[:, 2].copy()
            p[:, 0] += q[0]
            p[:, 1] = c * y - s * z + q[1]
            p[:, 2] = s * y + c * z
        return p

    def deform(self, q) -> np.ndarray:
        q = np.asarray(q, float)
        if self.spec.kind == "rod":
            # A cubic centerline joins two rigidly grasped regions. The right
            # gripper translates in y/z and rotates the local rod tangent.
            p = self.spec.rest.copy()
            left = int(np.max(self.spec.fixed))
            right = int(np.min(self.spec.driven))
            endpoint = self.spec.rest[self.spec.driven_anchor] + np.array([0.0, q[0], q[1]])
            direction = np.array([np.cos(q[2]), np.sin(q[2]) / np.sqrt(2),
                                  np.sin(q[2]) / np.sqrt(2)])
            spacing = self.spec.rest[1, 0] - self.spec.rest[0, 0]
            for i in self.spec.driven:
                steps = self.spec.driven_anchor - i
                p[i] = endpoint - steps * spacing * direction
            p[self.spec.fixed] = self.spec.rest[self.spec.fixed]
            p0, p3 = p[left], p[right]
            chord = np.linalg.norm(p3 - p0)
            c0 = p0 + np.array([chord / 3.0, 0.0, 0.0])
            c1 = p3 - chord * direction / 3.0
            for i in range(left + 1, right):
                u = (i - left) / (right - left)
                p[i] = ((1 - u) ** 3 * p0 + 3 * (1 - u) ** 2 * u * c0
                        + 3 * (1 - u) * u ** 2 * c1 + u ** 3 * p3)
            return p

        transformed = self._transformed(q)
        span = np.ptp(self.spec.rest[:, 0])
        blend = (self.spec.rest[:, 0] - self.spec.rest[:, 0].min()) / span
        vertices = self.spec.rest + blend[:, None] * (transformed - self.spec.rest)
        constraint_positions = transformed.copy()
        constraint_positions[self.spec.fixed] = self.spec.rest[self.spec.fixed]
        constraints = constraint_positions[self.constrained]
        vertices[self.constrained] = constraints
        rest = self.spec.rest

        for _ in range(7):
            differences = vertices[self.edge_i] - vertices[self.edge_j]
            covariance = np.zeros((len(rest), 3, 3))
            np.add.at(covariance, self.edge_i,
                      differences[:, :, None] * self.edge_rest[:, None, :])
            u, _, vt = np.linalg.svd(covariance)
            rotations = u @ vt
            reflected = np.linalg.det(rotations) < 0
            u[reflected, :, -1] *= -1
            rotations = u @ vt

            average_rotation = 0.5 * (rotations[self.edge_i] + rotations[self.edge_j])
            contributions = np.einsum("eij,ej->ei", average_rotation, self.edge_rest)
            b = np.zeros_like(vertices)
            np.add.at(b, self.edge_i, contributions)
            rhs = b[self.free] - self.Lfc @ constraints
            vertices[self.free] = self.Lff_inv @ rhs
            vertices[self.constrained] = constraints
        return vertices

    def features(self, vertices) -> np.ndarray:
        return np.asarray(vertices)[self.spec.features].ravel()

    def jacobian(self, q, eps: float = 1e-3) -> np.ndarray:
        q = np.asarray(q, float)
        columns = []
        for j in range(len(q)):
            d = np.zeros_like(q)
            d[j] = eps
            columns.append((self.features(self.deform(q + d)) - self.features(self.deform(q - d))) / (2 * eps))
        return np.column_stack(columns)

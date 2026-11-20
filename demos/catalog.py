"""
The demos, in the order of the tutorial.
========================================

Each entry names its session class (`sessions/`; the pages run its port in
docs/js/sim/sessions), its page files (`docs/lessons/<id>.html`,
`docs/js/demos/<id>.js`), and a small preview drawn on the menu card.
"""
from __future__ import annotations

import importlib
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from shape_servoing import (Plant, Q_ARCH, Q_NOMINAL, Q_REACHABLE, ShapeServoingLoop,
                            unfeasible_points, numerical_jacobian, offline_estimate,
                            probe_motions, range_split, singular_triplets, target_from_grasp)
from shape_servoing.failures import stretched_grasp


@dataclass(frozen=True)
class DemoInfo:
    id: str
    part: int
    number: int
    title: str
    summary: str
    concepts: tuple
    session_path: str                     # "module:Class" inside demos.sessions

    @property
    def session(self):
        module, name = self.session_path.split(":")
        return getattr(importlib.import_module(f"demos.sessions.{module}"), name)


_ENTRIES = [
    DemoInfo("problem", 1, 1, "The shape-servoing problem",
             "Move the grippers by hand and observe the features $\\vs$, the target $\\vs^{*}$, "
             "and the shape error $\\ve=\\vs^{*}-\\vs$.",
             ("feature map", "shape error", "quasistatic equilibrium"),
             "d01_problem:ProblemSession"),
    DemoInfo("jacobian", 1, 2, "The deformation Jacobian",
             "Apply a small robot motion and compare the feature change predicted "
             "by $\\vJ$ with the measured one.",
             ("ds/dt = J u", "columns of J", "locality"),
             "d02_jacobian:JacobianSession"),
    DemoInfo("offline", 1, 3, "Offline estimation by regression",
             "Probe the rod with 24 small motions and fit the initial estimate "
             "from the action-response pairs.",
             ("probing", "least squares", "residual"),
             "d03_offline:OfflineSession"),
    DemoInfo("loop", 1, 4, "Closing the loop",
             "Run Algorithm 1 step by step: control law, motion, measurement, "
             "and the online Broyden update.",
             ("pseudoinverse law", "Broyden update", "gain"),
             "d04_loop:LoopSession"),
    DemoInfo("targets", 1, 5, "Target feasibility and reachability",
             "Separate global feasibility and reachability, then compare a recorded "
             "reachable target with a synthetic target beyond deformation limits.",
             ("feasible set", "admissible path", "infeasible target"),
             "d05_targets:TargetsSession"),
    DemoInfo("subspaces", 1, 6, "Range, null spaces, and residual error",
             "Using only point coordinates, decompose feature error and expose "
             "gripper motions that preserve two tracked point positions.",
             ("Range(J)", "Null(J^T)", "Null(J)"),
             "d06_subspaces:SubspacesSession"),
    DemoInfo("stability", 1, 7, "Local stability and convergence",
             "Perturb the Jacobian estimate and check whether its commands still "
             "reduce the shape error.",
             ("Lyapunov candidate", "Sym(J_hat^+ J)", "corrective directions"),
             "d06_stability:StabilitySession"),
    DemoInfo("svd", 2, 8, "Singular value analysis of the deformation Jacobian",
             "Move the robot along each direction $\\vpsi_l$ and compare the feature "
             "change with its prediction $\\sigma_l\\vphi_l$.",
             ("SVD", "manipulability", "condition number"),
             "d07_svd:SvdSession"),
    DemoInfo("features", 2, 9, "Choosing the shape features",
             "Compare point coordinates, edge vectors, and curvature-based features: which "
             "motions change them, what the controller can regulate, and how much "
             "each stretches a looped rod.",
             ("invariance", "null space", "deformation path"),
             "d08_features:FeaturesSession"),
    DemoInfo("failures", 2, 10, "Failure modes of the shape-servoing loop",
             "Pull the rod taut, move it in ways the features do not register, "
             "and compress it until it buckles.",
             ("controllability", "observability", "mechanical instability"),
             "d09_failures:FailuresSession"),
    DemoInfo("extensions", 2, 11, "Extensions of the shape-servoing loop",
             "Track a moving target with feedforward, pursue a secondary task "
             "in the null space of the estimate, and combine two feature "
             "representations to unloop a rod and move it to a target pose.",
             ("feedforward", "null-space projection", "two feature representations"),
             "d10_extensions:ExtensionsSession"),
    DemoInfo("spatial", 2, 12, "Three-dimensional objects",
             "Apply the same shape-servoing loop to a spatial rod and to a sheet "
             "and a solid that deform as-rigid-as-possible (ARAP).",
             ("3-D features", "ARAP", "surface and volume meshes"),
             "d11_spatial:SpatialSession"),
]
CATALOG = {entry.id: entry for entry in _ENTRIES}


# --------------------------------------------------------------------------- #
#  Menu previews: a few rods, points, a target, and arrows
# --------------------------------------------------------------------------- #
def _preview(demo_id: str) -> dict:
    plant = Plant(q=Q_NOMINAL)
    arch_nodes, arch_points, _ = target_from_grasp(Q_ARCH)
    view = {"rods": [plant.nodes], "points": plant.points, "target": None, "arrows": []}
    if demo_id in ("problem", "loop", "stability"):
        view["target"] = {"nodes": arch_nodes, "points": arch_points}
        view["arrows"] = list(zip(plant.points, arch_points))
    if demo_id == "loop":
        J0, _, _ = offline_estimate(plant.peek, Q_NOMINAL, probe_motions())
        loop = ShapeServoingLoop(plant, target_from_grasp(Q_ARCH)[2], J0)
        view["rods"] = []
        for k in range(141):
            if k in (0, 6, 16, 140):
                view["rods"].append(plant.nodes.copy())
            loop.step()
        view["points"], view["arrows"] = plant.points, []
    elif demo_id in ("jacobian", "svd"):
        J = numerical_jacobian(plant.peek, plant.q)
        column = J[:, 1] if demo_id == "jacobian" else singular_triplets(J)[1][:, -1]
        view["arrows"] = [(p, p + 1.2 * d) for p, d in zip(plant.points, column.reshape(-1, 2))]
    elif demo_id == "offline":
        view["rods"] = [plant.nodes] + [Plant(q=Q_NOMINAL + d).nodes for d in
                                        (np.r_[0, 0.35, 0, 0, 0, 0], np.r_[0, -0.35, 0, 0, 0, 0])]
    elif demo_id == "targets":
        nodes, points, _ = target_from_grasp(Q_REACHABLE)
        view["target"] = {"nodes": None, "points": unfeasible_points(points)}
        view["rods"], view["points"] = [nodes], points
        view["arrows"] = list(zip(points, unfeasible_points(points)))
    elif demo_id == "subspaces":
        _, points, _ = target_from_grasp(Q_REACHABLE)
        target = unfeasible_points(points)
        J = numerical_jacobian(plant.peek, plant.q)
        e_range, _ = range_split(J, target.ravel() - plant.s)
        projected = plant.points + e_range.reshape(-1, 2)
        view["target"] = {"nodes": None, "points": target}
        view["arrows"] = list(zip(plant.points, projected))
    elif demo_id == "features":
        view["rods"], view["points"] = [arch_nodes], arch_points
        view["arrows"] = list(zip(arch_points[:-1], arch_points[1:]))
    elif demo_id == "failures":
        view["rods"] = [Plant(q=stretched_grasp(d)).nodes for d in (6.6, 5.7, 5.0)]
        view["points"] = []
    elif demo_id == "extensions":
        view["rods"] = [Plant(q=Q_ARCH + np.r_[dx, 0, 0, dx, 0, 0]).nodes for dx in (-0.8, 0.0, 0.8)]
        view["points"] = []
    elif demo_id == "spatial":
        t = np.linspace(0.0, 1.0, 17)
        rest = np.column_stack((6.0 * (t - 0.5), 0.4 + 0.0 * t))
        target = np.column_stack((6.0 * (t - 0.5), 0.4 + 1.55 * np.sin(np.pi * t)
                                  + 0.225 * np.sin(2.0 * np.pi * t)))
        view["rods"], view["points"] = [rest, target], target[1:-1:2]
        view["target"] = {"nodes": target, "points": None}
        view["arrows"] = list(zip(rest[1:-1:2], target[1:-1:2]))
    return view


@lru_cache(maxsize=1)
def catalog_with_previews() -> list:
    return [{"id": e.id, "part": e.part, "number": e.number, "title": e.title,
             "summary": e.summary, "concepts": list(e.concepts), "preview": _preview(e.id)}
            for e in _ENTRIES]

"""
Demo 11 - Extensions of the shape-servoing loop: moving targets and secondary tasks.
==============================================================================

    tracking    a time-varying target s*(t), recorded from a moving grasp. The
                feedforward term J_hat^+ ds*/dt is added to the control law.
    secondary   with turning angles (n = 3 < m = 6) the shape task leaves robot
                directions unused. A secondary task (bring the left gripper
                to a marker) is pursued within Null(J_hat) by projecting it
                through I - J_hat^+ J_hat, or added unprojected for comparison.
    combined    two feature maps: turning angles unloop a looped rod, and a
                point-coordinate task, projected onto Null(J_hat) of the turning
                angles, transports the rod to the target pose. Only the point
                Jacobian is estimated; the turning-angle Jacobian follows from it
                by the chain rule (see `combined_loop`).
"""
from __future__ import annotations

import numpy as np

from shape_servoing import (FeatureTask, Plant, Q_ARCH, Q_STRAIGHT_MOVED, ShapeServoingLoop,
                            feature_vector, offline_estimate, probe_motions, ring_plant,
                            target_from_grasp)

from .common import Session, scene

PERIOD = 120                       # iterations per target oscillation
SECONDARY_GAIN = 0.05


def combined_loop(plant, angle_target, point_target):
    """Turning angles regulate the shape, and a point-coordinate task the pose.

    The point Jacobian J_p_hat is estimated offline and updated with Broyden's
    rule; the turning-angle Jacobian is J_hat = (d kappa / d s_p) J_p_hat, the
    known derivative of the turning angles times that estimate. Returns the loop
    and the point task (assign the task to `loop.secondary` to transport the rod).
    """
    task = FeatureTask(plant, "points", point_target, gain=0.10)
    loop = ShapeServoingLoop(plant, angle_target, task.chained_jacobian("angles"),
                             alpha=0.12, damping=0.05)
    loop.derive_jacobian_from(task)
    return loop, task


def moving_grasp(k: int) -> np.ndarray:
    """Grasp that records the moving target: the arch sways and tilts."""
    phase = 2 * np.pi * k / PERIOD
    dx, dtheta = 0.8 * np.sin(phase), 0.25 * np.sin(phase)
    return Q_ARCH + np.array([dx, 0.0, dtheta, dx, 0.0, dtheta])


class ExtensionsSession(Session):

    def __init__(self):
        self.alpha, self.feedforward, self.project = 0.15, True, True
        self.transport = True
        self._reset_tracking()
        self._reset_secondary()
        self._reset_combined()

    def state(self) -> dict:
        return {"tracking": self._tracking_state(), "secondary": self._secondary_state(),
                "combined": self._combined_state()}

    # ------------------------------------------------------------- tracking
    def _reset_tracking(self):
        self.k = 0
        self.target_plant = Plant(q=moving_grasp(0))
        self.plant = Plant(q=moving_grasp(0))
        J0, _, _ = offline_estimate(self.plant.peek, self.plant.q, probe_motions())
        self.loop = ShapeServoingLoop(self.plant, self.target_plant.s, J0, alpha=self.alpha)

    def _tracking_state(self) -> dict:
        return {"scene": scene(self.plant), "k": self.k,
                "target": {"nodes": self.target_plant.nodes, "points": self.target_plant.points},
                "error": float(np.linalg.norm(self.loop.error)),
                "params": {"alpha": self.alpha, "feedforward": self.feedforward}}

    def act_track(self, n: int = 1):
        samples = []
        for _ in range(max(1, int(n))):
            s_star_now = self.target_plant.s
            s_star_next = self.target_plant.move_to(moving_grasp(self.k + 1))
            self.loop.s_star = s_star_now                                     # e_k = s*_k - s_k
            rate = (s_star_next - s_star_now) / self.loop.dt if self.feedforward else None
            self.loop.step(s_star_rate=rate)
            self.loop.s_star = s_star_next
            self.k += 1
            samples.append({"k": self.k, "error": float(np.linalg.norm(self.loop.error))})
        return {"tracking": self._tracking_state(), "samples": samples}

    def act_set_tracking(self, alpha=None, feedforward=None):
        self.alpha = self.alpha if alpha is None else float(alpha)
        self.feedforward = self.feedforward if feedforward is None else bool(feedforward)
        self.loop.alpha = self.alpha
        return {"tracking": self._tracking_state()}

    def act_reset_tracking(self):
        self._reset_tracking()
        return {"tracking": self._tracking_state()}

    # ------------------------------------------------------------- secondary task
    def _reset_secondary(self):
        self.shape_plant = Plant(feature_map="angles", q=Q_ARCH)
        J0, _, _ = offline_estimate(self.shape_plant.peek, Q_ARCH, probe_motions())
        self.shape_loop = ShapeServoingLoop(self.shape_plant, self.shape_plant.s, J0, alpha=0.1)
        self.shape_loop.secondary = self._secondary_command
        self.shape_loop.project_secondary = self.project
        self.marker = np.array([-2.6, 2.4])
        self.secondary_k = 0

    def _secondary_command(self, q) -> np.ndarray:
        """u2 moves the left gripper toward the marker (positions only)."""
        u2 = np.zeros(6)
        u2[:2] = SECONDARY_GAIN * (self.marker - q[:2])
        return u2

    def _secondary_state(self) -> dict:
        return {"scene": scene(self.shape_plant), "k": self.secondary_k, "marker": self.marker,
                "shape_error": float(np.linalg.norm(self.shape_loop.error)),
                "distance": float(np.linalg.norm(self.marker - self.shape_plant.q[:2])),
                "project": self.project}

    def act_secondary(self, n: int = 1):
        samples = []
        for _ in range(max(1, int(n))):
            self.shape_loop.step()
            self.secondary_k += 1
            samples.append({"k": self.secondary_k, "shape_error": float(np.linalg.norm(self.shape_loop.error)),
                            "distance": float(np.linalg.norm(self.marker - self.shape_plant.q[:2]))})
        return {"secondary": self._secondary_state(), "samples": samples}

    def act_set_marker(self, xy):
        self.marker = np.clip(np.asarray(xy, float), [-4.4, -1.0], [-0.4, 3.8])
        return {"secondary": self._secondary_state()}

    def act_set_projection(self, project):
        self.project = bool(project)
        self.shape_loop.project_secondary = self.project
        return {"secondary": self._secondary_state()}

    def act_reset_secondary(self):
        self._reset_secondary()
        return {"secondary": self._secondary_state()}

    # ------------------------------------------------------------- combined feature maps
    def _reset_combined(self):
        self.target_nodes, self.target_points, _ = target_from_grasp(Q_STRAIGHT_MOVED)
        self.angle_target = feature_vector(self.target_points, "angles")
        self.point_target = feature_vector(self.target_points, "points")
        self.ring = ring_plant("angles")
        self.ring_loop, self.transport_task = combined_loop(self.ring, self.angle_target,
                                                            self.point_target)
        self.ring_loop.secondary = self.transport_task if self.transport else None
        self.combined_k = 0

    def _combined_state(self) -> dict:
        points = feature_vector(self.ring.points, "points")
        return {"scene": scene(self.ring), "k": self.combined_k, "transport": self.transport,
                "target": {"nodes": self.target_nodes, "points": self.target_points},
                "angle_error": float(np.linalg.norm(self.ring_loop.error)),
                "point_error": float(np.linalg.norm(self.point_target - points)),
                "strain": float(self.ring.rod.axial_strain(self.ring.nodes).max())}

    def act_combined(self, n: int = 1):
        samples = []
        for _ in range(max(1, int(n))):
            self.ring_loop.step()             # updates the point estimate, also without transport
            self.combined_k += 1
            state = self._combined_state()
            samples.append({"k": self.combined_k, "angle_error": state["angle_error"],
                            "point_error": state["point_error"], "strain": state["strain"]})
        return {"combined": self._combined_state(), "samples": samples}

    def act_set_transport(self, transport):
        self.transport = bool(transport)
        self.ring_loop.secondary = self.transport_task if self.transport else None
        return {"combined": self._combined_state()}

    def act_reset_combined(self):
        self._reset_combined()
        return {"combined": self._combined_state()}

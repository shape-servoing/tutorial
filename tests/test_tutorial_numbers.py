"""
The numbers quoted in the tutorial, reproduced by the `shape_servoing` package.

Run from the repository root:  python -m unittest discover tests
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from shape_servoing import Q_STRAIGHT, Q_STRAIGHT_MOVED, ring_plant  # noqa: E402
from shape_servoing import (Plant, Q_ARCH, Q_NOMINAL, Q_REACHABLE, Rod,  # noqa: E402
                            ShapeServoingLoop, condition_number, descent_margin,
                            feature_vector, unfeasible_points, numerical_jacobian,
                            numerical_rank, offline_estimate, probe_motions, range_split,
                            singular_triplets, target_from_grasp)
from shape_servoing.failures import buckling_onset, taut_sweep  # noqa: E402
from demos.sessions.d10_extensions import ExtensionsSession, combined_loop  # noqa: E402
from demos.sessions import d10_extensions as d10  # noqa: E402
from demos.sessions.d11_spatial import SpatialSession  # noqa: E402


def normalized_errors_and_strain(loop, plant, iterations):
    errors, strain = [np.linalg.norm(loop.error)], [plant.rod.axial_strain(plant.nodes).max()]
    for _ in range(iterations):
        loop.step()
        errors.append(np.linalg.norm(loop.error))
        strain.append(plant.rod.axial_strain(plant.nodes).max())
    return np.array(errors) / errors[0], np.array(strain)


def normalized_errors(loop, iterations):
    errors = [np.linalg.norm(loop.error)]
    for _ in range(iterations):
        loop.step()
        errors.append(np.linalg.norm(loop.error))
    return np.array(errors) / errors[0]


class RodModel(unittest.TestCase):
    def test_gradient_and_hessian_match_finite_differences(self):
        rod = Rod()
        rng = np.random.default_rng(0)
        nodes = rod.straight_nodes(Q_ARCH) + 0.05 * rng.standard_normal((rod.n_nodes, 2))
        eps, x = 1e-6, nodes.ravel()
        grad = rod.gradient(nodes).ravel()
        hess = rod.hessian(nodes)
        for i in range(0, x.size, 7):
            d = np.zeros_like(x)
            d[i] = eps
            plus, minus = (x + d).reshape(-1, 2), (x - d).reshape(-1, 2)
            self.assertAlmostEqual(grad[i], (rod.energy(plus) - rod.energy(minus)) / (2 * eps), delta=1e-5)
            column = (rod.gradient(plus).ravel() - rod.gradient(minus).ravel()) / (2 * eps)
            np.testing.assert_allclose(hess[:, i], column, atol=1e-3)

    def test_equilibrium_is_stationary(self):
        rod = Rod()
        nodes = rod.equilibrium(Q_ARCH)
        self.assertLess(np.abs(rod.gradient(nodes).ravel()[rod.free_coordinates]).max(), 1e-9)


class PartOne(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plant = Plant(q=Q_NOMINAL)
        cls.J0, cls.dS, cls.residual = offline_estimate(cls.plant.peek, Q_NOMINAL, probe_motions())

    def test_offline_regression(self):
        # "C = 24 ... residual ~0.31, approximately 18% of ||dS||_F ~ 1.72"
        self.assertEqual(probe_motions().shape, (6, 24))
        self.assertAlmostEqual(self.residual, 0.31, delta=0.005)
        self.assertAlmostEqual(np.linalg.norm(self.dS), 1.72, delta=0.005)
        self.assertAlmostEqual(self.residual / np.linalg.norm(self.dS), 0.18, delta=0.005)

    def test_estimate_conditioning(self):
        # "full column rank and a condition number of about 17"
        self.assertEqual(numerical_rank(self.J0), 6)
        self.assertAlmostEqual(condition_number(self.J0), 17, delta=0.5)

    def test_closed_loop_on_the_arch(self):
        s_star = target_from_grasp(Q_ARCH)[2]
        self.plant.reset(Q_NOMINAL)
        online = normalized_errors(ShapeServoingLoop(self.plant, s_star, self.J0, alpha=0.05), 140)
        self.plant.reset(Q_NOMINAL)
        fixed = normalized_errors(ShapeServoingLoop(self.plant, s_star, self.J0, alpha=0.05,
                                                    online_update=False), 140)
        self.plant.reset(Q_NOMINAL)
        fast = normalized_errors(ShapeServoingLoop(self.plant, s_star, self.J0, alpha=0.35), 140)
        self.assertLess(online[-1], 0.01)           # converges with the online update
        self.assertGreater(fixed[-1], 1.0)          # ends above its initial value with J0 fixed
        self.assertGreater(fast.max(), 1.0)         # alpha = 0.35 first rises above 1 ...
        self.assertLess(fast[-1], 1e-6)             # ... then converges

    def test_reachable_and_unfeasible_targets(self):
        _, points, s_reach = target_from_grasp(Q_REACHABLE)
        s_unfeasible = unfeasible_points(points).ravel()
        finals = []
        for s_star in (s_reach, s_unfeasible):
            self.plant.reset(Q_NOMINAL)
            loop = ShapeServoingLoop(self.plant, s_star, self.J0, alpha=0.06, damping=0.02,
                                     max_response=25.0)
            finals.append(normalized_errors(loop, 90)[-1])
        self.assertLess(finals[0], 0.01)
        self.assertGreater(finals[1], 0.3)

    def test_unfeasible_target_remains_bounded(self):
        """The illustrative residual must not drive the rod into looping wrist rotations."""
        _, points, _ = target_from_grasp(Q_REACHABLE)
        self.plant.reset(Q_NOMINAL)
        loop = ShapeServoingLoop(self.plant, unfeasible_points(points).ravel(),
                                 self.J0, alpha=0.06, damping=0.02, max_response=25.0)
        max_wrist_angle = 0.0
        for _ in range(500):
            loop.step()
            max_wrist_angle = max(max_wrist_angle, float(np.max(np.abs(self.plant.q[[2, 5]]))))
        self.assertLess(max_wrist_angle, 1.5)

    def test_unfeasible_residual_is_orthogonal_with_exact_jacobian(self):
        _, points, _ = target_from_grasp(Q_REACHABLE)
        self.plant.reset(Q_NOMINAL)
        loop = ShapeServoingLoop(self.plant, unfeasible_points(points).ravel(),
                                 self.J0, alpha=0.06, online_update=False)
        for _ in range(150):
            loop.J_hat = numerical_jacobian(self.plant.peek, self.plant.q)
            loop.step()
        e_range, e_orth = range_split(numerical_jacobian(self.plant.peek, self.plant.q), loop.error)
        self.assertLess(np.linalg.norm(e_range) / np.linalg.norm(loop.error), 1e-2)

    def test_scaled_estimate_keeps_stability(self):
        # "J_hat = c J with any c > 0 satisfies the condition with lambda = 1/c"
        J = numerical_jacobian(self.plant.peek, Q_NOMINAL)
        for c in (0.5, 2.0, 5.0):
            self.assertAlmostEqual(descent_margin(J, c * J)[0], 1 / c, places=6)


class PartTwo(unittest.TestCase):
    def test_singular_values_and_entries(self):
        # sigma_1 = 2.54, sigma_2 = 1.58, sigma_6 = 0.15; y_L moves p_1 by 0.97, p_5 by 0.03
        plant = Plant(q=Q_NOMINAL)
        J0, _, _ = offline_estimate(plant.peek, Q_NOMINAL, probe_motions())
        sigma, _, _ = singular_triplets(J0)
        np.testing.assert_allclose(sigma[[0, 1, 5]], [2.54, 1.585, 0.15], atol=0.006)
        self.assertAlmostEqual(J0[1, 1], 0.97, delta=0.005)
        self.assertAlmostEqual(J0[9, 1], 0.03, delta=0.005)

    def test_feature_representations_in_closed_loop(self):
        # Fig. 6: a disturbance pulls the grippers during the first 20 of 80 iterations.
        # Point coordinates return the rod to the target pose; edge vectors leave it
        # shifted, and curvature-based features shifted and rotated.
        start = np.array([-2.45, 1.10, 0.55, 2.45, 1.10, -0.55])
        arch_nodes, arch_points, _ = target_from_grasp(Q_ARCH)
        pull_target = transformed_arch(np.radians(-24), (2.0, 0.15))
        gains = {"points": 0.06, "edges": 0.08, "angles": 0.12}
        ranks, shift, turn = {}, {}, {}
        for feature_map, gain in gains.items():
            plant = Plant(feature_map=feature_map, q=start)
            J0, _, _ = offline_estimate(plant.peek, start, probe_motions())
            ranks[feature_map] = numerical_rank(J0)
            loop = ShapeServoingLoop(plant, feature_vector(arch_points, feature_map), J0, alpha=gain,
                                     damping=0.05, max_response=25.0, relaxation=0.4)
            loop.secondary = lambda q, loop=loop: (-0.03 * (q - pull_target) if loop.k < 20
                                                   else np.zeros_like(q))
            loop.project_secondary = False
            self.assertLess(normalized_errors(loop, 80)[-1], 0.03, feature_map)
            shift[feature_map] = np.linalg.norm(plant.nodes.mean(0) - arch_nodes.mean(0))
            chord, chord0 = plant.nodes[-1] - plant.nodes[0], arch_nodes[-1] - arch_nodes[0]
            turn[feature_map] = np.degrees(np.arctan2(chord[1], chord[0]) - np.arctan2(chord0[1], chord0[0]))
        self.assertLess(shift["points"], 0.05)                          # back at the target pose
        self.assertGreater(shift["edges"], 1.0)                         # shifted ...
        self.assertLess(abs(turn["edges"]), 1.0)                        # ... but not rotated
        self.assertGreater(shift["angles"], 1.0)                        # shifted and rotated
        self.assertGreater(abs(turn["angles"]), 5.0)
        # translations leave edge vectors unchanged; turning angles have n = 3 < m = 6
        self.assertEqual(ranks, {"points": 6, "edges": 4, "angles": 3})

    def test_taut_rod(self):
        # sigma_min 0.15 -> 0.04, w / 21, cond 16 -> 42, wrist share 68% -> 78%
        out = taut_sweep(Plant(q=Q_NOMINAL), [6.0, 7.2])
        np.testing.assert_allclose(out["sigma_min"], [0.15, 0.04], atol=0.006)
        self.assertAlmostEqual(out["manipulability"][0] / out["manipulability"][1], 21, delta=0.5)
        np.testing.assert_allclose(out["condition"], [16, 42], atol=0.5)
        np.testing.assert_allclose(out["wrist_share"], [0.68, 0.78], atol=0.005)

    def test_buckling_onset(self):
        # "crosses zero at 5.8% compression"
        separation = buckling_onset(Rod(), np.linspace(6.6, 4.8, 44))
        self.assertAlmostEqual(100 * (6.0 - separation) / 6.0, 5.8, delta=0.05)


class PartTwoExtensions(unittest.TestCase):
    """Appendix E: the experiments of demos 11 and 12."""

    def test_moving_target_with_and_without_feedforward(self):
        # "Without feedforward, the tracking error oscillates with peaks of 0.46, and
        #  each tracked point lags about 0.2 behind its target ...; with feedforward,
        #  the error remains below 1.7e-3."
        peaks, lag = {}, None
        for feedforward in (True, False):
            session = ExtensionsSession()
            session.act_set_tracking(feedforward=feedforward)
            errors, point_errors = [], []
            for _ in range(240):
                session.act_track(1)
                errors.append(np.linalg.norm(session.loop.error))
                point_errors.append(np.linalg.norm(session.loop.error.reshape(-1, 2), axis=1))
            peaks[feedforward] = max(errors)
            if not feedforward:
                lag = point_errors[int(np.argmax(errors))]
        self.assertLess(peaks[True], 1.7e-3)
        self.assertAlmostEqual(peaks[False], 0.46, delta=0.005)
        self.assertTrue(np.all((lag > 0.17) & (lag < 0.22)), lag)

    def test_secondary_objective_with_and_without_projection(self):
        # Demo 11 default (beta = 0.05, marker at (-2.6, 2.4)): below 1.2e-3 rad with
        # the projection while the gripper reaches the marker, 0.14 rad without it
        results = {}
        for project in (True, False):
            session = ExtensionsSession()
            session.act_set_projection(project)
            samples = session.act_secondary(200)["samples"]
            results[project] = (max(s["shape_error"] for s in samples), samples[-1]["distance"])
        self.assertLess(results[True][0], 1.2e-3)
        self.assertLess(results[True][1], 0.015)
        self.assertAlmostEqual(results[False][0], 0.14, delta=0.005)

    def test_secondary_objective_of_fig7(self):
        # Fig. S3 (beta = 0.2, marker at (-3.8, 2.8)): "the shape error remains
        #  below 1.4e-2 rad while the left gripper reaches the marker. Without the
        #  projection ... the shape error rises to 0.65 rad"
        results = {}
        default, d10.SECONDARY_GAIN = d10.SECONDARY_GAIN, 0.2
        try:
            for project in (True, False):
                session = ExtensionsSession()
                session.act_set_projection(project)
                session.act_set_marker([-3.8, 2.8])
                samples = session.act_secondary(200)["samples"]
                results[project] = (max(s["shape_error"] for s in samples), samples[-1]["distance"])
        finally:
            d10.SECONDARY_GAIN = default
        self.assertLess(results[True][0], 1.4e-2)
        self.assertLess(results[True][1], 1e-3)
        self.assertAlmostEqual(results[False][0], 0.65, delta=0.005)

    def test_looped_rod_feature_comparison(self):
        # "All three representations reach the unlooped rod ... the largest axial
        #  strain reaches 27% with point coordinates, 3% with edge vectors, and
        #  never exceeds its initial 2.1% with turning angles"
        _, target_points, _ = target_from_grasp(Q_STRAIGHT)
        peaks = {}
        for rep, gain in (("points", 0.06), ("edges", 0.08), ("angles", 0.12)):
            plant = ring_plant(rep)
            initial = plant.rod.axial_strain(plant.nodes).max()
            J0, _, _ = offline_estimate(plant.peek, plant.q, probe_motions())
            loop = ShapeServoingLoop(plant, feature_vector(target_points, rep), J0, alpha=gain,
                                     damping=0.05, max_response=25.0, relaxation=0.4)
            errors, strain = normalized_errors_and_strain(loop, plant, 200)
            self.assertLess(errors[-1], 0.01, rep)
            peaks[rep] = max(strain)
        self.assertAlmostEqual(initial, 0.021, delta=0.0005)
        self.assertAlmostEqual(peaks["points"], 0.27, delta=0.005)
        self.assertLess(peaks["edges"], 0.035)
        self.assertLessEqual(peaks["angles"], initial + 1e-9)

    def test_turning_angles_with_point_transport(self):
        # "With curvature-based features alone, the controller recovers the target
        #  shape but leaves a point-coordinate error of 3.6. Adding the
        #  null-space-projected point-coordinate task reduces this error below 1e-4
        #  and the curvature error below 1e-8 within 300 iterations, while the
        #  maximum axial strain never exceeds its initial value."
        # Only the point Jacobian is estimated; the turning-angle Jacobian is
        # (d kappa / d s_p) J_p_hat (chain rule).
        _, target_points, _ = target_from_grasp(Q_STRAIGHT_MOVED)
        s_points = feature_vector(target_points, "points")
        results = {}
        for combined in (False, True):
            plant = ring_plant("angles")
            initial = plant.rod.axial_strain(plant.nodes).max()
            loop, task = combined_loop(plant, feature_vector(target_points, "angles"), s_points)
            if combined:
                loop.secondary = task
            peak, stall, run = initial, 0, 0
            for _ in range(300):
                before = np.linalg.norm(task.error)
                loop.step()
                peak = max(peak, plant.rod.axial_strain(plant.nodes).max())
                # iterations in a row in which the point error drops by less than 1%
                run = run + 1 if np.linalg.norm(task.error) > 0.99 * before else 0
                stall = max(stall, run if np.linalg.norm(task.error) > 0.05 else 0)
            point_error = np.linalg.norm(s_points - feature_vector(plant.points, "points"))
            results[combined] = (np.linalg.norm(loop.error), point_error, peak - initial, stall)
        self.assertLess(results[False][0], 1e-8)
        self.assertAlmostEqual(results[False][1], 3.6, delta=0.05)
        self.assertLess(results[True][0], 1e-8)
        self.assertLess(results[True][1], 1e-4)
        self.assertLessEqual(results[True][2], 1e-9)
        self.assertLess(results[True][3], 20)          # no long pause before the transport

    def test_three_dimensional_objects(self):
        # n = 21, 27, 72; "the normalized error falls below 1% within 20 iterations"
        session = SpatialSession()
        for kind, n in (("rod", 21), ("sheet", 27), ("solid", 72)):
            with self.subTest(kind=kind):
                state = session.act_set_object(kind)
                self.assertEqual(3 * len(state["features"]), n)
                for _ in range(20):
                    state = session.act_step()
                self.assertLess(state["error"] / state["initial_error"], 0.01)


def transformed_arch(angle, shift, center=(0.0, 1.15)):
    """Grasp of the arch target moved rigidly (the pull target of the disturbance of Section VI)."""
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    q = np.array(Q_ARCH, float)
    for i in (0, 3):
        q[i:i + 2] = rotation @ (q[i:i + 2] - center) + center + np.asarray(shift)
        q[i + 2] += angle
    return q


if __name__ == "__main__":
    unittest.main()

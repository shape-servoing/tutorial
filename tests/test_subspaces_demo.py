"""Regression tests for the Section IV-B/IV-C interactive examples."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from demos.sessions.d05_targets import TargetsSession  # noqa: E402
from demos.sessions.d06_subspaces import SubspacesSession  # noqa: E402
from shape_servoing import descent_margin, unfeasible_points, unreachable_points  # noqa: E402


class SectionFourDemos(unittest.TestCase):

    def test_unfeasible_name_keeps_legacy_alias(self):
        points = np.zeros((5, 2))
        np.testing.assert_allclose(unfeasible_points(points), unreachable_points(points))
        session = TargetsSession()
        self.assertEqual(session.act_set_target("unfeasible")["kind"], "unfeasible")

    def test_feature_error_orthogonal_decomposition(self):
        state = SubspacesSession().state()
        J = np.asarray(state["J"])
        e = np.asarray(state["e"])
        parallel = np.asarray(state["e_range"])
        residual = np.asarray(state["e_orth"])
        np.testing.assert_allclose(e, parallel + residual, atol=1e-12)
        np.testing.assert_allclose(J.T @ residual, 0.0, atol=1e-10)
        np.testing.assert_allclose(J @ np.linalg.pinv(J) @ e, parallel, atol=1e-10)
        self.assertEqual(state["dimensions"]["null_jt_dim"], 4)

    def test_robot_null_motion_preserves_two_point_positions_to_first_order(self):
        session = SubspacesSession()
        state = session.act_set_case("motion")
        np.testing.assert_allclose(state["linear_null_change"], 0.0, atol=1e-10)
        self.assertEqual(state["dimensions"]["n"], 4)
        self.assertEqual(state["dimensions"]["range_jt_dim"], 4)
        self.assertEqual(state["dimensions"]["null_j_dim"], 2)
        self.assertLess(np.linalg.norm(state["null_change"]), 0.01)

    def test_tall_jacobian_uses_robot_space_stability_condition(self):
        J = np.vstack((np.eye(2), np.array([[0.4, -0.2]])))
        angle = np.radians(60.0)
        rotation = np.array([[np.cos(angle), -np.sin(angle)],
                             [np.sin(angle), np.cos(angle)]])
        margin, eigenvalues = descent_margin(J, J @ rotation)
        self.assertEqual(len(eigenvalues), 2)
        self.assertAlmostEqual(margin, np.cos(angle), places=10)


if __name__ == "__main__":
    unittest.main()

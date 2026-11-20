"""Regression tests for the 3-D rod and ARAP mesh demo."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from demos.sessions.d11_spatial import SpatialSession  # noqa: E402
from shape_servoing.spatial import SpatialModel, topological_ball  # noqa: E402


class SpatialModels(unittest.TestCase):

    def test_topology_constraints_and_finite_jacobians(self):
        expected = {"rod": (17, 0), "sheet": (35, 48), "solid": (150, 224)}
        minimum_target_rms = {"rod": 0.90, "sheet": 0.95, "solid": 1.00}
        for kind, (n_vertices, n_faces) in expected.items():
            with self.subTest(kind=kind):
                model = SpatialModel(kind)
                target = model.deform(model.spec.q_target)
                self.assertEqual(target.shape, (n_vertices, 3))
                self.assertEqual(len(model.spec.faces), n_faces)
                self.assertTrue(np.isfinite(target).all())
                target_rms = np.linalg.norm(target - model.spec.rest) / np.sqrt(n_vertices)
                self.assertGreater(target_rms, minimum_target_rms[kind])
                np.testing.assert_allclose(target[model.spec.fixed], model.spec.rest[model.spec.fixed],
                                           atol=1e-10)
                self.assertEqual(model.spec.grasp_radius, 1)
                self.assertIn(model.spec.fixed_anchor, model.spec.fixed)
                self.assertIn(model.spec.driven_anchor, model.spec.driven)
                self.assertEqual(len(np.intersect1d(model.spec.fixed, model.spec.driven)), 0)
                np.testing.assert_array_equal(
                    model.spec.fixed,
                    topological_ball(model.spec.model_edges, model.spec.fixed_anchor,
                                     model.spec.grasp_radius),
                )
                np.testing.assert_array_equal(
                    model.spec.driven,
                    topological_ball(model.spec.model_edges, model.spec.driven_anchor,
                                     model.spec.grasp_radius),
                )
                self.assertEqual(np.linalg.matrix_rank(model.jacobian(np.zeros(3))), 3)
                if kind == "solid":
                    self.assertLess(len(model.spec.driven) / n_vertices, 0.10)

    def test_all_three_shape_control_runs_converge(self):
        session = SpatialSession()
        for kind in ("rod", "sheet", "solid"):
            with self.subTest(kind=kind):
                state = session.act_set_object(kind)
                for _ in range(18):
                    state = session.act_step()
                self.assertLess(state["error"] / state["initial_error"], 0.03)
                self.assertEqual(np.asarray(state["jacobian"]).shape[1], 3)
                self.assertEqual(state["rank"], 3)
                self.assertTrue(np.isfinite(state["condition"]))
                self.assertEqual(len(state["command_vector"]), 3)


if __name__ == "__main__":
    unittest.main()

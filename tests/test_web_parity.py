"""
The JavaScript port of the demo sessions (docs/js/sim) gives the same results as
the Python sessions (demos/sessions) and the shape_servoing package.

tests/web_parity.mjs runs scripted actions on every JavaScript session; the
scenarios below run the same actions in Python. Skipped without Node.js.
Quantities that depend on random numbers (random targets, probe noise, random
perturbations) are not compared, and neither are directions whose sign or basis
an SVD chooses freely.
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from demos.catalog import _ENTRIES, _preview, CATALOG  # noqa: E402
from demos.sessions import d10_extensions  # noqa: E402

NODE = shutil.which("node")


def start(demo_id):
    return CATALOG[demo_id].session()


def call(session, action, **payload):
    return session.dispatch(action, payload)


def displacements(frames):
    first = np.asarray(frames[0]["points"])
    return sorted(float(np.linalg.norm(np.asarray(f["points"]) - first)) for f in frames)


def scenario_catalog():
    return {"entries": [[e.id, e.part, e.number, e.title, e.summary, list(e.concepts)] for e in _ENTRIES],
            "previews": [_preview(e.id) for e in _ENTRIES if e.id != "svd"]}


def scenario_problem():
    s = start("problem")
    a = s.state()
    b = call(s, "move", q=[-2.6, 1.4, 0.3, 2.8, 0.9, -0.2])
    return {"s0": a["s"], "e0": a["e"], "initial_error": a["initial_error"], "s1": b["s"],
            "error1": b["error_norm"], "nodes1": b["scene"]["nodes"]}


def scenario_jacobian():
    s = start("jacobian")
    a = s.state()
    b = call(s, "preset", name="taut")
    c = call(s, "set_dq", dq=[0.1, 0.0, 0.2, 0.0, -0.1, 0.0])
    play = call(s, "play", frames=8)
    return {"J": a["J"], "predicted": a["predicted"], "actual": a["actual"],
            "relative_error": a["relative_error"], "curve": a["curve"]["errors"], "J_taut": b["J"],
            "rel_taut": c["relative_error"], "play": [f["points"] for f in play["frames"]]}


def scenario_offline():
    s = start("offline")
    p = call(s, "probe")
    m = call(s, "set_model", bend=2.0, gravity=0.5)
    p2 = call(s, "set_params", scale=0.5, per_dof=2)
    q2 = call(s, "probe")
    sweep = call(s, "sweep")
    e = p["estimate"]
    return {"J0": e["J0"], "residual": e["residual"], "error": e["error"], "condition": e["condition"],
            "sigma": e["sigma"], "model_error": m["model"]["error"], "J": p2["J"],
            "residual2": q2["estimate"]["residual"], "ratio": sweep["residual_ratio"],
            "sweep_error": sweep["error"]}


def scenario_loop():
    s = start("loop")
    a = call(s, "step", n=1)
    b = call(s, "step", n=40)
    call(s, "set_params", gate=True, gate_ratio=2.0)
    c = call(s, "step", n=40)
    call(s, "set_params", online=False, alpha=0.3)
    call(s, "reset")
    d = call(s, "step", n=30)
    return {"detail_u": a["detail"]["u"], "detail_r": a["detail"]["r"], "correction": a["detail"]["correction"],
            "errors_b": [x["error"] for x in b["samples"]], "J_hat_b": b["J_hat"],
            "rejected_c": [x["rejected"] for x in c["samples"]], "errors_c": [x["error"] for x in c["samples"]],
            "errors_d": [x["error"] for x in d["samples"]]}


def scenario_targets():
    s = start("targets")
    a = call(s, "step", n=60)
    call(s, "set_target", kind="unfeasible")
    b = call(s, "step", n=60)
    call(s, "set_params", exact=True)
    c = call(s, "step", n=30)
    return {"errors_a": [x["error"] for x in a["samples"]], "split_a": a["split"]["range_fraction"],
            "errors_b": [x["error"] for x in b["samples"]], "orth_b": b["split"]["orth_fraction"],
            "exact_orth_b": b["split"]["exact_orth_fraction"], "errors_c": [x["error"] for x in c["samples"]],
            "orth_c": c["split"]["orth_fraction"]}


def scenario_subspaces():
    s = start("subspaces")
    a = s.state()
    b = call(s, "set_params", range_weight=0.5, residual_weight=2.0)
    c = call(s, "set_case", case="motion")
    return {"e": a["e"], "e_range": a["e_range"], "command": a["command"],
            "fractions": [b["range_fraction"], b["orth_fraction"]], "dims": list(c["dimensions"].values()),
            "J_motion": c["J"], "dq_task_abs": np.abs(c["dq_task"]), "task_change_abs": np.abs(c["task_change"]),
            "null_change_norm": np.linalg.norm(c["null_change"]),
            "null_linear_norm": np.linalg.norm(c["linear_null_change"])}


def scenario_stability():
    s = start("stability")
    a = s.state()
    b = call(s, "set_estimate", scale=0.6, angle=40, plane="xL-xR")
    c = call(s, "step", n=30)
    d = call(s, "set_estimate", scale=1.0, angle=0, flip=2)
    e = call(s, "step", n=30)
    return {"J": a["J"], "margin": a["margin"], "rate": a["rate"], "margin_b": b["margin"],
            "eig_b": b["eigenvalues"], "matrix_error_b": b["matrix_error"],
            "errors_c": [x["error"] for x in c["samples"]], "margin_d": d["margin"],
            "errors_e": [x["error"] for x in e["samples"]], "diverged_e": e["diverged"]}


def scenario_svd():
    s = start("svd")
    a = s.state()
    b = call(s, "set", units="deg")
    c = call(s, "set", source="exact", units="rad")
    d = call(s, "move", q=[-2.9, 1.1, 0.1, 2.9, 1.0, -0.1])
    play = call(s, "play", index=0, frames=6)
    return {"sigma": a["sigma"], "w": a["manipulability"], "cond": a["condition"], "rank": a["rank"],
            "sigma_deg": b["sigma"], "sigma_exact": c["sigma"], "sigma_moved": d["sigma"],
            "play": displacements(play["frames"])}


def scenario_features():
    s = start("features")
    out = {}
    for rep in ("points", "edges", "angles"):
        out[f"rank_{rep}"] = call(s, "set_rep", rep=rep)["rank"]
        out[f"errors_{rep}"] = [x["error"] for x in call(s, "step", n=80)["samples"]]
    cmp = call(s, "compare")
    out["compare"] = [cmp["runs"][r]["ratio"] for r in ("points", "edges", "angles")]
    for kind in ("translate", "rotate", "deform"):
        out[f"gesture_{kind}"] = [f["features"] for f in call(s, "gesture", kind=kind, frames=9)["frames"]]
    call(s, "set_rep", rep="angles")
    nm = call(s, "null_motion", frames=8)
    out["null_dim"] = nm["null_dim"]
    out["null_feature_change"] = max(f["feature_change"] for f in nm["frames"])
    ring = call(s, "compare_ring", iterations=60)
    out["ring"] = [[ring["runs"][r]["peak"], ring["runs"][r]["error"][-1]] for r in ("points", "edges", "angles")]
    return out


def scenario_failures():
    s = start("failures")
    a = s.state()
    t = call(s, "taut", extension=12)
    out = {"sweep_sigma": a["sweep"]["sigma_min"], "sweep_cond": a["sweep"]["condition"],
           "wrist": a["sweep"]["wrist_share"], "lambda": a["buckling"]["lambda"], "onset": a["buckling"]["onset"],
           "taut_sigma": t["taut"]["sigma"], "taut_share": t["taut"]["wrist_share"], "psi_min": t["taut"]["psi_min"]}
    for scenario in ("controllability", "observability", "healthy"):
        out[f"ratio_{scenario}"] = call(s, "observe", scenario=scenario)["ratio"]
    comp = [call(s, "compress", compression=c)["compress"] for c in (-5, 0, 3, 6, 9, 12)]
    out["compress"] = [[c["lambda_straight"], c["lambda_equilibrium"], c["ratio"]] for c in comp]
    for th in (0.8, 0.6, 0.4, 0.2, 0.0, -0.2):
        snap = call(s, "wrist", theta=th)["snap"]
    out["snap"] = [snap["path"]["lambda"], snap["path"]["ratio"][1:]]
    return out


def scenario_extensions():
    s = start("extensions")
    tr = [x["error"] for x in call(s, "track", n=120)["samples"]]
    call(s, "set_tracking", feedforward=False)
    call(s, "reset_tracking")
    tr2 = [x["error"] for x in call(s, "track", n=120)["samples"]]
    sec = call(s, "secondary", n=60)["samples"]
    call(s, "set_projection", project=False)
    call(s, "reset_secondary")
    sec2 = call(s, "secondary", n=60)["samples"]
    comb = call(s, "combined", n=300)["samples"]
    call(s, "set_transport", transport=False)
    call(s, "reset_combined")
    comb2 = call(s, "combined", n=300)["samples"]
    default, d10_extensions.SECONDARY_GAIN = d10_extensions.SECONDARY_GAIN, 0.2
    try:
        paper = start("extensions")
        call(paper, "set_marker", xy=[-3.8, 2.8])
        p1 = call(paper, "secondary", n=200)["samples"]
    finally:
        d10_extensions.SECONDARY_GAIN = default
    return {"track_ff": tr, "track_fb": tr2, "shape": [x["shape_error"] for x in sec],
            "dist": [x["distance"] for x in sec], "shape_unprojected": [x["shape_error"] for x in sec2],
            "transport_angle": [x["angle_error"] for x in comb], "transport_point": [x["point_error"] for x in comb],
            "transport_strain": max(x["strain"] for x in comb),
            "curvature_only_point": comb2[-1]["point_error"], "curvature_only_angle": comb2[-1]["angle_error"],
            "paper_secondary": [max(x["shape_error"] for x in p1), p1[-1]["distance"]]}


def scenario_spatial():
    s = start("spatial")
    out = {}
    for kind in ("rod", "sheet", "solid"):
        a = call(s, "set_object", kind=kind)
        errors = []
        for _ in range(20):
            st = call(s, "step", n=1)
            errors.append(st["samples"][0]["error"])
        out[kind] = {"errors": errors, "sigma": st["singular_values"], "cond": st["condition"], "rank": st["rank"],
                     "q": st["q"], "target": a["target"], "n": 3 * len(a["features"])}
    return out


# Comparison of each quantity: (relative, absolute) tolerance, or "small" for
# first-order zeros that depend on the choice of a null-space basis.
LOOP = (1e-4, 1e-9)            # closed-loop runs: rounding differences grow along the run
TOLERANCES = {
    "features": {key: LOOP for key in ("errors_points", "errors_edges", "errors_angles", "compare", "ring")}
    | {"null_feature_change": "small"},
    "extensions": {key: LOOP for key in ("track_ff", "track_fb", "shape", "dist", "shape_unprojected",
                                         "transport_angle", "transport_point", "curvature_only_point",
                                         "curvature_only_angle", "paper_secondary")},
    "loop": {key: LOOP for key in ("detail_r", "correction", "errors_b", "errors_c", "errors_d", "J_hat_b")},
    "targets": {key: LOOP for key in ("errors_a", "errors_b", "errors_c", "orth_c")},
    "stability": {"errors_c": LOOP, "errors_e": LOOP},
    "subspaces": {"null_change_norm": "small"},
    "spatial": {"errors": LOOP, "q": LOOP},
}
DEFAULT = (1e-6, 1e-9)


def scenario(name):
    return globals()[f"scenario_{name}"]()


def plain(value):
    return json.loads(json.dumps(value, default=lambda v: np.asarray(v).tolist()),
                      parse_constant=lambda c: None)


@unittest.skipIf(NODE is None, "Node.js is not installed")
class WebParity(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        result = subprocess.run([NODE, str(ROOT / "tests" / "web_parity.mjs")], capture_output=True,
                                text=True, encoding="utf-8", cwd=ROOT, timeout=600)
        if result.returncode != 0:
            raise RuntimeError(result.stderr)
        cls.js = json.loads(result.stdout)

    def compare(self, name, key, js, py, spec):
        label = f"{name}.{key}"
        if spec == "small":                   # first-order zero: both small
            self.assertLess(abs(js), 2e-2, label)
            self.assertLess(abs(py), 2e-2, label)
            return
        rtol, atol = spec
        if isinstance(js, dict):
            self.assertEqual(set(js), set(py), label)
            for k in js:
                self.compare(name, f"{key}.{k}", js[k], py[k], TOLERANCES.get(name, {}).get(k, spec))
            return
        if isinstance(js, str) or isinstance(py, str):
            self.assertEqual(js, py, label)
            return
        if isinstance(js, bool) or isinstance(py, bool):
            self.assertEqual(list(np.atleast_1d(js)), list(np.atleast_1d(py)), label)
            return
        if js is None or py is None:
            self.assertEqual(js, py, label)
            return
        try:
            a = np.array(js, dtype=float)             # None (inf in Python) -> nan
            b = np.array(py, dtype=float)
        except (TypeError, ValueError):               # lists of dicts or of unequal lists
            self.assertEqual(len(js), len(py), label)
            for i, (x, y) in enumerate(zip(js, py)):
                self.compare(name, f"{key}[{i}]", x, y, spec)
            return
        self.assertEqual(a.shape, b.shape, label)
        scale = max(np.nanmax(np.abs(b)), 1.0) if b.size and not np.all(np.isnan(b)) else 1.0
        np.testing.assert_allclose(a, b, rtol=rtol, atol=atol * scale, equal_nan=True, err_msg=label)

    def check(self, name):
        js = self.js[name]
        py = plain(scenario(name))
        for key, value in py.items():
            spec = TOLERANCES.get(name, {}).get(key, DEFAULT)
            with self.subTest(quantity=f"{name}.{key}"):
                self.compare(name, key, js[key], value, spec)

    def test_catalog(self): self.check("catalog")
    def test_problem(self): self.check("problem")
    def test_jacobian(self): self.check("jacobian")
    def test_offline(self): self.check("offline")
    def test_loop(self): self.check("loop")
    def test_targets(self): self.check("targets")
    def test_subspaces(self): self.check("subspaces")
    def test_stability(self): self.check("stability")
    def test_svd(self): self.check("svd")
    def test_features(self): self.check("features")
    def test_failures(self): self.check("failures")
    def test_extensions(self): self.check("extensions")
    def test_spatial(self): self.check("spatial")


if __name__ == "__main__":
    unittest.main()

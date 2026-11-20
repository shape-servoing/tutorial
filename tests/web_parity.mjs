// Runs scripted actions on the JavaScript sessions of the pages (docs/js/sim)
// and prints the results as JSON; tests/test_web_parity.py runs the same
// actions on the Python sessions (demos/sessions) and compares them.
//
//     node tests/web_parity.mjs [scenario ...]
//
// Quantities that depend on random numbers, or on the sign or basis that an SVD
// picks for a singular or null direction, are left out or compared up to sign.
import { CATALOG, ENTRIES, preview } from '../docs/js/sim/catalog.js';
import { ExtensionsSession } from '../docs/js/sim/sessions/d10_extensions.js';

const start = (id, options) => (id === 'extensions' && options ? new ExtensionsSession(options) : new CATALOG[id].session());
const call = (s, action, payload = {}) => s.dispatch(action, payload);
const last = (a) => a[a.length - 1];
// Sorted displacements of the tracked points from the first frame (independent of
// the sign of the direction played).
const displacements = (frames) => frames.map((f) => Math.hypot(...f.points.flatMap((p, i) =>
    [p[0] - frames[0].points[i][0], p[1] - frames[0].points[i][1]]))).sort((x, y) => x - y);

export const SCENARIOS = {
    catalog() {
        // Menu entries, and the menu previews except that of Demo 08 (an SVD direction).
        return { entries: ENTRIES.map((e) => [e.id, e.part, e.number, e.title, e.summary, e.concepts]),
            previews: ENTRIES.filter((e) => e.id !== 'svd').map((e) => preview(e.id)) };
    },
    problem() {
        const s = start('problem');
        const a = s.state();
        const b = call(s, 'move', { q: [-2.6, 1.4, 0.3, 2.8, 0.9, -0.2] });
        return { s0: a.s, e0: a.e, initial_error: a.initial_error, s1: b.s, error1: b.error_norm, nodes1: b.scene.nodes };
    },
    jacobian() {
        const s = start('jacobian');
        const a = s.state();
        const b = call(s, 'preset', { name: 'taut' });
        const c = call(s, 'set_dq', { dq: [0.1, 0.0, 0.2, 0.0, -0.1, 0.0] });
        const play = call(s, 'play', { frames: 8 });
        return { J: a.J, predicted: a.predicted, actual: a.actual, relative_error: a.relative_error,
            curve: a.curve.errors, J_taut: b.J, rel_taut: c.relative_error, play: play.frames.map((f) => f.points) };
    },
    offline() {
        const s = start('offline');
        const p = call(s, 'probe');
        const m = call(s, 'set_model', { bend: 2.0, gravity: 0.5 });
        const p2 = call(s, 'set_params', { scale: 0.5, per_dof: 2 });
        const q2 = call(s, 'probe');
        const sweep = call(s, 'sweep');
        return { J0: p.estimate.J0, residual: p.estimate.residual, error: p.estimate.error,
            condition: p.estimate.condition, sigma: p.estimate.sigma, model_error: m.model.error,
            J: p2.J, residual2: q2.estimate.residual, ratio: sweep.residual_ratio, sweep_error: sweep.error };
    },
    loop() {
        const s = start('loop');
        const a = call(s, 'step', { n: 1 });
        const b = call(s, 'step', { n: 40 });
        call(s, 'set_params', { gate: true, gate_ratio: 2.0 });
        const c = call(s, 'step', { n: 40 });
        call(s, 'set_params', { online: false, alpha: 0.3 });
        call(s, 'reset');
        const d = call(s, 'step', { n: 30 });
        return { detail_u: a.detail.u, detail_r: a.detail.r, correction: a.detail.correction,
            errors_b: b.samples.map((x) => x.error), J_hat_b: b.J_hat, rejected_c: c.samples.map((x) => x.rejected),
            errors_c: c.samples.map((x) => x.error), errors_d: d.samples.map((x) => x.error) };
    },
    targets() {
        const s = start('targets');
        const a = call(s, 'step', { n: 60 });
        call(s, 'set_target', { kind: 'unfeasible' });
        const b = call(s, 'step', { n: 60 });
        call(s, 'set_params', { exact: true });
        const c = call(s, 'step', { n: 30 });
        return { errors_a: a.samples.map((x) => x.error), split_a: a.split.range_fraction,
            errors_b: b.samples.map((x) => x.error), orth_b: b.split.orth_fraction,
            exact_orth_b: b.split.exact_orth_fraction, errors_c: c.samples.map((x) => x.error),
            orth_c: c.split.orth_fraction };
    },
    subspaces() {
        const s = start('subspaces');
        const a = s.state();
        const b = call(s, 'set_params', { range_weight: 0.5, residual_weight: 2.0 });
        const c = call(s, 'set_case', { case: 'motion' });
        const abs = (v) => v.map(Math.abs);
        return { e: a.e, e_range: a.e_range, command: a.command, fractions: [b.range_fraction, b.orth_fraction],
            dims: Object.values(c.dimensions), J_motion: c.J, dq_task_abs: abs(c.dq_task),
            task_change_abs: abs(c.task_change), null_change_norm: Math.hypot(...c.null_change),
            null_linear_norm: Math.hypot(...c.linear_null_change) };
    },
    stability() {
        const s = start('stability');
        const a = s.state();
        const b = call(s, 'set_estimate', { scale: 0.6, angle: 40, plane: 'xL-xR' });
        const c = call(s, 'step', { n: 30 });
        const d = call(s, 'set_estimate', { scale: 1.0, angle: 0, flip: 2 });
        const e = call(s, 'step', { n: 30 });
        return { J: a.J, margin: a.margin, rate: a.rate, margin_b: b.margin, eig_b: b.eigenvalues,
            matrix_error_b: b.matrix_error, errors_c: c.samples.map((x) => x.error),
            margin_d: d.margin, errors_e: e.samples.map((x) => x.error), diverged_e: e.diverged };
    },
    svd() {
        const s = start('svd');
        const a = s.state();
        const b = call(s, 'set', { units: 'deg' });
        const c = call(s, 'set', { source: 'exact', units: 'rad' });
        const d = call(s, 'move', { q: [-2.9, 1.1, 0.1, 2.9, 1.0, -0.1] });
        const play = call(s, 'play', { index: 0, frames: 6 });
        return { sigma: a.sigma, w: a.manipulability, cond: a.condition, rank: a.rank, sigma_deg: b.sigma,
            sigma_exact: c.sigma, sigma_moved: d.sigma, play: displacements(play.frames) };
    },
    features() {
        const s = start('features');
        const out = {};
        for (const rep of ['points', 'edges', 'angles']) {
            const st = call(s, 'set_rep', { rep });
            out[`rank_${rep}`] = st.rank;
            out[`errors_${rep}`] = call(s, 'step', { n: 80 }).samples.map((x) => x.error);
        }
        const cmp = call(s, 'compare');
        out.compare = ['points', 'edges', 'angles'].map((r) => cmp.runs[r].ratio);
        for (const kind of ['translate', 'rotate', 'deform']) {
            out[`gesture_${kind}`] = call(s, 'gesture', { kind, frames: 9 }).frames.map((f) => f.features);
        }
        call(s, 'set_rep', { rep: 'angles' });
        const nm = call(s, 'null_motion', { frames: 8 });
        out.null_dim = nm.null_dim;
        out.null_feature_change = Math.max(...nm.frames.map((f) => f.feature_change));
        const ring = call(s, 'compare_ring', { iterations: 60 });
        out.ring = ['points', 'edges', 'angles'].map((r) => [ring.runs[r].peak, last(ring.runs[r].error)]);
        return out;
    },
    failures() {
        const s = start('failures');
        const a = s.state();
        const t = call(s, 'taut', { extension: 12 });
        const out = { sweep_sigma: a.sweep.sigma_min, sweep_cond: a.sweep.condition, wrist: a.sweep.wrist_share,
            lambda: a.buckling.lambda, onset: a.buckling.onset, taut_sigma: t.taut.sigma,
            taut_share: t.taut.wrist_share, psi_min: t.taut.psi_min };
        for (const scenario of ['controllability', 'observability', 'healthy']) out[`ratio_${scenario}`] = call(s, 'observe', { scenario }).ratio;
        const comp = [];
        for (const c of [-5, 0, 3, 6, 9, 12]) comp.push(call(s, 'compress', { compression: c }).compress);
        out.compress = comp.map((c) => [c.lambda_straight, c.lambda_equilibrium, c.ratio]);
        let snap = null;
        for (const th of [0.8, 0.6, 0.4, 0.2, 0.0, -0.2]) snap = call(s, 'wrist', { theta: th }).snap;
        out.snap = [snap.path.lambda, snap.path.ratio.slice(1)];
        return out;
    },
    extensions() {
        const s = start('extensions');
        const tr = call(s, 'track', { n: 120 }).samples.map((x) => x.error);
        call(s, 'set_tracking', { feedforward: false });
        call(s, 'reset_tracking');
        const tr2 = call(s, 'track', { n: 120 }).samples.map((x) => x.error);
        const sec = call(s, 'secondary', { n: 60 }).samples;
        call(s, 'set_projection', { project: false });
        call(s, 'reset_secondary');
        const sec2 = call(s, 'secondary', { n: 60 }).samples;
        const comb = call(s, 'combined', { n: 300 }).samples;
        call(s, 'set_transport', { transport: false });
        call(s, 'reset_combined');
        const comb2 = call(s, 'combined', { n: 300 }).samples;
        const paper = start('extensions', { secondaryGain: 0.2 });
        call(paper, 'set_marker', { xy: [-3.8, 2.8] });
        const p1 = call(paper, 'secondary', { n: 200 }).samples;
        return { track_ff: tr, track_fb: tr2, shape: sec.map((x) => x.shape_error), dist: sec.map((x) => x.distance),
            shape_unprojected: sec2.map((x) => x.shape_error),
            transport_angle: comb.map((x) => x.angle_error), transport_point: comb.map((x) => x.point_error),
            transport_strain: Math.max(...comb.map((x) => x.strain)),
            curvature_only_point: last(comb2).point_error, curvature_only_angle: last(comb2).angle_error,
            paper_secondary: [Math.max(...p1.map((x) => x.shape_error)), last(p1).distance] };
    },
    spatial() {
        const s = start('spatial');
        const out = {};
        for (const kind of ['rod', 'sheet', 'solid']) {
            const a = call(s, 'set_object', { kind });
            const errors = [];
            let st = a;
            for (let k = 0; k < 20; k++) { st = call(s, 'step', { n: 1 }); errors.push(st.samples[0].error); }
            out[kind] = { errors, sigma: st.singular_values, cond: st.condition, rank: st.rank, q: st.q,
                target: a.target, n: 3 * a.features.length };
        }
        return out;
    },
};

const names = process.argv.slice(2);
const out = {};
for (const name of names.length ? names : Object.keys(SCENARIOS)) {
    const t0 = performance.now();
    out[name] = SCENARIOS[name]();
    out[name].__ms = performance.now() - t0;
}
process.stdout.write(JSON.stringify(out, (_, v) => (typeof v === 'number' && !Number.isFinite(v) ? null : v)));

// Demo 03 - Offline estimation by regression: J0 = dS dQ^+ from C probes.
import { DOF_COLS, FEATURE_ROWS } from '../lib/matrix.js';
import { gripperDrag, lerp, lerpNodes } from '../lib/scene.js';
import { alpha } from '../lib/theme.js';
import { button, card, h, readouts, segmented, slider, fmt, pct } from '../lib/ui.js';
import { matrixCard, plotCard, sceneCard } from '../lib/widgets.js';

const PROBE_MS = 260;

export async function mount({ workspace, session, state }) {
    let S = state;
    let anim = null;                 // {q, nodes, points, column}
    let shown = null;                // columns of dQ, dS shown so far

    const sc = sceneCard('Exploratory motions around $\\vq_0$', {
        note: 'drag the grippers to choose q0',
        guide: 'Drag either gripper to choose the identification configuration. Probe & fit then applies small motions around it; compare the fitted estimate with the exact simulation Jacobian below.',
        view: { x0: -5.6, x1: 5.6, y0: -1.8, y1: 4.2 },
        legendItems: (p) => [
            { label: 'rod at $\\vq_0$', color: p.rod, kind: 'line' },
            { label: 'probe response $\\ds_c$', color: p.aqua, kind: 'line' },
            { label: 'probe $\\dq_c$', color: p.accentStrong, kind: 'line' },
        ],
    });
    const move = session.stream('move', (next) => { S = next; shown = null; update(); });
    gripperDrag(sc.scene, { getQ: () => S.scene.q, getNodes: () => S.scene.nodes, enabled: () => !anim, onMove: (q) => move({ q }) });
    const probeButton = button('▶ Probe & fit', () => probe(true), { primary: true });
    const instantButton = button('Fit without animation', () => probe(false));
    const sweepButton = button('Sweep amplitudes', sweep);
    sc.toolbar.append(probeButton, instantButton);

    // ---- rail
    const params = card('Exploration');
    const amp = slider({ label: 'amplitude × (0.35 length units, 0.18 rad)', description: 'Larger probes improve signal strength but expose more nonlinearity.', min: 0.02, max: 4, value: 1, log: true,
        format: (v) => `×${v.toFixed(2)}`, onInput: (v) => setParams({ scale: v }) });
    const noise = slider({ label: 'measurement noise $\\sigma$', description: 'Adds noise to each measured feature response.', min: 0, max: 0.05, step: 0.001, value: 0,
        format: (v) => v.toFixed(3), onInput: (v) => setParams({ noise: v }) });
    const perDof = segmented({ options: [{ value: 1, label: '1' }, { value: 2, label: '2' }, { value: 4, label: '4' }],
        value: 4, onChange: (v) => setParams({ per_dof: v }) });
    params.body.append(amp.el, noise.el, h('div', { class: 'control' },
        h('div', { class: 'control-head' }, h('label', {}, 'probes per coordinate')), perDof.el));
    const setParams = session.stream('set_params', (next) => { S = next; shown = null; update(); });

    const fitCard = card('Fit');
    const out = readouts([
        { key: 'C', label: '$C$' },
        { key: 'res', label: '$\\norm{\\dS-\\vJh_0\\dQ}_F$' },
        { key: 'dS', label: '$\\norm{\\dS}_F$' },
        { key: 'ratio', label: 'residual / $\\norm{\\dS}_F$' },
        { key: 'err', label: '$\\norm{\\vJh_0-\\vJ}_F/\\norm{\\vJ}_F$' },
        { key: 'cond', label: '$\\cond(\\vJh_0)$' },
    ]);
    fitCard.body.append(out.el);

    const modelCard = card('Optional: model-based estimate $\\vJh_{\\vg}=\\partial\\vg/\\partial\\vq$', 'Appendix A · no probing');
    const setModel = session.stream('set_model', (next) => { S.model = next.model; updateModel(); });
    const bend = slider({ label: 'model bending stiffness ×', min: 0.25, max: 4, value: 1, log: true,
        format: (v) => `×${v.toFixed(2)}`, onInput: (v) => setModel({ bend: v }) });
    const weight = slider({ label: 'model weight ×', min: 0, max: 2, value: 1, step: 0.05,
        format: (v) => `×${v.toFixed(2)}`, onInput: (v) => setModel({ gravity: v }) });
    const modelOut = readouts([{ key: 'err', label: '$\\norm{\\vJh_{\\vg}-\\vJ}_F/\\norm{\\vJ}_F$' }]);
    modelCard.body.append(bend.el, weight.el, modelOut.el);

    // ---- matrices and sweep
    const dQCard = matrixCard('Actions $\\dQ$', { rowLabels: DOF_COLS, maxCell: 22 }, { note: 'm × C', height: 170 });
    const dSCard = matrixCard('Responses $\\dS$', { rowLabels: FEATURE_ROWS, maxCell: 22 }, { note: 'n × C', height: 260 });
    const J0Card = matrixCard('Estimate $\\vJh_0=\\dS\\dQ^{+}$', { rowLabels: FEATURE_ROWS, colLabels: DOF_COLS, maxCell: 26 }, { height: 300 });
    const JCard = matrixCard('Exact $\\vJ(\\vq_0)$ (simulation only)', { rowLabels: FEATURE_ROWS, colLabels: DOF_COLS, maxCell: 26 }, { height: 300 });
    const JgCard = matrixCard('Model-based $\\vJh_{\\vg}$ (same color scale as $\\vJ$)', { rowLabels: FEATURE_ROWS, colLabels: DOF_COLS, maxCell: 26 }, { height: 300 });
    const sweepCard = plotCard('Amplitude sweep', { xLabel: 'amplitude ×', yLabel: 'relative error', logX: true, logY: true, x: [0.02, 4] },
        { note: 'runs its own probes at each amplitude; does not change Ĵ₀', size: 'tall' });
    sweepCard.body.prepend(h('div', { style: 'margin-bottom:6px' }, sweepButton));

    workspace.append(
        h('div', { class: 'stage' }, sc.el, h('div', { class: 'rail' }, params.el, fitCard.el)),
        h('section', { class: 'plots' }, dQCard.el, dSCard.el),
        h('section', { class: 'plots' }, J0Card.el, JCard.el, sweepCard.el),
        h('section', { class: 'plots' }, modelCard.el, JgCard.el));

    // ---------------------------------------------------------------- logic
    async function probe(animated) {
        [probeButton, instantButton, sweepButton].forEach((b) => { b.disabled = true; });
        const result = await session.call('probe');
        S = result;
        if (animated) await playProbes(result.probes);
        shown = null;
        update();
        [probeButton, instantButton, sweepButton].forEach((b) => { b.disabled = false; });
    }

    function playProbes(probes) {
        return new Promise((resolve) => {
            const t0 = performance.now();
            const base = S.scene;
            const tick = (now) => {
                const u = Math.max(0, now - t0) / PROBE_MS;
                const c = Math.floor(u);
                if (c >= probes.length) { anim = null; resolve(); return; }
                const w = Math.sin(Math.PI * (u - c));
                const pr = probes[c];
                anim = { q: lerp(base.q, pr.q, w), nodes: lerpNodes(base.nodes, pr.nodes, w),
                    points: lerpNodes(base.points, pr.points, w), probe: pr, column: c, w };
                shown = c + (u - c >= 0.5 ? 1 : 0);
                drawScene();
                drawMatrices();
                requestAnimationFrame(tick);
            };
            requestAnimationFrame(tick);
        });
    }

    async function sweep() {
        sweepButton.disabled = true;
        sweepCard.note.textContent = 'computing…';
        const r = await session.call('sweep');
        const p = sc.scene.p;
        sweepCard.plot.set({
            series: [
                { x: r.scales, y: r.residual_ratio, color: p.rod, width: 2 },
                { x: r.scales, y: r.error, color: p.red, width: 2 },
            ],
            vlines: [{ x: S.params.scale, color: alpha(p.accentStrong, 0.7) }],
        });
        sweepCard.setLegend([
            { label: 'residual / $\\norm{\\dS}_F$', color: p.rod },
            { label: 'error of $\\vJh_0$ vs. $\\vJ$', color: p.red },
        ]);
        sweepCard.note.textContent = `noise σ = ${S.params.noise.toFixed(3)} · mean of two noise draws`;
        sweepButton.disabled = false;
    }

    function maskColumns(matrix, count) {
        return matrix.map((row) => row.map((v, c) => (c < count ? v : null)));
    }

    function drawMatrices() {
        const E = S.estimate;
        if (!E) {
            const C = 6 * S.params.per_dof;
            dQCard.matrix.set(Array.from({ length: 6 }, () => Array(C).fill(null)), { vmax: 1 });
            dSCard.matrix.set(Array.from({ length: 10 }, () => Array(C).fill(null)), { vmax: 1 });
            return;
        }
        const count = shown ?? E.dQ[0].length;
        const col = anim ? anim.column : -1;
        dQCard.matrix.set(maskColumns(E.dQ, count), { col });
        dSCard.matrix.set(maskColumns(E.dS, count), { col });
    }

    function drawScene() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background();
            const q = anim ? anim.q : S.scene.q;
            g.arms(q);
            if (anim) g.rod(S.scene.nodes, { alphaValue: 0.25, halo: false });
            g.rod(anim ? anim.nodes : S.scene.nodes);
            if (anim && anim.w > 0.25) {
                S.scene.points.forEach((pt, i) => g.arrow(pt, anim.probe.points[i], { color: p.aqua, width: 2.2, head: 8 }));
                g.command(S.scene.q, anim.probe.q.map((v, j) => v - S.scene.q[j]), { color: p.accentStrong });
            }
            g.points(anim ? anim.points : S.scene.points);
            g.grippers(q);
            if (!anim) g.handles(q);
            if (anim) g.badge(`probe c = ${anim.column + 1} of ${S.estimate.dQ[0].length}`);
        });
    }

    function updateModel() {
        const err = S.model.error;
        modelOut.set('err', pct(err, 1), err > 0.25 ? 'bad' : err < 0.05 ? 'good' : '');
        JgCard.matrix.opts.colColors = sc.scene.p.dof;
        JgCard.matrix.set(S.model.J, { vmax: Math.max(...S.J.flat().map(Math.abs)) });
    }

    function update() {
        drawScene();
        drawMatrices();
        const p = sc.scene.p;
        const vmax = Math.max(...S.J.flat().map(Math.abs));
        JCard.matrix.opts.colColors = p.dof;
        J0Card.matrix.opts.colColors = p.dof;
        JCard.matrix.set(S.J, { vmax });
        const E = S.estimate;
        if (E) {
            J0Card.matrix.set(E.J0, { vmax });
            out.set('C', String(E.dQ[0].length));
            out.set('res', fmt(E.residual, 3));
            out.set('dS', fmt(E.dS_norm, 3));
            out.set('ratio', pct(E.residual / E.dS_norm, 1));
            out.set('err', pct(E.error, 1), E.error > 0.25 ? 'bad' : E.error < 0.05 ? 'good' : '');
            out.set('cond', fmt(E.condition, 1));
        } else {
            J0Card.matrix.set(Array.from({ length: 10 }, () => Array(6).fill(null)), { vmax });
            for (const key of ['C', 'res', 'dS', 'ratio', 'err', 'cond']) out.set(key, '–');
            out.set('C', String(6 * S.params.per_dof));
        }
        updateModel();
    }
    update();
}

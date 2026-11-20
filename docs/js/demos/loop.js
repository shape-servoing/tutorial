// Demo 04 - Closing the loop: Algorithm 1 with the online Broyden update.
import { AlgorithmView } from '../lib/algorithm.js';
import { DOF_COLS, FEATURE_ROWS } from '../lib/matrix.js';
import { Runner } from '../lib/runner.js';
import { gripperDrag, lerpNodes, lerp, smoothstep } from '../lib/scene.js';
import { alpha, mix } from '../lib/theme.js';
import { button, card, h, readouts, segmented, slider, toggle, fmt, pct, DOF_TEX } from '../lib/ui.js';
import { matrixCard, plotCard, sceneCard } from '../lib/widgets.js';

const VIS = 10;          // display gain for the small per-iteration vectors

export async function mount({ workspace, session, state }) {
    let S = state;
    let walk = null;          // line-by-line overlay: {line, detail, t}
    let trail = [];
    let lastCorrection = null;   // latest Broyden correction (matrix)
    let run = newRun();
    const pastRuns = [];

    // ---- scene
    const sc = sceneCard('Closed loop', {
        note: 'pause, then drag the grippers to change the shape by hand',
        guide: 'Run continuously or step through Algorithm 1. While paused, drag a gripper or its rotation handle; the next iterations reveal whether the estimate adapts to the changed shape.',
        legendItems: (p) => [
            { label: 'rod', color: p.rod, kind: 'line' },
            { label: 'target $\\vs^{*}$', color: p.target, kind: 'dashed' },
            { label: `command $\\vu_k$ (×${VIS})`, color: p.accentStrong, kind: 'line' },
            { label: `response $\\ds_{k+1}$ (×${VIS})`, color: p.aqua, kind: 'line' },
        ],
    });
    const runner = new Runner(session, { rate: 20, onResult: (r) => apply(r), onStop: () => syncButtons() });
    const move = session.stream('move', (next) => { S = next; update(); });
    gripperDrag(sc.scene, { getQ: () => S.scene.q, getNodes: () => S.scene.nodes,
        enabled: () => !runner.running && !walk, onMove: (q) => move({ q }) });

    const runButton = button('▶ Run', () => { if (runner.running) runner.stop(); else runner.start(); syncButtons(); }, { primary: true });
    const stepButton = button('Step (one iteration)', stepThrough);
    const resetButton = button('Reset to $\\vq_0$', reset);
    const targetChoice = segmented({
        options: [{ value: 'arch', label: 'Arch' }, { value: 'random', label: 'Random' }, { value: 'current', label: 'Record current' }],
        value: 'arch',
        onChange: async (kind) => { runner.stop(); archive(); S = await session.call('set_target', { kind }); restart(); },
    });
    sc.toolbar.append(runButton, stepButton, resetButton, h('span', { class: 'toolbar-divider' }),
        h('span', { class: 'hint' }, 'target:'), targetChoice.el);

    // ---- rail
    const algo = new AlgorithmView();
    const algoCard = card('', '');
    algoCard.body.append(algo.el);
    const paramsCard = card('Parameters');
    const setParams = session.stream('set_params', (next) => { S = { ...S, params: next.params }; update(); });
    const gain = slider({ label: 'gain $\\alpha$', description: 'Scales every command: faster response also means larger, less-local motions.', min: 0.01, max: 0.6, step: 0.01, value: S.params.alpha,
        onInput: (v) => setParams({ alpha: v }) });
    const speed = slider({ label: 'iterations per second', min: 2, max: 120, step: 1, value: 20,
        format: (v) => v.toFixed(0), onInput: (v) => { runner.rate = v; } });
    const online = toggle({ label: 'online Broyden update', description: 'On: learn from each motion. Off: hold the initial local estimate fixed.', value: true, onChange: (v) => setParams({ online: v }) });
    const gate = toggle({ label: 'gate the update by the response ratio', description: 'On: pairs with $\\norm{\\ds}/\\norm{\\dq}$ above the threshold do not update the estimate.', value: false, onChange: (v) => setParams({ gate: v }) });
    const gateRatio = slider({ label: 'gate threshold', description: 'Ratios in this demo stay well below the default 25; lower the threshold to see rejected pairs freeze $\\vJh_k$.', min: 1, max: 25, step: 0.5, value: S.params.gate_ratio,
        format: (v) => v.toFixed(1), onInput: (v) => setParams({ gate_ratio: v }) });
    paramsCard.body.append(gain.el, speed.el, online.el, gate.el, gateRatio.el);
    const out = readouts([
        { key: 'k', label: 'iteration $k$' },
        { key: 'err', label: '$\\norm{\\ve_k}/\\norm{\\ve_0}$' },
        { key: 'cond', label: '$\\cond(\\vJh_k)$' },
        { key: 'ratio', label: 'last $\\norm{\\ds}/\\norm{\\dq}$' },
        { key: 'rejected', label: 'rejected pairs' },
    ]);
    paramsCard.body.append(out.el);

    // ---- plots and matrices
    const errorCard = plotCard('Normalized error', { xLabel: 'iteration k', yLabel: '‖e_k‖ / ‖e_0‖', y: null, yMinSpan: 1 }, { size: 'tall' });
    const uCard = plotCard('Commanded velocities $\\vu_k$', { xLabel: 'iteration k', yLabel: 'u_k,j' }, { size: 'tall' });
    const corrCard = plotCard('Broyden correction $\\norm{\\vr_k}/\\norm{\\dq_{k-1}}$', { xLabel: 'iteration k', yLabel: '', logY: true }, { size: 'tall' });
    const jCard = matrixCard('Estimate $\\vJh_k$', { rowLabels: FEATURE_ROWS, colLabels: DOF_COLS, maxCell: 24 }, { height: 280 });
    const dJCard = matrixCard('Latest rank-one correction $\\vJh_{k+1}-\\vJh_k$', { rowLabels: FEATURE_ROWS, colLabels: DOF_COLS, maxCell: 24 }, { height: 280 });

    workspace.append(
        h('div', { class: 'stage wide-rail' }, sc.el, h('div', { class: 'rail' }, algoCard.el, paramsCard.el)),
        h('section', { class: 'plots' }, errorCard.el, uCard.el, corrCard.el),
        h('section', { class: 'plots' }, jCard.el, dJCard.el));

    // ---------------------------------------------------------------- data
    function newRun() {
        return { k: [0], err: [1], u: [], uk: [], corr: [], ck: [], alpha: S.params.alpha, online: S.params.online, ratio: NaN, rejected: 0 };
    }

    function archive() {
        if (run.k.length > 2) {
            pastRuns.unshift(run);
            pastRuns.length = Math.min(pastRuns.length, 3);
        }
    }

    function restart() { run = newRun(); trail = []; walk = null; lastCorrection = null; update(); }

    async function reset() { runner.stop(); archive(); S = await session.call('reset'); restart(); }

    function apply(result) {
        S = result;
        for (const s of result.samples) {
            run.k.push(s.k); run.err.push(s.error / S.initial_error);
            run.u.push(s.u); run.uk.push(s.k);
            if (s.updated) { run.corr.push(s.correction); run.ck.push(s.k); }
            run.ratio = s.ratio; if (s.rejected) run.rejected += 1;
        }
        run.alpha = S.params.alpha; run.online = S.params.online;
        lastCorrection = result.detail.correction;
        trail.push(S.scene.nodes);
        if (trail.length > 5) trail.shift();
        update();
    }

    // ---------------------------------------------------------------- one iteration, line by line
    const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
    async function stepThrough() {
        runner.stop();
        syncButtons(true);
        const result = await runner.once();
        const d = result.detail;
        walk = { line: 3, detail: d, t: 0 };
        for (const line of [3, 4, 5, 6, 7, 8, 9]) {
            walk.line = line;
            algo.highlight(line);
            if (line === 5) {
                const t0 = performance.now();
                await new Promise((resolve) => {
                    const tick = (now) => {
                        walk.t = Math.min(1, Math.max(0, now - t0) / 700);
                        walk.after = result.scene;
                        draw();
                        if (walk.t < 1) requestAnimationFrame(tick); else resolve();
                    };
                    requestAnimationFrame(tick);
                });
            } else {
                if (line === 8) { apply(result); walk.applied = true; dJCard.matrix.pulse(); jCard.matrix.pulse(); }
                draw();
                await sleep(line === 9 ? 450 : 850);
            }
        }
        if (!walk.applied) apply(result);
        walk = null;
        algo.highlight(0);
        draw();
        syncButtons(false);
    }

    function syncButtons(busy = false) {
        runButton.textContent = runner.running ? '❚❚ Pause' : '▶ Run';
        [stepButton, resetButton].forEach((b) => { b.disabled = busy || runner.running; });
        runButton.disabled = busy;
    }

    // ---------------------------------------------------------------- drawing
    function draw() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background();
            g.target(S.target);
            const d = walk?.detail;
            let q = S.scene.q, nodes = S.scene.nodes, points = S.scene.points;
            if (walk && !walk.applied) {
                const t = walk.line >= 5 ? smoothstep(walk.line === 5 ? walk.t : 1) : 0;
                const after = walk.after || S.scene;
                q = lerp(d.before.q, after.q || d.before.q, t);
                nodes = lerpNodes(d.before.nodes, after.nodes || d.before.nodes, t);
                points = lerpNodes(d.before.points, after.points || d.before.points, t);
            }
            g.arms(q);
            trail.slice(0, -1).forEach((tn, i) => g.rod(tn, { alphaValue: 0.12 + 0.08 * i, halo: false, width: 0.07 }));
            g.rod(nodes);
            const e = walk ? d.e : S.e;
            const from = walk ? d.before.points : S.scene.points;
            from.forEach((pt, i) => {
                const strong = walk && walk.line === 3;
                g.arrow(pt, [pt[0] + e[2 * i], pt[1] + e[2 * i + 1]], {
                    color: strong ? p.red : alpha(p.rod, 0.45), width: strong ? 2.6 : 1.3, head: strong ? 9 : 6 });
            });
            if (walk && walk.line === 4) g.command(d.before.q, d.u.map((v) => VIS * v), { color: p.accentStrong });
            if (walk && walk.line >= 7 && walk.line <= 8) {
                d.before.points.forEach((pt, i) => g.arrow(pt, [pt[0] + VIS * d.ds[2 * i], pt[1] + VIS * d.ds[2 * i + 1]],
                    { color: p.aqua, width: 2.4, head: 8 }));
            }
            g.points(points, { r: walk && walk.line === 6 ? 0.16 : 0.1 });
            g.grippers(q);
            if (!runner.running && !walk) g.handles(q);
            if (walk) g.badge(['', '', '', 'line 3 · shape error eₖ', `line 4 · command uₖ (×${VIS})`, 'line 5 · apply Δqₖ = uₖ Δt',
                'line 6 · measure sₖ₊₁', `line 7 · feature change Δsₖ₊₁ (×${VIS})`, 'line 8 · Broyden update of Ĵₖ', 'line 9 · k ← k + 1'][walk.line]);
            g.corner(`k = ${S.k}`);
        });
    }

    function update() {
        draw();
        const p = sc.scene.p;
        out.set('k', String(S.k));
        const ratio = S.error / S.initial_error;
        out.set('err', pct(ratio, 1), ratio < 0.01 ? 'good' : ratio > 1 ? 'bad' : '');
        out.set('cond', fmt(S.condition, 1));
        out.set('ratio', Number.isFinite(run.ratio) ? fmt(run.ratio, 2) : '-');
        out.set('rejected', String(run.rejected), run.rejected ? 'bad' : '');
        // error plot: current run, past runs, nominal decay
        const kMax = Math.max(60, run.k[run.k.length - 1], ...pastRuns.map((r) => r.k[r.k.length - 1]));
        const nominalK = Array.from({ length: kMax + 1 }, (_, i) => i);
        const series = [{ x: nominalK, y: nominalK.map((k) => (1 - S.params.alpha) ** k), color: alpha(p.ink, 0.45), width: 1.2, dash: [2, 3] }];
        const legendItems = [{ label: `nominal $(1-\\alpha)^k$, $\\alpha=${S.params.alpha.toFixed(2)}$`, color: alpha(p.ink, 0.6), dash: 'dotted' }];
        pastRuns.slice().reverse().forEach((r, i) => {
            const color = r.online ? mix(p.rod, p.canvas, 0.55) : mix(p.red, p.canvas, 0.45);
            series.push({ x: r.k, y: r.err, color, width: 1.5 });
            legendItems.push({ label: `earlier: $\\alpha=${r.alpha.toFixed(2)}$, ${r.online ? 'online' : '$\\vJh_0$ fixed'}`, color });
        });
        series.push({ x: run.k, y: run.err, color: S.params.online ? p.rod : p.red, width: 2.4 });
        legendItems.push({ label: `this run: ${S.params.online ? 'online update' : '$\\vJh_0$ fixed'}`, color: S.params.online ? p.rod : p.red });
        errorCard.plot.opts.x = [0, kMax];
        errorCard.plot.set({ series });
        errorCard.setLegend(legendItems);
        // commands
        uCard.plot.opts.x = [0, kMax];
        uCard.plot.set({ series: DOF_TEX.map((_, j) => ({ x: run.uk, y: run.u.map((u) => u[j]), color: p.dof[j], width: 1.6 })) });
        uCard.setLegend(DOF_TEX.map((t, j) => ({ label: `$${t}$`, color: p.dof[j] })));
        corrCard.plot.opts.x = [0, kMax];
        corrCard.plot.set({ series: [{ x: run.ck, y: run.corr, color: p.accentStrong, width: 1.6 }] });
        jCard.matrix.opts.colColors = p.dof;
        dJCard.matrix.opts.colColors = p.dof;
        jCard.matrix.set(S.J_hat);
        dJCard.matrix.set(lastCorrection || S.J_hat.map((row) => row.map(() => 0)), { vmax: lastCorrection ? undefined : 1 });
        syncButtons();
    }
    update();
}

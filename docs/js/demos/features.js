// Demo 08 - Choosing the shape features: invariances, closed loop, null space,
// and a looped rod (deformation imposed by each representation).
import { Runner } from '../lib/runner.js';
import { SceneView, lerp, lerpNodes } from '../lib/scene.js';
import { alpha, mix, palette } from '../lib/theme.js';
import { button, card, h, readouts, segmented, fmt, pct } from '../lib/ui.js';
import { plotCard, sceneCard } from '../lib/widgets.js';

const REPS = ['points', 'edges', 'angles'];
const REP_NAME = { points: 'point coordinates $\\vp_i$', edges: 'edge vectors $\\vd_i$', angles: 'curvature $\\kappa_i$' };
const REP_SHORT = { points: 'point coordinates', edges: 'edge vectors', angles: 'curvature' };
const subscript = (n) => String(n).replace(/\d/g, (d) => '₀₁₂₃₄₅₆₇₈₉'[d]);
const repColor = (p, rep) => ({ points: p.repPoints, edges: p.repEdges, angles: p.repAngles })[rep];
const ramp = (p, rep, i, n) => mix(mix(repColor(p, rep), p.canvas, 0.45), mix(repColor(p, rep), p.ink, 0.35), n <= 1 ? 0.5 : i / (n - 1));

// Axial strain in percent; tiny compressions are shown as 0.0%.
const strainText = (v) => `${Math.max(0, 100 * v).toFixed(1)}%`;

export async function mount({ workspace, session, state, lesson }) {
    let S = state;
    let rep = S.rep;
    let gesture = null;            // {kind, frames, index}
    let traces = { points: [], edges: [], angles: [] };
    let nullFrame = null;
    let run = { k: [0], err: [1] };

    // ---------------------------------------------------------------- tabs
    const tabs = h('div', { class: 'tabs', role: 'tablist' });
    const panels = {};
    for (const [id, title] of [['inv', 'Invariances'], ['loop', 'Closed loop'], ['ring', 'Looped rod']]) {
        const b = h('button', { type: 'button', role: 'tab', 'aria-selected': String(id === 'inv') }, title);
        b.addEventListener('click', () => {
            tabs.querySelectorAll('button').forEach((x) => x.setAttribute('aria-selected', String(x === b)));
            Object.entries(panels).forEach(([k, el]) => { el.style.display = k === id ? '' : 'none'; });
            if (id !== 'ring') ringRunner.stop();
            if (id !== 'loop') runner.stop();
            if (id === 'loop') drawLoop(); else if (id === 'ring') { drawRing(); plotRing(); } else drawInv();
        });
        tabs.append(b);
    }
    const repChoice = () => segmented({
        options: REPS.map((r) => ({ value: r, label: REP_NAME[r], color: repColor(palette(), r) })), value: rep,
        onChange: async (r) => { rep = r; repChoices.forEach((c) => c.set(r)); runner.stop(); S = await session.call('set_rep', { rep: r }); run = { k: [0], err: [1] }; update(); },
    });
    const repChoices = [repChoice(), repChoice()];

    // ---------------------------------------------------------------- invariances
    const inv = sceneCard('Tracked points and the chosen representation', {
        compact: true,
        guide: 'Select a feature representation, then apply Translate, Rotate, or Bend. The plots below show which feature changes each representation can—and cannot—observe.',
        arms: false, camera: false, view: { x0: -4.6, x1: 4.6, y0: -0.6, y1: 4.6 },
        legendItems: (p) => [{ label: 'rod', color: p.rod, kind: 'line' }, { label: 'tracked points', color: p.feature, kind: 'dot' }],
    });
    const gestureButtons = [['translate', 'Translate'], ['rotate', 'Rotate'], ['deform', 'Bend']].map(([kind, text]) =>
        button(text, () => playGestures([kind])));
    inv.toolbar.append(repChoices[0].el, h('span', { class: 'toolbar-divider' }), ...gestureButtons,
        button('All three', () => playGestures(['translate', 'rotate', 'deform']), { primary: true }));
    const tracePlots = Object.fromEntries(REPS.map((r) => [r, plotCard(REP_NAME[r], { xLabel: 'gesture', yLabel: r === 'angles' ? 'Δκ_i [rad]' : `Δ${r === 'points' ? 'p' : 'd'}_i`, x: [0, 3] },
        { size: 'tall', note: r === 'angles' ? '' : 'solid x, dashed y' })]));
    panels.inv = h('div', {}, inv.el, h('section', { class: 'plots' }, ...REPS.map((r) => tracePlots[r].el)));

    // ---------------------------------------------------------------- closed loop
    const loopScene = sceneCard('Closed loop from a common start to the arch', {
        guide: `Select the regulated features and run. During the first ${state.disturbance_steps} iterations, a disturbance pulls the grippers; each loop undoes only the drift its features register. Compare all three final poses, or animate a null-space motion to see the rod move while those features remain unchanged.`,
        legendItems: (p) => [{ label: 'rod', color: p.rod, kind: 'line' }, { label: 'target', color: p.target, kind: 'dashed' },
            { label: 'tracked points', color: p.feature, kind: 'dot' }],
    });
    const runner = new Runner(session, { rate: 20, onResult: apply, onStop: syncButtons });
    const runButton = button('▶ Run', () => { if (runner.running) runner.stop(); else runner.start(); syncButtons(); }, { primary: true });
    const resetButton = button('Reset', async () => { runner.stop(); S = await session.call('reset'); run = { k: [0], err: [1] }; update(); });
    const nullButton = button('Null-space motion', playNull);
    const compareButton = button('Compare all three', compare);
    loopScene.toolbar.append(repChoices[1].el, h('span', { class: 'toolbar-divider' }), runButton, resetButton, nullButton, compareButton);
    const stats = card('This representation');
    const out = readouts([
        { key: 'n', label: '$n$' }, { key: 'rank', label: '$\\operatorname{rank}(\\vJh_0)$' }, { key: 'null', label: '$\\dim\\Null(\\vJh_0)$' },
        { key: 'k', label: 'iteration $k$' }, { key: 'err', label: '$\\norm{\\ve_k}/\\norm{\\ve_0}$' },
    ]);
    stats.body.append(out.el);
    const errCard = plotCard('Feature error of the selected representation', { xLabel: 'iteration k', yLabel: '‖e_k‖ / ‖e_0‖', logY: true, y: [1e-3, 2] }, { size: 'tall' });
    const nullCard = plotCard('During the null-space motion', { xLabel: 'motion phase', yLabel: 'change from start', x: [0, 1] }, { size: 'tall' });
    const compareCard = card(`Final configurations after ${S.horizon} iterations`, 'same start, same target, same disturbance');
    const minis = REPS.map((r) => {
        const canvas = h('canvas');
        const caption = h('div', { class: 'mini-caption' }, '');
        return { rep: r, canvas, caption, view: new SceneView(canvas, { arms: false, view: { x0: -4.2, x1: 6.4, y0: -0.4, y1: 4.6 } }), el: h('div', {}, canvas, caption) };
    });
    compareCard.body.append(h('div', { class: 'mini-grid' }, minis.map((m) => m.el)));
    panels.loop = h('div', { style: 'display:none' },
        h('div', { class: 'stage' }, loopScene.el, h('div', { class: 'rail' }, stats.el)),
        h('section', { class: 'plots' }, errCard.el, nullCard.el), h('div', { style: 'margin-top:12px' }, compareCard.el));

    // ---------------------------------------------------------------- looped rod
    let R = (await session.call('ring_state')).ring;
    let ringRun = newRingRun();
    let ringCompare = null;            // histories of all three representations
    const ringScene = sceneCard('From a looped rod to the same rod, unlooped', {
        guide: 'Select the features and run, or compare all three. With point coordinates the grippers move apart before the loop opens and the rod is stretched; with curvature-based features the loop opens first.',
        legendItems: (p) => [{ label: 'rod', color: p.rod, kind: 'line' }, { label: 'target (unlooped)', color: p.target, kind: 'dashed' },
            { label: 'tracked points', color: p.feature, kind: 'dot' }],
    });
    const ringRunner = new Runner(session, { action: 'step_ring', rate: 30, onResult: applyRing, onStop: syncRing });
    const ringRunButton = button('▶ Run', () => {
        if (ringRunner.running) ringRunner.stop(); else { ringCompare = null; ringRunner.start(); }
        syncRing();
    }, { primary: true });
    const ringResetButton = button('Reset', async () => {
        ringRunner.stop(); ringCompare = null;
        R = (await session.call('reset_ring')).ring; ringRun = newRingRun(); drawRing(); plotRing();
    });
    const ringCompareButton = button('Compare all three', compareRing);
    const ringChoice = segmented({
        options: REPS.map((r) => ({ value: r, label: REP_NAME[r], color: repColor(palette(), r) })), value: R.rep,
        onChange: async (r) => {
            ringRunner.stop(); ringCompare = null;
            R = (await session.call('set_ring_rep', { rep: r })).ring; ringRun = newRingRun(); drawRing(); plotRing();
        },
    });
    ringScene.toolbar.append(ringChoice.el, h('span', { class: 'toolbar-divider' }), ringRunButton, ringResetButton, ringCompareButton);
    const ringStats = card('Deformation of the rod');
    const ringOut = readouts([
        { key: 'k', label: 'iteration $k$' }, { key: 'err', label: '$\\norm{\\ve_k}/\\norm{\\ve_0}$' },
        { key: 'strain', label: 'largest axial strain' }, { key: 'peak', label: 'peak so far' },
    ]);
    ringStats.body.append(ringOut.el);
    const ringErrCard = plotCard('Feature error', { xLabel: 'iteration k', yLabel: '‖e_k‖ / ‖e_0‖', logY: true, y: [1e-6, 2], x: [0, 200] }, { size: 'tall' });
    const ringStrainCard = plotCard('Largest axial strain of the rod', { xLabel: 'iteration k', yLabel: 'strain [%]', y: [0, 32], x: [0, 200] }, { size: 'tall' });
    panels.ring = h('div', { style: 'display:none' },
        h('div', { class: 'stage' }, ringScene.el, h('div', { class: 'rail' }, ringStats.el)),
        h('section', { class: 'plots' }, ringErrCard.el, ringStrainCard.el));

    workspace.append(tabs, panels.inv, panels.loop, panels.ring);

    // ---------------------------------------------------------------- looped rod logic
    function newRingRun() { return { rep: R.rep, k: [R.k], err: [R.error], strain: [100 * R.strain] }; }
    function applyRing(result) {
        R = result.ring;
        for (const s of result.samples) { ringRun.k.push(s.k); ringRun.err.push(s.error); ringRun.strain.push(100 * s.strain); }
        if (R.k >= 400) ringRunner.stop();
        drawRing(); plotRing();
    }
    function syncRing() {
        ringRunButton.textContent = ringRunner.running ? '❚❚ Pause' : '▶ Run';
        [ringResetButton, ringCompareButton].forEach((b) => { b.disabled = ringRunner.running; });
    }
    async function compareRing() {
        ringRunner.stop();
        ringCompareButton.disabled = true;
        ringErrCard.note.textContent = 'computing…';
        ringCompare = await session.call('compare_ring');
        ringErrCard.note.textContent = '';
        ringCompareButton.disabled = false;
        drawRing(); plotRing();
    }
    function drawRing() {
        ringScene.scene.draw((g) => {
            g.background();
            g.target(R.target);
            if (ringCompare) {
                g.rod(ringCompare.start, { alphaValue: 0.25, halo: false, width: 0.07 });
                for (const r of REPS) g.rod(ringCompare.runs[r].peak_nodes, { color: repColor(g.p, r), alphaValue: 0.8, halo: false, width: 0.08 });
                g.badge('each representation at its largest strain');
                return;
            }
            g.arms(R.scene.q);
            g.rod(R.scene.nodes, { color: repColor(g.p, R.rep) });
            g.points(R.scene.points, { r: 0.1 });
            g.grippers(R.scene.q);
            g.badge(`largest axial strain ${strainText(R.strain)}`);
            g.corner(`k = ${R.k}`);
        });
        ringOut.set('k', String(R.k));
        ringOut.set('err', pct(R.error, 2), R.error < 0.01 ? 'good' : '');
        ringOut.set('strain', strainText(R.strain), R.strain > 0.1 ? 'bad' : '');
        ringOut.set('peak', strainText(R.peak_strain), R.peak_strain > 0.1 ? 'bad' : '');
    }
    function plotRing() {
        const p = palette();
        const errSeries = [], strainSeries = [], legend = [];
        if (ringCompare) {
            for (const r of REPS) {
                const run = ringCompare.runs[r], xs = run.error.map((_, i) => i);
                errSeries.push({ x: xs, y: run.error, color: repColor(p, r), width: 2 });
                strainSeries.push({ x: xs, y: run.strain.map((v) => 100 * v), color: repColor(p, r), width: 2 });
                legend.push({ label: `${REP_NAME[r]} · peak ${(100 * run.peak).toFixed(0)}%`, color: repColor(p, r) });
            }
        } else {
            errSeries.push({ x: ringRun.k, y: ringRun.err, color: repColor(p, ringRun.rep), width: 2.4 });
            strainSeries.push({ x: ringRun.k, y: ringRun.strain, color: repColor(p, ringRun.rep), width: 2.4 });
            legend.push({ label: REP_NAME[ringRun.rep], color: repColor(p, ringRun.rep) });
        }
        const xMax = Math.max(200, ringRun.k[ringRun.k.length - 1]);
        ringErrCard.plot.opts.x = [0, xMax];
        ringStrainCard.plot.opts.x = [0, xMax];
        ringErrCard.plot.set({ series: errSeries });
        ringStrainCard.plot.set({ series: strainSeries });
        ringErrCard.setLegend(legend);
        ringStrainCard.setLegend(legend);
    }

    // ---------------------------------------------------------------- invariances logic
    async function playGestures(kinds) {
        gestureButtons.forEach((b) => { b.disabled = true; });
        traces = { points: [], edges: [], angles: [] };
        for (const [phase, kind] of ['translate', 'rotate', 'deform'].entries()) {
            if (!kinds.includes(kind)) continue;
            const { frames } = await session.call('gesture', { kind });
            await new Promise((resolve) => {
                const t0 = performance.now(), duration = 2600;
                const tick = (now) => {
                    const u = Math.min(1, Math.max(0, now - t0) / duration);
                    const index = Math.round(u * (frames.length - 1));
                    gesture = { kind, frame: frames[index] };
                    for (const r of REPS) traces[r].push({ t: phase + u, v: frames[index].features[r] });
                    drawInv();
                    drawTraces();
                    if (u < 1) requestAnimationFrame(tick); else resolve();
                };
                requestAnimationFrame(tick);
            });
        }
        gesture = null;
        drawInv();
        gestureButtons.forEach((b) => { b.disabled = false; });
    }

    function overlay(g, r, points, strong = true) {
        const p = g.p, c = repColor(p, r);
        if (r === 'points') { g.points(points, { color: c, r: 0.12, ring: p.canvas }); return; }
        if (r === 'edges') {
            for (let i = 0; i < points.length - 1; i++) g.arrow(points[i], points[i + 1], { color: c, width: 2.4, head: 9 });
            g.points(points, { color: p.canvas, r: 0.08, ring: c });
            return;
        }
        for (let i = 0; i < points.length - 1; i++) g.line([points[i], points[i + 1]], { color: alpha(c, 0.55), width: 1.6 });
        for (let i = 1; i < points.length - 1; i++) {
            const a = [points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]];
            const b = [points[i + 1][0] - points[i][0], points[i + 1][1] - points[i][1]];
            const kappa = Math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]);
            const len = Math.hypot(...a);
            g.line([points[i], [points[i][0] + 0.6 * a[0] / len, points[i][1] + 0.6 * a[1] / len]], { color: alpha(p.inkSoft, 0.6), width: 1, dash: [3, 3] });
            const r0 = (0.07 + 0.3 * Math.min(1, Math.abs(kappa))) * g.scale;
            g.disc(points[i], r0, kappa >= 0 ? alpha(c, 0.45) : alpha(p.canvas, 0.85), c, 2);
        }
        if (strong) g.points([points[0], points[points.length - 1]], { color: p.canvas, r: 0.07, ring: c });
    }

    function drawInv() {
        inv.scene.draw((g) => {
            g.background({ floor: false });
            const f = gesture?.frame;
            g.rod(f ? f.nodes : S.target.nodes, { alphaValue: f ? 1 : 1 });
            if (f) g.rod(S.target.nodes, { alphaValue: 0.18, halo: false, width: 0.06 });
            overlay(g, rep, f ? f.points : S.target.points);
            g.badge(f ? `${gesture.kind === 'deform' ? 'bend' : gesture.kind} · showing ${REP_SHORT[rep]}` : `showing ${REP_SHORT[rep]}`);
        });
    }

    function drawTraces() {
        const p = palette();
        for (const r of REPS) {
            const data = traces[r];
            const nComp = data.length ? data[0].v.length : 0;
            const nEnt = r === 'angles' ? nComp : nComp / 2;
            const series = [];
            for (let c = 0; c < nComp; c++) {
                const entry = r === 'angles' ? c : Math.floor(c / 2);
                series.push({ x: data.map((d) => d.t), y: data.map((d) => d.v[c]), color: ramp(p, r, entry, nEnt), width: 1.4,
                    dash: r !== 'angles' && c % 2 === 1 ? [4, 3] : null });
            }
            const lim = r === 'points' ? 0.7 : r === 'edges' ? 0.6 : 0.8;
            tracePlots[r].plot.opts.y = [-lim, lim];
            tracePlots[r].plot.set({ series, bands: [{ x0: 0, x1: 1, color: alpha(p.rod, 0.06) }, { x0: 1, x1: 2, color: alpha(p.gripR, 0.06) }, { x0: 2, x1: 3, color: alpha(p.aqua, 0.06) }] });
        }
    }

    // ---------------------------------------------------------------- closed loop logic
    function apply(result) {
        S = result;
        for (const s of result.samples) { run.k.push(s.k); run.err.push(s.error / S.initial_error); }
        update();
    }
    function syncButtons() {
        runButton.textContent = runner.running ? '❚❚ Pause' : '▶ Run';
        [resetButton, nullButton, compareButton].forEach((b) => { b.disabled = runner.running; });
    }

    async function playNull() {
        runner.stop();
        const { frames, null_dim: dim } = await session.call('null_motion');
        if (!dim) { nullCard.note.textContent = 'Null(J) is trivial for point coordinates: every robot motion changes the features.'; return; }
        nullCard.note.textContent = `dim Null(J) = ${dim}; motion along its first basis vector`;
        const p = palette();
        const xs = frames.map((_, i) => i / (frames.length - 1));
        nullCard.plot.set({ series: [
            { x: xs, y: frames.map((f) => f.feature_change), color: repColor(p, rep), width: 2.2 },
            { x: xs, y: frames.map((f) => f.point_change), color: alpha(p.ink, 0.6), width: 1.6, dash: [5, 4] },
        ] });
        nullCard.setLegend([{ label: `change of the ${rep} features`, color: repColor(p, rep) },
            { label: 'change of the tracked points', color: alpha(p.ink, 0.6), dash: 'dashed' }]);
        await new Promise((resolve) => {
            const t0 = performance.now(), duration = 3000;
            const tick = (now) => {
                const u = Math.min(1, Math.max(0, now - t0) / duration) * (frames.length - 1);
                const i = Math.min(frames.length - 2, Math.floor(u)), t = u - i, a = frames[i], b = frames[i + 1];
                nullFrame = { q: lerp(a.q, b.q, t), nodes: lerpNodes(a.nodes, b.nodes, t), points: lerpNodes(a.points, b.points, t) };
                drawLoop();
                if (now - t0 < duration) requestAnimationFrame(tick); else { nullFrame = null; drawLoop(); resolve(); }
            };
            requestAnimationFrame(tick);
        });
    }

    async function compare() {
        compareButton.disabled = true;
        compareCard.note.textContent = 'computing…';
        const { runs, start } = await session.call('compare');
        for (const m of minis) {
            const r = runs[m.rep];
            m.view.redraw = () => m.view.draw((g) => {
                g.background({ floor: false });
                g.rod(start, { alphaValue: 0.25, halo: false, width: 0.07 });
                g.target(S.target);
                g.rod(r.nodes, { color: repColor(g.p, m.rep) });
                g.points(r.points, { r: 0.12 });
            });
            m.view.redraw();
            m.caption.innerHTML = '';
            m.caption.append(`${m.rep}: ‖e${subscript(S.horizon)}‖/‖e₀‖ = ${r.ratio.toFixed(3)}`);
        }
        compareCard.note.textContent = 'grey: start · dashed: target';
        compareCard.el.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
        compareButton.disabled = false;
    }

    function drawLoop() {
        loopScene.scene.draw((g) => {
            g.background();
            g.target(S.target);
            const f = nullFrame;
            const q = f ? f.q : S.scene.q;
            g.arms(q);
            if (f) g.rod(S.scene.nodes, { alphaValue: 0.22, halo: false, width: 0.07 });
            g.rod(f ? f.nodes : S.scene.nodes, { color: repColor(g.p, rep) });
            overlay(g, rep, f ? f.points : S.scene.points, false);
            if (rep !== 'points') g.points(f ? f.points : S.scene.points, { r: 0.06 });
            g.grippers(q);
            if (f) g.badge('robot motion inside Null(J): the features do not change');
            else if (S.k > 0 && S.k <= S.disturbance_steps) g.badge(`disturbance pulling the grippers (iterations 1-${S.disturbance_steps})`, { color: g.p.red });
            g.corner(`k = ${S.k}`);
        });
    }

    function update() {
        drawInv();
        drawLoop();
        lesson.querySelectorAll('tr[data-rep]').forEach((tr) => tr.classList.toggle('active', tr.dataset.rep === rep));
        const p = palette();
        out.set('n', String(S.n)); out.set('rank', String(S.rank)); out.set('null', String(S.null_dim));
        out.set('k', String(S.k));
        const ratio = S.error / S.initial_error;
        out.set('err', pct(ratio, 1), ratio < 0.03 ? 'good' : '');
        errCard.plot.opts.x = [0, Math.max(S.horizon, run.k[run.k.length - 1])];
        errCard.plot.set({ series: [{ x: run.k, y: run.err, color: repColor(p, rep), width: 2.4 }],
            hlines: [{ y: 0.03, color: alpha(p.green, 0.7) }], bands: [{ x0: 0, x1: S.disturbance_steps, color: alpha(p.red, 0.08) }] });
        errCard.setLegend([{ label: REP_NAME[rep], color: repColor(p, rep) }, { label: '3% of the initial error', color: p.green, dash: 'dashed' },
            { label: 'disturbance acting', color: alpha(p.red, 0.45) }]);
        syncButtons();
    }
    update();
    drawTraces();
}

// Demo 02 - The deformation Jacobian: columns of J and the prediction ds ~ J dq.
import { DOF_COLS, FEATURE_ROWS } from '../lib/matrix.js';
import { gripperDrag, lerpNodes } from '../lib/scene.js';
import { alpha } from '../lib/theme.js';
import { button, card, h, readouts, segmented, slider, fmt, pct, DOF_TEX } from '../lib/ui.js';
import { barsCard, matrixCard, plotCard, plotsGrid, pointColors, sceneCard, POINT_LABELS } from '../lib/widgets.js';

const UNIT_STEP = [0.3, 0.3, 0.2, 0.3, 0.3, 0.2];

export async function mount({ workspace, session, state }) {
    let S = state;
    let frame = null;            // animation frame being shown

    const sc = sceneCard('Robot motion and feature response', {
        note: 'drag the grippers to change the configuration',
        guide: 'Choose a configuration, set $\\dq$ with the sliders or by clicking a Jacobian column, then play the motion. Blue predictions and green measurements separate as the step leaves the local linear regime.',
        view: { x0: -5.6, x1: 5.6, y0: -1.7, y1: 4.2 },
        legendItems: (p) => [
            { label: 'command $\\dq$', color: p.accentStrong, kind: 'line' },
            { label: 'predicted $\\vJ\\dq$', color: p.rod, kind: 'line' },
            { label: 'measured $\\ds$', color: p.aqua, kind: 'line' },
            { label: 'rod after the motion', color: p.rod, kind: 'dashed' },
        ],
    });
    const move = session.stream('move', (next) => { S = next; update(); });
    const setDq = session.stream('set_dq', (next) => { S = next; update(); });
    gripperDrag(sc.scene, {
        getQ: () => S.scene.q, getNodes: () => S.scene.nodes, enabled: () => !frame,
        onMove: (q) => move({ q }),
    });
    const presets = segmented({
        options: [{ value: 'nominal', label: 'Nominal' }, { value: 'slack', label: 'Slack' },
            { value: 'taut', label: 'Taut' }, { value: 'arch', label: 'Arch' }],
        value: 'nominal',
        onChange: async (name) => { S = await session.call('preset', { name }); update(); },
    });
    const play = button('▶ Play motion', async () => {
        play.disabled = true;
        const { frames } = await session.call('play');
        await animate(frames, 2600);
        play.disabled = false;
    }, { primary: true });
    sc.toolbar.append(presets.el, h('span', { class: 'toolbar-divider' }), play,
        button('Zero $\\dq$', () => { dqSliders.forEach((s) => s.set(0)); sendDq(); }));

    // ---- rail: J, readouts, dq sliders
    const jCard = matrixCard('Exact Jacobian $\\vJ(\\vq)$', {
        rowLabels: FEATURE_ROWS, colLabels: DOF_COLS,
        onClick: ({ c }) => { dqSliders.forEach((s, j) => s.set(j === c ? UNIT_STEP[j] : 0)); sendDq(); },
    }, { note: '10 × 6 · click a column', height: 300 });
    const out = readouts([
        { key: 'pred', label: '$\\norm{\\vJ\\dq}$' },
        { key: 'act', label: '$\\norm{\\ds}$' },
        { key: 'err', label: 'linearization error' },
    ]);
    const outCard = card('Prediction vs. response');
    outCard.body.append(out.el);
    const dqSliders = DOF_TEX.map((name, j) => slider({
        label: `$\\Delta ${name}$`, min: j % 3 === 2 ? -0.6 : -0.8, max: j % 3 === 2 ? 0.6 : 0.8, step: 0.01,
        value: S.dq[j],
        format: (v) => v.toFixed(2) + (j % 3 === 2 ? ' rad' : ''), onInput: sendDq,
    }));
    const dqCard = card('Robot increment $\\dq$', 'the six sliders set the increment applied by ▶ Play motion');
    dqCard.body.append(...dqSliders.map((s) => s.el));
    function sendDq() { setDq({ dq: dqSliders.map((s) => s.get()) }); }

    // ---- plots
    const curve = plotCard('Linearization error vs. step size', { xLabel: "step size ‖ε Δq‖", yLabel: "relative error [%]" },
        { note: '‖Δs − εJΔq‖ / ‖Δs‖', size: 'short' });
    const bars = barsCard('Measured $\\ds$ (bars) and predicted $\\vJ\\dq$ (dashed)', { labels: POINT_LABELS }, { height: 150 });

    workspace.append(
        h('div', { class: 'stage wide-rail' }, h('div', {}, sc.el, plotsGrid(bars, curve)),
            h('div', { class: 'rail' }, jCard.el, outCard.el, dqCard.el)));

    function draw() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background();
            const q = frame ? frame.q : S.scene.q;
            g.arms(q);
            if (!frame) g.rod(S.after.nodes, { color: p.rod, dash: [6, 5], halo: false, alphaValue: 0.55, width: 0.07 });
            g.rod(frame ? frame.nodes : S.scene.nodes);
            const pts = frame ? frame.points : S.scene.points;
            if (frame) {
                frame.predicted.forEach((pr, i) => g.disc(pr, 0.16 * g.scale, null, p.rod, 2));
            } else {
                S.scene.points.forEach((pt, i) => {
                    const pr = [pt[0] + S.predicted[2 * i], pt[1] + S.predicted[2 * i + 1]];
                    const ac = [pt[0] + S.actual[2 * i], pt[1] + S.actual[2 * i + 1]];
                    g.arrow(pt, pr, { color: p.rod, width: 2.2, head: 8 });
                    g.arrow(pt, ac, { color: p.aqua, width: 2.2, head: 8 });
                });
                g.command(S.scene.q, S.dq, { color: p.accentStrong });
            }
            g.points(pts);
            g.grippers(q);
            if (!frame) g.handles(q);
            if (frame) g.badge('robot motion ±Δq · rings: first-order prediction');
        });
    }

    function update() {
        draw();
        const p = sc.scene.p;
        const n = (v) => Math.hypot(...v);
        out.set('pred', fmt(n(S.predicted)));
        out.set('act', fmt(n(S.actual)));
        out.set('err', pct(S.relative_error, 1), S.relative_error > 0.25 ? 'bad' : S.relative_error < 0.05 ? 'good' : '');
        const nonzero = S.dq.map((v) => Math.abs(v) > 1e-9);
        const only = nonzero.filter(Boolean).length === 1 ? nonzero.indexOf(true) : -1;
        jCard.matrix.opts.colColors = DOF_COLS.map((_, j) => p.dof[j]);
        jCard.matrix.set(S.J, { col: only });
        bars.bars.opts.colors = pointColors(p);
        bars.bars.set(S.actual, { ghost: S.predicted });
        const current = Math.hypot(...S.dq);
        curve.plot.set({
            series: [{ x: S.curve.steps, y: S.curve.errors.map((v) => 100 * v), color: p.rod, width: 2 }],
            vlines: current > 0 ? [{ x: current, color: alpha(p.accentStrong, 0.7) }] : [],
        });
    }

    function animate(frames, duration) {
        return new Promise((resolve) => {
            const t0 = performance.now();
            const step = (now) => {
                const u = Math.min(1, Math.max(0, now - t0) / duration) * (frames.length - 1);
                const i = Math.min(frames.length - 2, Math.floor(u)), t = u - i;
                const a = frames[i], b = frames[i + 1];
                frame = {
                    q: a.q.map((v, j) => v + (b.q[j] - v) * t),
                    nodes: lerpNodes(a.nodes, b.nodes, t),
                    points: lerpNodes(a.points, b.points, t),
                    predicted: lerpNodes(a.predicted, b.predicted, t),
                };
                draw();
                if (now - t0 < duration) requestAnimationFrame(step);
                else { frame = null; draw(); resolve(); }
            };
            requestAnimationFrame(step);
        });
    }
    update();
}

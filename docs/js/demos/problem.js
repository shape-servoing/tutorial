// Demo 01 - The shape-servoing problem: s, s*, and e = s* - s by hand.
import { gripperDrag } from '../lib/scene.js';
import { alpha } from '../lib/theme.js';
import { button, card, h, readouts, toggle, fmt, pct } from '../lib/ui.js';
import { barsCard, plotCard, plotsGrid, pointColors, robotSliders, sceneCard, POINT_LABELS } from '../lib/widgets.js';

export async function mount({ workspace, session, state }) {
    let S = state;
    let showNodes = false;
    let active = -1;
    let corrections = 0;
    const history = [];
    let eRange = 0;        // error-bar scale: largest |e_i| since the last target or reset

    // ---- scene
    const sc = sceneCard('Robots, rod, and target', {
        note: 'drag a gripper, its ring handle, or the rod',
        guide: 'Drag a gripper to translate it, drag its ring handle to rotate it, or drag the rod to move both grippers. Watch the error arrows and $\\norm{\\ve}$ respond.',
        legendItems: (p) => [
            { label: 'rod $\\vx$', color: p.rod, kind: 'line' },
            { label: 'tracked points $\\vs$', color: p.feature, kind: 'dot' },
            { label: 'target $\\vs^{*}$', color: p.target, kind: 'dashed' },
            { label: 'error $\\ve$ (per point)', color: p.rod, kind: 'line' },
        ],
    });
    const move = session.stream('move', (next) => { S = next; update(); });
    gripperDrag(sc.scene, {
        getQ: () => S.scene.q, getNodes: () => S.scene.nodes,
        onMove: (q) => move({ q }),
        onStart: (handle) => { active = handle.side; },
        onEnd: () => { active = -1; corrections += 1; update(); },
    });
    const nodesToggle = toggle({ label: 'show all 25 nodes of $\\vx$', onChange: (v) => { showNodes = v; draw(); } });
    sc.toolbar.append(
        button('New target', async () => { S = await session.call('new_target'); history.length = 0; corrections = 0; eRange = 0; update(); }),
        button('Reset', async () => { S = await session.call('reset'); history.length = 0; corrections = 0; eRange = 0; update(); }),
        h('span', { class: 'toolbar-divider' }), nodesToggle.el);

    // ---- rail
    const sliders = robotSliders(S.scene.q, (q) => move({ q }));
    const robotCard = card('Robot coordinates $\\vq$', 'sliders are an alternative to dragging · m = 6');
    robotCard.body.append(sliders.el);
    const errorCard = card('Shape error', 'n = 10');
    const out = readouts([
        { key: 'norm', label: '$\\norm{\\ve}$' },
        { key: 'ratio', label: '$\\norm{\\ve}/\\norm{\\ve_0}$' },
        { key: 'moves', label: 'corrections' },
    ]);
    errorCard.body.append(out.el);

    // ---- plots
    const featureBars = barsCard('Features $\\vs$ (bars) and target $\\vs^{*}$ (dashed)', { labels: POINT_LABELS, yTicks: true }, { height: 140 });
    const errorBars = barsCard('Shape error $\\ve=\\vs^{*}-\\vs$', { labels: POINT_LABELS, yTicks: true }, { height: 140 });
    const errorPlot = plotCard('Error during your corrections', { xLabel: 'interaction samples', yLabel: '‖e‖ / ‖e₀‖', y: [0, 1.6] },
        { size: 'short' });

    workspace.append(
        h('div', { class: 'stage' }, sc.el, h('div', { class: 'rail' }, errorCard.el, robotCard.el)),
        plotsGrid(featureBars, errorBars, errorPlot));

    function draw() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background();
            g.target(S.target);
            g.arms(S.scene.q);
            g.rod(S.scene.nodes);
            if (showNodes) S.scene.nodes.forEach((n) => g.disc(n, 3, p.canvas, p.rod, 1.2));
            S.scene.points.forEach((pt, i) => {
                g.arrow(pt, S.target.points[i], { color: alpha(p.rod, 0.75), width: 1.6, head: 7 });
                g.text(`p${i + 1}`, [pt[0], pt[1] - 0.42], { color: p.muted, size: 10 });
            });
            g.points(S.scene.points);
            g.grippers(S.scene.q, { active });
            g.handles(S.scene.q, { active });
        });
    }

    function update() {
        draw();
        sliders.set(S.scene.q);
        const ratio = S.error_norm / S.initial_error;
        out.set('norm', fmt(S.error_norm, 3));
        out.set('ratio', pct(ratio), ratio < 0.1 ? 'good' : '');
        out.set('moves', String(corrections));
        const p = sc.scene.p;
        featureBars.bars.opts.colors = pointColors(p);
        errorBars.bars.opts.colors = pointColors(p);
        featureBars.bars.set(S.s, { ghost: S.s_star });
        eRange = Math.max(eRange, ...S.e.map(Math.abs));
        errorBars.bars.opts.range = [-eRange, eRange];
        errorBars.bars.set(S.e);
        history.push(ratio);
        if (history.length > 400) history.shift();
        errorPlot.plot.set({
            series: [{ x: history.map((_, i) => i), y: history, color: p.rod, width: 2 }],
            hlines: [{ y: 0.1, color: alpha(p.green, 0.7), dash: [4, 4] }],
        });
    }
    update();
}

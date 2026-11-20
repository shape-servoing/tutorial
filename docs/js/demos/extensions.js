// Demo 11 - Extensions of the shape-servoing loop: feedforward tracking, null-space secondary tasks.
import { Runner } from '../lib/runner.js';
import { dragController } from '../lib/scene.js';
import { alpha, mix, palette } from '../lib/theme.js';
import { button, card, h, readouts, slider, toggle, fmt } from '../lib/ui.js';
import { plotCard, sceneCard } from '../lib/widgets.js';

// Axial strain in percent; tiny compressions are shown as 0.0%.
const strainText = (v) => `${Math.max(0, 100 * v).toFixed(1)}%`;

export async function mount({ workspace, session, state }) {
    let T = state.tracking, D = state.secondary, C = state.combined;
    const panels = {};
    const tabs = h('div', { class: 'tabs', role: 'tablist' });
    const runners = [];
    function showTab(id) {
        runners.forEach((r) => r.stop());
        tabs.querySelectorAll('button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.tab === id)));
        Object.entries(panels).forEach(([k, el]) => { el.style.display = k === id ? '' : 'none'; });
        drawTrack(); drawSecondary(); drawCombined();
    }
    for (const [id, title] of [['track', 'Moving target'], ['secondary', 'Secondary task in Null(Ĵ)'],
        ['combined', 'Shape and transport']]) {
        const b = h('button', { type: 'button', role: 'tab', 'data-tab': id }, title);
        b.addEventListener('click', () => showTab(id));
        tabs.append(b);
    }

    // =============================================================== tracking
    let trackRun = { k: [], err: [], ff: T.params.feedforward };
    const pastTrack = [];
    const trackScene = sceneCard('Tracking a swaying target', {
        guide: 'Run once with feedforward on, reset, then run with it off. Without feedforward—and especially at low gain—the blue rod visibly lags behind the dashed moving target.',
        legendItems: (p) => [{ label: 'rod', color: p.rod, kind: 'line' }, { label: 'moving target $\\vs^{*}(t)$', color: p.target, kind: 'dashed' }] });
    const trackRunner = new Runner(session, { action: 'track', rate: 30, onResult: applyTrack, onStop: syncTrack });
    runners.push(trackRunner);
    const trackButton = button('▶ Run', () => { if (trackRunner.running) trackRunner.stop(); else trackRunner.start(); syncTrack(); }, { primary: true });
    const trackReset = button('Reset', async () => { trackRunner.stop(); archiveTrack(); T = (await session.call('reset_tracking')).tracking; drawTrack(); plotTrack(); });
    trackScene.toolbar.append(trackButton, trackReset);
    const setTracking = session.stream('set_tracking', (r) => { T = r.tracking; plotTrack(); });
    const ff = toggle({ label: 'feedforward $\\vJh_k^{+}\\dot{\\vs}^{*}$', description: 'On: anticipate target motion. Off: react only to accumulated shape error.', value: true, onChange: (v) => setTracking({ feedforward: v }) });
    const gain = slider({ label: 'gain $\\alpha$', description: 'Higher feedback gain reduces lag but cannot remove it completely without feedforward.', min: 0.02, max: 0.6, value: T.params.alpha, onInput: (v) => setTracking({ alpha: v }) });
    const trackCard = card('Controller');
    const trackOut = readouts([{ key: 'k', label: 'iteration $k$' }, { key: 'err', label: 'tracking error $\\norm{\\ve_k}$' }]);
    trackCard.body.append(ff.el, gain.el, trackOut.el);
    const trackPlot = plotCard('Tracking error', { xLabel: 'iteration k', yLabel: '‖e_k‖', logY: true, y: [1e-4, 2] }, { size: 'tall' });
    panels.track = h('div', {}, h('div', { class: 'stage' }, trackScene.el, h('div', { class: 'rail' }, trackCard.el)),
        h('section', { class: 'plots' }, trackPlot.el));

    function archiveTrack() {
        if (trackRun.k.length > 5) { pastTrack.unshift(trackRun); pastTrack.length = Math.min(pastTrack.length, 2); }
        trackRun = { k: [], err: [], ff: T.params.feedforward, alpha: T.params.alpha };
    }
    function applyTrack(r) {
        T = r.tracking;
        for (const s of r.samples) { trackRun.k.push(s.k); trackRun.err.push(s.error); }
        trackRun.ff = T.params.feedforward; trackRun.alpha = T.params.alpha;
        drawTrack(); plotTrack();
    }
    function syncTrack() { trackButton.textContent = trackRunner.running ? '❚❚ Pause' : '▶ Run'; trackReset.disabled = trackRunner.running; }
    function drawTrack() {
        trackScene.scene.draw((g) => {
            g.background();
            g.target(T.target);
            g.arms(T.scene.q);
            g.rod(T.scene.nodes);
            g.points(T.scene.points);
            g.grippers(T.scene.q);
            g.corner(`k = ${T.k}`);
        });
        trackOut.set('k', String(T.k));
        trackOut.set('err', fmt(T.error, 4));
    }
    function plotTrack() {
        const p = palette();
        const colorOf = (r) => (r.ff ? p.rod : p.red);
        const series = [], legend = [];
        pastTrack.slice().reverse().forEach((r) => {
            const c = mix(colorOf(r), p.canvas, 0.5);
            series.push({ x: r.k, y: r.err, color: c, width: 1.5 });
            legend.push({ label: `earlier: ${r.ff ? 'with' : 'without'} feedforward, $\\alpha=${(r.alpha ?? 0).toFixed(2)}$`, color: c });
        });
        series.push({ x: trackRun.k, y: trackRun.err, color: colorOf(T.params), width: 2.4 });
        legend.push({ label: `this run: ${T.params.feedforward ? 'with' : 'without'} feedforward`, color: colorOf(T.params) });
        trackPlot.plot.opts.x = [0, Math.max(240, T.k)];
        trackPlot.plot.set({ series });
        trackPlot.setLegend(legend);
    }

    // =============================================================== secondary task
    let secRun = { k: [], shape: [], dist: [] };
    const secScene = sceneCard('Hold the shape, move the left gripper to the marker', {
        note: 'drag the green marker',
        guide: 'Drag the green marker to set the left-gripper target, then run. With projection on, the gripper approaches it through $\\Null(\\vJh)$ while the shape, measured by curvature-based features, stays regulated.',
        legendItems: (p) => [{ label: 'rod (curvature-based features)', color: p.repAngles, kind: 'line' }, { label: 'marker', color: p.green, kind: 'ring' }] });
    const secRunner = new Runner(session, { action: 'secondary', rate: 30, onResult: applySecondary, onStop: syncSecondary });
    runners.push(secRunner);
    const secButton = button('▶ Run', () => { if (secRunner.running) secRunner.stop(); else secRunner.start(); syncSecondary(); }, { primary: true });
    const secReset = button('Reset', async () => { secRunner.stop(); D = (await session.call('reset_secondary')).secondary; secRun = { k: [], shape: [], dist: [] }; drawSecondary(); plotSecondary(); });
    secScene.toolbar.append(secButton, secReset);
    const moveMarker = session.stream('set_marker', (r) => { D = r.secondary; drawSecondary(); });
    dragController(secScene.scene, {
        hit: (w) => (Math.hypot(w[0] - D.marker[0], w[1] - D.marker[1]) < 0.4 ? { mode: 'marker' } : null),
        move: (_, w) => { D.marker = w; drawSecondary(); moveMarker({ xy: w }); },
    });
    const project = toggle({ label: 'project through $\\mathbf{I}-\\vJh_k^{+}\\vJh_k$', description: 'On: protect the shape task. Off: the secondary command deforms the rod, so shape error rises.', value: true,
        onChange: async (v) => { D = (await session.call('set_projection', { project: v })).secondary; drawSecondary(); } });
    const secCard = card('Secondary task');
    const secOut = readouts([{ key: 'shape', label: 'shape error $\\norm{\\ve_k}$' }, { key: 'dist', label: 'marker error $\\norm{\\ve_{\\mathrm{m},k}}$' }]);
    secCard.body.append(project.el, secOut.el);
    const secPlot = plotCard('Shape error and marker error', { xLabel: 'iteration k', yLabel: '', logY: true, y: [1e-5, 10] }, { size: 'tall' });
    panels.secondary = h('div', { style: 'display:none' }, h('div', { class: 'stage' }, secScene.el, h('div', { class: 'rail' }, secCard.el)),
        h('section', { class: 'plots' }, secPlot.el));

    function applySecondary(r) {
        D = r.secondary;
        for (const s of r.samples) { secRun.k.push(s.k); secRun.shape.push(s.shape_error); secRun.dist.push(s.distance); }
        if (secRun.k.length > 1500) { secRun.k.splice(0, 300); secRun.shape.splice(0, 300); secRun.dist.splice(0, 300); }
        drawSecondary(); plotSecondary();
    }
    function syncSecondary() { secButton.textContent = secRunner.running ? '❚❚ Pause' : '▶ Run'; secReset.disabled = secRunner.running; }
    function drawSecondary() {
        secScene.scene.draw((g) => {
            const p = g.p;
            g.background();
            g.arms(D.scene.q);
            g.rod(D.scene.nodes, { color: p.repAngles });
            g.points(D.scene.points, { r: 0.08 });
            g.grippers(D.scene.q);
            g.line([[D.scene.q[0], D.scene.q[1]], D.marker], { color: alpha(p.green, 0.6), width: 1.4, dash: [4, 4] });
            g.disc(D.marker, 0.22 * g.scale, alpha(p.green, 0.18), p.green, 2.4);
            g.badge(D.project ? 'u₂ projected onto Null(Ĵ)' : 'u₂ added without projection');
            g.corner(`k = ${D.k}`);
        });
        secOut.set('shape', fmt(D.shape_error, 4), D.shape_error < 0.01 ? 'good' : 'bad');
        secOut.set('dist', fmt(D.distance, 3));
    }
    function plotSecondary() {
        const p = palette();
        const k0 = secRun.k[0] ?? 0;
        secPlot.plot.opts.x = [k0, Math.max(k0 + 200, secRun.k[secRun.k.length - 1] ?? 0)];
        secPlot.plot.set({ series: [{ x: secRun.k, y: secRun.shape, color: p.repAngles, width: 2.2 },
            { x: secRun.k, y: secRun.dist, color: p.green, width: 1.8, dash: [5, 4] }] });
        secPlot.setLegend([{ label: 'shape error $\\norm{\\ve_k}$ (curvature)', color: p.repAngles }, { label: 'marker error $\\norm{\\ve_{\\mathrm{m},k}}$', color: p.green, dash: 'dashed' }]);
    }

    // =============================================================== two feature maps
    let combRun = { k: [], angle: [], point: [], transport: C.transport };
    const pastComb = [];
    const combScene = sceneCard('Unloop with curvature-based features, move with point coordinates', {
        guide: 'Run with transport on, reset, then run with it off. Curvature-based features open the loop in both cases; only the point-coordinate task, projected onto $\\Null(\\vJh)$ of the curvature estimate, brings the rod to the target pose.',
        legendItems: (p) => [{ label: 'rod (curvature-based features)', color: p.repAngles, kind: 'line' },
            { label: 'target (unlooped, moved)', color: p.target, kind: 'dashed' }] });
    const combRunner = new Runner(session, { action: 'combined', rate: 30, onResult: applyCombined, onStop: syncCombined });
    runners.push(combRunner);
    const combButton = button('▶ Run', () => { if (combRunner.running) combRunner.stop(); else combRunner.start(); syncCombined(); }, { primary: true });
    const combReset = button('Reset', async () => {
        combRunner.stop(); archiveCombined();
        C = (await session.call('reset_combined')).combined; drawCombined(); plotCombined();
    });
    combScene.toolbar.append(combButton, combReset);
    const transport = toggle({ label: 'transport $(\\mathbf{I}-\\vJh_k^{+}\\vJh_k)\\,\\beta\\,\\vJh_{\\mathrm{p},k}^{+}\\ve_{\\mathrm{p},k}$',
        description: 'On: a point-coordinate secondary task moves the rod within the null space of the curvature estimate. Off: curvature-based features only.',
        value: C.transport, onChange: async (v) => { C = (await session.call('set_transport', { transport: v })).combined; drawCombined(); } });
    const combCard = card('Two feature maps');
    const combOut = readouts([{ key: 'k', label: 'iteration $k$' }, { key: 'angle', label: 'curvature error' },
        { key: 'point', label: 'point-coordinate error' }, { key: 'strain', label: 'largest axial strain' }]);
    combCard.body.append(transport.el, combOut.el);
    const combPlot = plotCard('Errors of the two feature maps', { xLabel: 'iteration k', yLabel: '', logY: true, y: [1e-10, 20] }, { size: 'tall' });
    panels.combined = h('div', { style: 'display:none' }, h('div', { class: 'stage' }, combScene.el, h('div', { class: 'rail' }, combCard.el)),
        h('section', { class: 'plots' }, combPlot.el));

    function archiveCombined() {
        if (combRun.k.length > 5) { pastComb.unshift(combRun); pastComb.length = Math.min(pastComb.length, 1); }
        combRun = { k: [], angle: [], point: [], transport: C.transport };
    }
    function applyCombined(r) {
        C = r.combined;
        for (const s of r.samples) { combRun.k.push(s.k); combRun.angle.push(s.angle_error); combRun.point.push(s.point_error); }
        combRun.transport = C.transport;
        if (C.k >= 400) combRunner.stop();
        drawCombined(); plotCombined();
    }
    function syncCombined() { combButton.textContent = combRunner.running ? '❚❚ Pause' : '▶ Run'; combReset.disabled = combRunner.running; }
    function drawCombined() {
        combScene.scene.draw((g) => {
            const p = g.p;
            g.background();
            g.target(C.target);
            g.arms(C.scene.q);
            g.rod(C.scene.nodes, { color: p.repAngles });
            g.points(C.scene.points, { r: 0.08 });
            g.grippers(C.scene.q);
            g.badge(C.transport ? 'curvature + projected point-coordinate task' : 'curvature only');
            g.corner(`k = ${C.k}`);
        });
        combOut.set('k', String(C.k));
        combOut.set('angle', fmt(C.angle_error, 4));
        combOut.set('point', fmt(C.point_error, 4), C.point_error < 0.01 ? 'good' : '');
        combOut.set('strain', strainText(C.strain));
    }
    function plotCombined() {
        const p = palette();
        const colorOf = (r) => (r.transport ? p.rod : p.red);
        const series = [], legend = [];
        pastComb.slice().reverse().forEach((r) => {
            const c = mix(colorOf(r), p.canvas, 0.5);
            series.push({ x: r.k, y: r.angle, color: c, width: 1.5 }, { x: r.k, y: r.point, color: c, width: 1.4, dash: [5, 4] });
            legend.push({ label: `earlier: ${r.transport ? 'with' : 'without'} transport`, color: c });
        });
        series.push({ x: combRun.k, y: combRun.angle, color: colorOf(combRun), width: 2.2 },
            { x: combRun.k, y: combRun.point, color: colorOf(combRun), width: 1.8, dash: [5, 4] });
        legend.push({ label: `this run: ${combRun.transport ? 'with' : 'without'} transport (solid: curvature, dashed: points)`, color: colorOf(combRun) });
        combPlot.plot.opts.x = [0, Math.max(300, C.k)];
        combPlot.plot.set({ series });
        combPlot.setLegend(legend);
    }

    workspace.append(tabs, panels.track, panels.secondary, panels.combined);
    showTab('track');
    plotTrack(); plotSecondary(); plotCombined();
}

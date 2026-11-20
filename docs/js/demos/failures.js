// Demo 10 - Failure modes of the loop: controllability, observability, mechanical instability.
import { BarChart } from '../lib/bars.js';
import { lerp, lerpNodes } from '../lib/scene.js';
import { alpha, palette } from '../lib/theme.js';
import { button, card, h, label, readouts, segmented, slider, fmt, pct, DOF_TEXT } from '../lib/ui.js';
import { plotCard, sceneCard } from '../lib/widgets.js';

export async function mount({ workspace, session, state, lesson }) {
    let S = state;
    const panels = {};
    const tabs = h('div', { class: 'tabs', role: 'tablist' });
    const tabNames = [['ctrl', 'Loss of controllability'], ['obs', 'Loss of observability'], ['inst', 'Mechanical instability']];
    function showTab(id) {
        tabs.querySelectorAll('button').forEach((b) => b.setAttribute('aria-selected', String(b.dataset.tab === id)));
        Object.entries(panels).forEach(([k, el]) => { el.style.display = k === id ? '' : 'none'; });
        lesson.querySelectorAll('tr[data-tab]').forEach((tr) => tr.classList.toggle('active', tr.dataset.tab === id));
        redrawAll();
    }
    for (const [id, title] of tabNames) {
        const b = h('button', { type: 'button', role: 'tab', 'data-tab': id }, title);
        b.addEventListener('click', () => showTab(id));
        tabs.append(b);
    }

    // =============================================================== controllability
    const ctrlScene = sceneCard('Pulling the rod taut', { guide: 'Increase extension to weaken a controllable direction; raise damping to cap the inverse gain commanded along weak directions.', view: { x0: -5.4, x1: 5.4, y0: -1.2, y1: 3.4 }, camera: false,
        legendItems: (p) => [{ label: 'rod', color: p.rod, kind: 'line' }, { label: 'weakest feature direction $\\vphi_{\\min}$', color: p.red, kind: 'line' }] });
    const setTaut = session.stream('taut', (r) => { S.taut = r.taut; drawCtrl(); });
    const setMu = session.stream('set_mu', (r) => { S.taut = r.taut; drawCtrl(); });
    const ext = slider({ label: 'extension beyond the rest length', min: 0, max: 20, step: 0.5, value: 0, format: (v) => `${v.toFixed(1)}%`,
        onInput: (v) => setTaut({ extension: v }) });
    const mu = slider({ label: 'damping $\\mu$', min: 0.005, max: 0.3, value: 0.05, log: true, format: (v) => v.toFixed(3),
        onInput: (v) => setMu({ mu: v }) });
    const ctrlCard = card('Exact Jacobian at this extension');
    const ctrlOut = readouts([{ key: 'smin', label: '$\\sigma_{\\min}$' }, { key: 'cond', label: '$\\cond$' }, { key: 'wrist', label: 'wrist share of $\\vpsi_{\\min}$' },
        { key: 'cap', label: 'damped cap $1/(2\\mu)$' }]);
    ctrlCard.body.append(ext.el, mu.el, ctrlOut.el);
    const sweepPlot = plotCard('Relative to the rest length', { xLabel: 'extension [%]', yLabel: 'ratio', logY: true, x: [0, 20], y: [0.03, 5] }, { size: 'tall' });
    const psiCard = card('Weakest robot direction $\\vpsi_{\\min}$');
    const psiCanvas = h('canvas', { class: 'bars-canvas', style: 'height:150px' });
    psiCard.body.append(psiCanvas);
    const psiBars = new BarChart(psiCanvas, { labels: DOF_TEXT, range: [-1, 1], colors: (i) => palette().dof[i] });
    const gainCard = card('Inverted gains: pseudoinverse $1/\\sigma_l$ (dashed) vs. damped');
    const gainCanvas = h('canvas', { class: 'bars-canvas', style: 'height:150px' });
    gainCard.body.append(gainCanvas);
    const gainBars = new BarChart(gainCanvas, { labels: ['σ1', 'σ2', 'σ3', 'σ4', 'σ5', 'σ6'], logY: true, range: [0.1, 100] });
    panels.ctrl = h('div', {}, h('div', { class: 'stage' }, ctrlScene.el, h('div', { class: 'rail' }, ctrlCard.el)),
        h('section', { class: 'plots' }, sweepPlot.el, psiCard.el, gainCard.el));

    function drawCtrl() {
        const T = S.taut, p = palette();
        ctrlScene.scene.draw((g) => {
            g.background({ floor: false });
            g.arms(T.scene.q);
            g.rod(T.scene.nodes);
            T.scene.points.forEach((pt, i) => g.arrow(pt, [pt[0] + T.phi_min[2 * i], pt[1] + T.phi_min[2 * i + 1]], { color: p.red, width: 2, head: 7 }));
            g.points(T.scene.points);
            g.grippers(T.scene.q);
            const [l, r] = [[T.scene.q[0], T.scene.q[1] + 0.5], [T.scene.q[3], T.scene.q[4] + 0.5]];
            g.arrow(l, [l[0] - 0.8, l[1]], { color: p.red, width: 2.4 }); g.arrow(r, [r[0] + 0.8, r[1]], { color: p.red, width: 2.4 });
            g.badge(`extension ${T.extension.toFixed(1)}%`);
        });
        ctrlOut.set('smin', fmt(T.sigma[5], 3));
        ctrlOut.set('cond', fmt(T.sigma[0] / T.sigma[5], 1));
        ctrlOut.set('wrist', pct(T.wrist_share, 0));
        ctrlOut.set('cap', fmt(1 / (2 * T.mu), 1));
        const sw = S.sweep, x = sw.extension;
        const rel = (arr) => arr.map((v) => v / arr[0]);
        const cur = sw.extension.reduce((best, v, i) => (Math.abs(v - T.extension) < Math.abs(sw.extension[best] - T.extension) ? i : best), 0);
        sweepPlot.plot.set({
            series: [{ x, y: rel(sw.condition), color: p.series[2], width: 2 }, { x, y: rel(sw.sigma_min), color: p.series[0], width: 2 },
                { x, y: rel(sw.manipulability), color: p.series[1], width: 2 }],
            markers: [{ x: x[cur], y: rel(sw.condition)[cur], color: p.series[2] }, { x: x[cur], y: rel(sw.sigma_min)[cur], color: p.series[0] },
                { x: x[cur], y: rel(sw.manipulability)[cur], color: p.series[1] }],
            hlines: [{ y: 1, color: alpha(p.ink, 0.25) }],
        });
        sweepPlot.setLegend([{ label: '$\\cond$', color: p.series[2] }, { label: '$\\sigma_{\\min}$', color: p.series[0] }, { label: '$w$', color: p.series[1] }]);
        psiBars.set(T.psi_min);
        gainBars.opts.colors = () => p.accentStrong;
        gainBars.set(T.damped_gain, { ghost: T.pinv_gain });
    }

    // =============================================================== observability
    const obsScene = sceneCard('Measured response to three robot motions', { guide: 'Play each labeled motion. The first two have different physical causes but both produce almost no measured feature change.', view: { x0: -5.4, x1: 5.4, y0: -1.6, y1: 3.4 }, camera: false,
        legendItems: (p) => [{ label: 'rod', color: p.rod, kind: 'line' }, { label: 'tracked points', color: p.feature, kind: 'dot' }] });
    const results = {};
    const scenarios = [['controllability', 'Taut rod, wrist motion (points)'], ['observability', 'Rigid translation (curvature-based features)'], ['healthy', 'Lift the left gripper (points)']];
    let obsFrame = null, obsKind = null;
    const obsButtons = scenarios.map(([kind, text]) => button(text, () => observe(kind)));
    obsScene.toolbar.append(...obsButtons);
    const obsPlot = plotCard('Along the motion', { xLabel: 'motion phase', yLabel: 'norm of the change', x: [0, 1] }, { size: 'tall' });
    const ratioCard = card('Measured response ratio $\\norm{\\ds}/\\norm{\\dq}$');
    const ratioCanvas = h('canvas', { class: 'bars-canvas', style: 'height:150px' });
    ratioCard.body.append(ratioCanvas, h('p', { class: 'hint' }, 'Each motion fills its own bar; the one played last is highlighted. Near zero for both failures: the pair (Δq, Δs) alone does not distinguish them.'));
    const ratioBars = new BarChart(ratioCanvas, { labels: ['controllability', 'observability', 'healthy'], range: [0, 1.2], yTicks: true });
    panels.obs = h('div', { style: 'display:none' }, obsScene.el, h('section', { class: 'plots' }, obsPlot.el, ratioCard.el));

    async function observe(kind) {
        obsButtons.forEach((b) => { b.disabled = true; });
        const r = await session.call('observe', { scenario: kind });
        results[kind] = r.ratio; obsKind = r;
        const p = palette();
        const xs = r.frames.map((_, i) => i / (r.frames.length - 1));
        obsPlot.plot.set({ series: [{ x: xs, y: r.frames.map((f) => f.dq), color: p.accentStrong, width: 2 },
            { x: xs, y: r.frames.map((f) => f.ds), color: p.rod, width: 2.4 }] });
        obsPlot.setLegend([{ label: 'robot motion $\\norm{\\dq}$', color: p.accentStrong }, { label: `feature change $\\norm{\\ds}$ (${r.feature_map})`, color: p.rod }]);
        ratioBars.opts.colors = (i) => [p.red, p.red, p.green][i];
        const kinds = ['controllability', 'observability', 'healthy'];
        ratioBars.set(kinds.map((k) => results[k] ?? NaN), { highlight: kinds.indexOf(kind) });
        await new Promise((resolve) => {
            const t0 = performance.now(), duration = 2600, frames = r.frames;
            const tick = (now) => {
                const u = Math.min(1, Math.max(0, now - t0) / duration) * (frames.length - 1);
                const i = Math.min(frames.length - 2, Math.floor(u)), t = u - i, a = frames[i], b = frames[i + 1];
                obsFrame = { q: lerp(a.q, b.q, t), nodes: lerpNodes(a.nodes, b.nodes, t), points: lerpNodes(a.points, b.points, t), start: frames[0] };
                drawObs();
                if (now - t0 < duration) requestAnimationFrame(tick); else resolve();
            };
            requestAnimationFrame(tick);
        });
        obsButtons.forEach((b) => { b.disabled = false; });
    }

    function drawObs() {
        obsScene.scene.draw((g) => {
            g.background({ floor: false });
            if (!obsFrame) { g.badge('select a motion'); return; }
            g.arms(obsFrame.q);
            g.rod(obsFrame.start.nodes, { alphaValue: 0.22, halo: false, width: 0.07 });
            g.rod(obsFrame.nodes);
            g.points(obsFrame.points);
            g.grippers(obsFrame.q);
            g.badge(`${obsKind.scenario} · features: ${obsKind.feature_map} · ‖Δs‖/‖Δq‖ ≈ ${obsKind.ratio.toFixed(3)}`);
        });
    }

    // =============================================================== instability
    let instMode = 'compress';
    const instScene = sceneCard('Loading the rod', { guide: 'Choose compression or wrist rotation, then move its slider slowly through the critical point. Watch the Hessian eigenvalue and response ratio for loss of equilibrium stability.', view: { x0: -5.4, x1: 5.4, y0: -1.6, y1: 3.8 }, camera: false,
        legendItems: (p) => [{ label: 'equilibrium', color: p.rod, kind: 'line' }, { label: 'straight rod', color: p.red, kind: 'dashed' }] });
    const modeChoice = segmented({ options: [{ value: 'compress', label: 'Compression (buckling)' }, { value: 'snap', label: 'Wrist rotation (snap-through)' }],
        value: 'compress', onChange: (v) => { instMode = v; comp.el.style.display = v === 'compress' ? '' : 'none'; wrist.el.style.display = v === 'snap' ? '' : 'none'; drawInst(); } });
    const resetInst = button('Reset', async () => { const r = await session.call('reset_instability'); S.compress = r.compress; S.snap = r.snap; comp.set(-5); wrist.set(0.8); drawInst(); });
    instScene.toolbar.append(modeChoice.el, resetInst);
    const setComp = session.stream('compress', (r) => { S.compress = r.compress; drawInst(); });
    const setWrist = session.stream('wrist', (r) => { S.snap = r.snap; drawInst(); });
    const comp = slider({ label: 'compression', min: -10, max: 20, step: 0.25, value: -5, format: (v) => `${v.toFixed(2)}%`, onInput: (v) => setComp({ compression: v }) });
    const wrist = slider({ label: 'wrist tilt $\\theta_L=-\\theta_R$', min: -0.8, max: 0.8, step: 0.01, value: 0.8, format: (v) => `${v.toFixed(2)} rad`,
        onInput: (v) => setWrist({ theta: v }) });
    wrist.el.style.display = 'none';
    const instCard = card('Stability of the equilibrium');
    const instOut = readouts([{ key: 'lam', label: '$\\lambda_{\\min}(\\boldsymbol{\\Pi}_{\\vx\\vx})$, equilibrium' },
        { key: 'lamS', label: 'straight rod' }, { key: 'ratio', label: 'last $\\norm{\\ds}/\\norm{\\dq}$' }]);
    instCard.body.append(comp.el, wrist.el, instOut.el, h('p', { class: 'hint' }, label('The red dashed straight rod is the ideal reference whose $\\lambda_{\\min}$ crosses zero at 5.8% compression. The simulated rod sags under gravity, so it bends gradually instead of buckling suddenly; the abrupt transition appears with the wrist rotation (snap-through).')));
    const lamPlot = plotCard('Smallest Hessian eigenvalue', { xLabel: 'compression [%]', yLabel: 'λ_min(Π_xx)' }, { size: 'tall' });
    const ratioPlot = plotCard('Response ratio along the wrist-angle path', { xLabel: 'step', yLabel: '‖Δs‖ / ‖Δq‖', logY: true }, { size: 'tall' });
    panels.inst = h('div', { style: 'display:none' }, h('div', { class: 'stage' }, instScene.el, h('div', { class: 'rail' }, instCard.el)),
        h('section', { class: 'plots' }, lamPlot.el, ratioPlot.el));

    function drawInst() {
        const p = palette();
        if (instMode === 'compress') {
            const C = S.compress, B = S.buckling;
            instScene.scene.draw((g) => {
                g.background({ floor: false });
                g.arms(C.scene.q);
                g.rod(C.straight, { color: p.red, dash: [6, 5], halo: false, width: 0.05 });
                g.rod(C.scene.nodes);
                g.grippers(C.scene.q);
                const [l, r] = [[C.scene.q[0] - 0.9, C.scene.q[1] + 0.5], [C.scene.q[3] + 0.9, C.scene.q[4] + 0.5]];
                g.arrow(l, [l[0] + 0.8, l[1]], { color: p.red, width: 2.4 }); g.arrow(r, [r[0] - 0.8, r[1]], { color: p.red, width: 2.4 });
                g.badge(`compression ${C.compression.toFixed(2)}% · straight rod ${C.lambda_straight > 0 ? 'stable' : 'unstable'}`);
            });
            instOut.set('lam', fmt(C.lambda_equilibrium, 3), C.lambda_equilibrium > 0 ? 'good' : 'bad');
            instOut.set('lamS', fmt(C.lambda_straight, 3), C.lambda_straight > 0 ? 'good' : 'bad');
            instOut.set('ratio', C.ratio === null ? '–' : fmt(C.ratio, 2));
            lamPlot.plot.opts.xLabel = 'compression [%]';
            lamPlot.plot.opts.x = [-10, 20];
            lamPlot.plot.set({
                series: [{ x: B.compression, y: B.lambda, color: p.series[0], width: 2.2 }],
                markers: [{ x: B.onset, y: 0, color: p.red, r: 5 }, { x: C.compression, y: C.lambda_straight, color: p.ink, r: 4 }],
                bands: [{ x0: B.onset, x1: 20, color: alpha(p.red, 0.08) }], hlines: [{ y: 0, color: alpha(p.ink, 0.4), dash: [] }], vlines: [],
            });
            lamPlot.setLegend([{ label: 'straight rod', color: p.series[0] }, { label: `onset ${B.onset.toFixed(1)}%`, color: p.red }]);
            ratioPlot.plot.set({ series: [] });
            ratioPlot.note.textContent = 'switch to wrist rotation';
        } else {
            const P = S.snap.path;
            instScene.scene.draw((g) => {
                g.background({ floor: false });
                g.arms(S.snap.scene.q);
                g.rod(S.snap.scene.nodes);
                g.grippers(S.snap.scene.q);
                g.command(S.snap.scene.q, [0, 0, -0.6, 0, 0, 0.6], { color: alpha(p.red, 0.8) });   // wrists tilt down
                g.badge(`θ = ${P.theta[P.theta.length - 1].toFixed(2)} rad`);
            });
            const last = P.lambda[P.lambda.length - 1], ratio = P.ratio[P.ratio.length - 1];
            instOut.set('lam', fmt(last, 3), last > 0.1 ? 'good' : 'bad');
            instOut.set('lamS', '–');
            instOut.set('ratio', ratio === null ? '–' : fmt(ratio, 2), ratio > 5 ? 'bad' : '');
            lamPlot.plot.opts.xLabel = 'wrist-angle step';
            const steps = P.theta.map((_, i) => i);
            lamPlot.plot.opts.x = [0, Math.max(10, steps.length - 1)];
            lamPlot.plot.set({ series: [{ x: steps, y: P.lambda, color: p.series[0], width: 2.2 }], markers: [], bands: [],
                hlines: [{ y: 0, color: alpha(p.ink, 0.4), dash: [] }] });
            lamPlot.setLegend([{ label: 'current equilibrium', color: p.series[0] }]);
            ratioPlot.note.textContent = '';
            ratioPlot.plot.opts.x = [0, Math.max(10, steps.length - 1)];
            ratioPlot.plot.set({ series: [{ x: steps, y: P.ratio.map((v) => v ?? NaN), color: p.red, width: 2 }],
                hlines: [{ y: 1, color: alpha(p.ink, 0.3) }] });
        }
    }

    function redrawAll() { drawCtrl(); drawObs(); drawInst(); }
    workspace.append(tabs, panels.ctrl, panels.obs, panels.inst);
    showTab('ctrl');
}

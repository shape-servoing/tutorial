// Local stability for n > m: Sym(J_hat^+ J) on Range(J^T).
import { BarChart } from '../lib/bars.js';
import { Runner } from '../lib/runner.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { button, card, h, readouts, segmented, slider, fmt, pct } from '../lib/ui.js';
import { plotCard, sceneCard } from '../lib/widgets.js';

export async function mount({ workspace, session, state }) {
    let S = state;
    let run = newRun();

    const sc = sceneCard('Run with the estimate held fixed', {
        camera: false,
        guide: 'Distort the estimate, then run. Here n>m: positivity of $\\Sym(\\vJh^{+}\\vJ)$ on $\\Range(\\vJ^{\\top})$ is sufficient for convergence of every controllable mode.',
        legendItems: (p) => [
            { label: 'rod', color: p.rod, kind: 'line' },
            { label: 'locally reachable target', color: p.target, kind: 'dashed' },
            { label: 'error $\\ve$', color: p.red, kind: 'line' },
            { label: 'direction of $\\vJ\\vJh^{+}\\ve$', color: p.aqua, kind: 'line' },
        ],
    });
    const runner = new Runner(session, { rate: 25, onResult: apply, onStop: syncButtons });
    const runButton = button('▶ Run', () => { if (runner.running) runner.stop(); else runner.start(); syncButtons(); }, { primary: true });
    const resetButton = button('Reset', async () => { runner.stop(); S = await session.call('reset'); restart(); });
    const newTarget = button('New nearby target', async () => { runner.stop(); S = await session.call('new_target', { distance: distance.get() }); restart(); });
    sc.toolbar.append(runButton, resetButton, newTarget);

    // ---- estimate controls
    const setEstimate = session.stream('set_estimate', (next) => { runner.stop(); S = next; restart(); });
    const est = card('Estimate $\\vJh$ built from the exact $\\vJ$');
    const scale = slider({ label: 'scale $c$ in $\\vJh=c\\vJ$', description: 'Changes magnitude but preserves command directions when $c>0$.', min: 0.2, max: 5, value: 1, log: true,
        format: (v) => v.toFixed(2), onInput: (v) => setEstimate({ scale: v }) });
    const plane = segmented({ options: [{ value: 'xL-xR', label: '$x_L,x_R$' }, { value: 'yL-thetaL', label: '$y_L,\\theta_L$' },
        { value: 'thetaL-thetaR', label: '$\\theta_L,\\theta_R$' }], value: 'yL-thetaL', onChange: (v) => setEstimate({ plane: v }) });
    const angle = slider({ label: 'rotation angle $\\gamma$', description: 'Rotates the selected pair of Jacobian columns and can reverse descent.', min: -180, max: 180, step: 1, value: 0,
        format: (v) => `${v.toFixed(0)}°`, onInput: (v) => setEstimate({ angle: v }) });
    const noise = slider({ label: 'random perturbation $\\varepsilon$', min: 0, max: 1.5, step: 0.01, value: 0,
        onInput: (v) => setEstimate({ noise: v }) });
    const flip = segmented({ options: [{ value: -1, label: 'none' }, ...['x_L', 'y_L', '\\theta_L', 'x_R', 'y_R', '\\theta_R'].map((t, j) => ({ value: j, label: `$${t}$` }))],
        value: -1, onChange: (v) => setEstimate({ flip: v }) });
    est.body.append(scale.el,
        h('div', { class: 'control' }, h('div', { class: 'control-head' }, h('label', {}, 'columns rotated')), plane.el),
        angle.el, noise.el,
        h('div', { class: 'control' }, h('div', { class: 'control-head' }, h('label', {}, 'flip the sign of column')), flip.el));
    const out = readouts([
        { key: 'merr', label: '$\\norm{\\vJ-\\vJh}_F/\\norm{\\vJ}_F$' },
        { key: 'lambda', label: '$\\lambda$ (smallest eigenvalue)' },
        { key: 'frac', label: 'sampled errors with descent' },
    ]);
    est.body.append(out.el);

    const loopCard = card('Loop');
    const gain = slider({ label: 'gain $\\alpha$ ($\\Delta t=1$)', description: 'Tests the discrete-time prediction $0<\\alpha\\Delta t<2$.', min: 0.02, max: 2.2, value: S.params.alpha,
        onInput: (v) => setAlpha({ alpha: v }) });
    const setAlpha = session.stream('set_alpha', (next) => { runner.stop(); S = next; restart(); });
    const distance = slider({ label: 'target distance $\\norm{\\boldsymbol{\\delta}}$', description: 'Applied when you press New nearby target.', min: 0.1, max: 1.5, value: S.params.distance });
    loopCard.body.append(gain.el, distance.el);

    // ---- descent fan and eigenvalues
    const fanCard = card('Angle between sampled errors $\\ve$ and the feature change $\\vJ\\vJh^{+}\\ve$ they produce', '600 random e in Range(J)');
    const fanCanvas = h('canvas', { style: 'display:block;width:100%;height:190px' });
    fanCard.body.append(fanCanvas);
    const eigCard = card('Eigenvalues of $\\Sym(\\vJh^{+}\\vJ)$ on $\\Range(\\vJ^{\\top})$');
    const eigCanvas = h('canvas', { class: 'bars-canvas', style: 'height:130px' });
    eigCard.body.append(eigCanvas);
    const eigBars = new BarChart(eigCanvas, { colors: (i, v) => (v > 0 ? palette().green : palette().red) });

    const errorCard = plotCard('Error with $\\vJh$ fixed', { xLabel: 'iteration k', yLabel: '‖e_k‖ / ‖e_0‖', logY: true }, { size: 'tall' });
    const rateCard = plotCard('Descent along the run', { xLabel: 'iteration k', yLabel: 'eᵀ J Ĵ⁺ e / ‖e‖²' }, { size: 'tall',
        note: '> 0: the error decreases to first order' });

    workspace.append(
        h('div', { class: 'stage wide-rail' }, h('div', {}, sc.el, h('section', { class: 'plots' }, fanCard.el, eigCard.el)),
            h('div', { class: 'rail' }, est.el, loopCard.el)),
        h('section', { class: 'plots' }, errorCard.el, rateCard.el));

    function newRun() { return { k: [0], err: [1], rk: [], rate: [] }; }
    function restart() { run = newRun(); update(); }
    function apply(result) {
        S = result;
        for (const s of result.samples) { run.k.push(s.k); run.err.push(s.error / S.initial_error); run.rk.push(s.k); run.rate.push(s.rate); }
        if (result.diverged) runner.stop();
        update();
    }
    function syncButtons() {
        runButton.textContent = runner.running ? '❚❚ Pause' : '▶ Run';
        [resetButton, newTarget].forEach((b) => { b.disabled = runner.running; });
    }

    function drawFan() {
        const c = fanCanvas, dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = c.clientWidth, hgt = c.clientHeight;
        if (!w) return;
        c.width = Math.round(w * dpr); c.height = Math.round(hgt * dpr);
        const ctx = c.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const p = palette();
        const cx = w / 2, cy = hgt - 26, R = Math.min(w / 2 - 20, hgt - 40);
        const counts = S.angle_counts, maxCount = Math.max(1, ...counts);
        ctx.fillStyle = alpha(p.green, 0.08);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, R, Math.PI, 1.5 * Math.PI); ctx.fill();
        ctx.fillStyle = alpha(p.red, 0.08);
        ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, R, 1.5 * Math.PI, 2 * Math.PI); ctx.fill();
        // 0 deg points left (same direction as e), 180 deg right (opposite).
        counts.forEach((n, i) => {
            const a0 = Math.PI + (S.angle_bins[i] / 180) * Math.PI, a1 = Math.PI + (S.angle_bins[i + 1] / 180) * Math.PI;
            const r = 12 + (R - 12) * Math.sqrt(n / maxCount);
            if (!n) return;
            ctx.fillStyle = S.angle_bins[i] < 90 ? p.green : p.red;
            ctx.beginPath(); ctx.moveTo(cx, cy); ctx.arc(cx, cy, r, a0, a1); ctx.closePath(); ctx.fill();
        });
        ctx.strokeStyle = p.lineStrong; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.arc(cx, cy, R, Math.PI, 2 * Math.PI); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(cx - R - 6, cy); ctx.lineTo(cx + R + 6, cy); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - R - 4); ctx.stroke();
        ctx.fillStyle = p.muted; ctx.font = `600 10px ${getComputedStyle(document.body).fontFamily}`;
        ctx.textAlign = 'center';
        ctx.fillText('0° (along e)', cx - R + 8, cy + 14); ctx.fillText('90°', cx, cy + 14); ctx.fillText('180°', cx + R - 8, cy + 14);
        ctx.fillStyle = p.green; ctx.textAlign = 'left'; ctx.fillText('descent', 6, 14);
        ctx.fillStyle = p.red; ctx.textAlign = 'right'; ctx.fillText('ascent', w - 6, 14);
    }
    new ResizeObserver(drawFan).observe(fanCanvas);
    onThemeChange(drawFan);

    function draw() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background({ floor: false });
            g.target(S.target);
            g.arms(S.scene.q);
            g.rod(S.scene.nodes);
            // e (red) and the direction J J_hat^+ e in which the command moves the
            // features (aqua), drawn with the same length as e.
            const scale = Math.hypot(...S.e) / Math.max(Math.hypot(...S.velocity), 1e-12);
            S.scene.points.forEach((pt, i) => {
                g.arrow(pt, [pt[0] + S.e[2 * i], pt[1] + S.e[2 * i + 1]], { color: p.red, width: 2.2, head: 8 });
                g.arrow(pt, [pt[0] + scale * S.velocity[2 * i], pt[1] + scale * S.velocity[2 * i + 1]],
                    { color: p.aqua, width: 2.2, head: 8 });
            });
            g.points(S.scene.points);
            g.grippers(S.scene.q);
            g.corner(`k = ${S.k}`);
        });
    }

    function update() {
        draw();
        drawFan();
        const p = sc.scene.p;
        out.set('merr', pct(S.matrix_error, 0));
        out.set('lambda', fmt(S.margin, 3), S.margin > 0 ? 'good' : 'bad');
        out.set('frac', pct(S.descent_fraction, 0), S.descent_fraction === 1 ? 'good' : 'bad');
        eigBars.set(S.eigenvalues);
        const a = S.params.alpha, lam = S.margin;
        const kMax = Math.max(60, run.k[run.k.length - 1]);
        const ks = Array.from({ length: kMax + 1 }, (_, i) => i);
        const series = [{ x: ks, y: ks.map((k) => Math.abs(1 - a) ** k), color: alpha(p.ink, 0.45), width: 1.2, dash: [2, 3] }];
        const legend = [{ label: 'exact-J rate $|1-\\alpha|^k$', color: alpha(p.ink, 0.6), dash: 'dotted' }];
        series.push({ x: run.k, y: run.err, color: lam > 0 ? p.rod : p.red, width: 2.4 });
        legend.push({ label: 'this run', color: lam > 0 ? p.rod : p.red });
        errorCard.plot.opts.x = [0, kMax];
        errorCard.plot.opts.y = [1e-6, 10];
        errorCard.plot.set({ series, hlines: [{ y: 1, color: alpha(p.ink, 0.25) }] });
        errorCard.setLegend(legend);
        rateCard.plot.opts.x = [0, kMax];
        rateCard.plot.set({ series: [{ x: run.rk, y: run.rate, color: p.accentStrong, width: 2 }], hlines: [{ y: 0, color: alpha(p.red, 0.6) }] });
        syncButtons();
    }
    update();
}

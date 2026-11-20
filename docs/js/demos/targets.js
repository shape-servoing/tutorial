// Demo 05 - Global feasibility/reachability and the running-example targets.
import { Runner } from '../lib/runner.js';
import { dragController } from '../lib/scene.js';
import { renderMath } from '../lib/math.js';
import { alpha, mix, onThemeChange, palette } from '../lib/theme.js';
import { button, card, h, readouts, segmented, slider, toggle, pct } from '../lib/ui.js';
import { plotCard, sceneCard } from '../lib/widgets.js';

export async function mount({ workspace, session, state }) {
    let S = state;
    let showSplit = true;
    let globalSelection = 'reachable';
    let run = newRun();
    const pastRuns = [];

    const sc = sceneCard('Recorded and beyond-limit targets', {
        note: 'drag a grey target ring to edit the target',
        guide: 'Compare a target recorded at an admissible grasp with a synthetic target beyond the rod deformation limits. Aqua error is locally reducible; red error is the orthogonal residual.',
        legendItems: (p) => [
            { label: 'target points', color: p.targetPoint, kind: 'ring' },
            { label: '$\\ve_{\\parallel}\\in\\Range(\\vJh_k)$', color: p.aqua, kind: 'line' },
            { label: '$\\ve_{\\perp}\\in\\Null(\\vJh_k^{\\top})$', color: p.red, kind: 'line' },
        ],
    });
    const runner = new Runner(session, { rate: 20, onResult: apply, onStop: syncButtons });
    const movePoint = session.stream('move_point', (next) => { S = next; update(); });
    dragController(sc.scene, {
        enabled: () => !runner.running,
        hit: (world) => {
            const index = S.target.points.findIndex((point) => Math.hypot(point[0] - world[0], point[1] - world[1]) < 0.32);
            return index >= 0 ? { mode: 'point', index } : null;
        },
        move: (handle, world) => movePoint({ index: handle.index, xy: world }),
    });
    const runButton = button('▶ Run', () => { if (runner.running) runner.stop(); else runner.start(); syncButtons(); }, { primary: true });
    const resetButton = button('Reset', async () => { runner.stop(); archive(); S = await session.call('reset'); restart(); });
    const kind = segmented({
        options: [
            { value: 'reachable', label: 'Feasible & reachable (recorded)' },
            { value: 'unfeasible', label: 'Infeasible (beyond limits)' },
        ],
        value: S.kind,
        onChange: async (value) => {
            runner.stop(); archive(); S = await session.call('set_target', { kind: value });
            globalSelection = value === 'unfeasible' ? 'unfeasible' : 'reachable';
            globalCase.set(globalSelection); restart();
        },
    });
    const split = toggle({ label: 'show error split', value: true, onChange: (value) => { showSplit = value; draw(); } });
    sc.toolbar.append(runButton, resetButton, h('span', { class: 'toolbar-divider' }), kind.el, split.el);

    const paramsCard = card('Controller');
    const exact = toggle({
        label: 'exact Jacobian $\\vJ$ (simulation only)',
        description: 'Compare ideal local knowledge with the online Broyden estimate.', value: false,
        onChange: async (value) => { runner.stop(); archive(); S = await session.call('set_params', { exact: value }); restart(); },
    });
    const setAlpha = session.stream('set_params', (next) => { archive(); S = next; restart(); });
    const gain = slider({
        label: 'gain $\\alpha$', description: 'Sets how far the controller moves per iteration.',
        min: 0.02, max: 0.3, value: S.params.alpha, onInput: (value) => setAlpha({ alpha: value }),
    });
    paramsCard.body.append(exact.el, gain.el);
    const out = readouts([
        { key: 'k', label: 'iteration $k$' },
        { key: 'err', label: '$\\norm{\\ve_k}/\\norm{\\ve_0}$' },
        { key: 'acted', label: '$\\norm{\\ve_{\\parallel}}/\\norm{\\ve}$' },
        { key: 'residual', label: '$\\norm{\\ve_{\\perp}}/\\norm{\\ve}$' },
        { key: 'exact', label: 'exact reducible fraction' },
    ]);
    paramsCard.body.append(out.el);

    const globalCard = card('Global feasibility and reachability', 'schematic only · the middle case is not simulated · global properties the local Jacobian cannot test');
    const globalCanvas = h('canvas', { class: 'subspace-canvas', role: 'img',
        'aria-label': 'Schematic feature space: a feasible set with two branches, the current state, and three target cases' });
    const globalCase = segmented({
        options: [
            { value: 'reachable', label: 'Feasible and reachable' },
            { value: 'blocked', label: 'Feasible, not reachable' },
            { value: 'unfeasible', label: 'Infeasible' },
        ],
        value: globalSelection,
        onChange: (value) => { globalSelection = value; drawGlobal(); },
    });
    const globalNote = h('p', { class: 'hint schematic-note' });
    globalCard.body.append(globalCanvas, h('div', { class: 'toolbar' }, globalCase.el), globalNote);

    const errorCard = plotCard('Normalized error', {
        xLabel: 'iteration k', yLabel: '||e_k|| / ||e_0||', y: [0, 1.05],
    }, { size: 'tall' });
    const commandCard = plotCard('Command norm $\\norm{\\vu_k}$', {
        xLabel: 'iteration k', yLabel: '||u_k||', logY: true,
    }, { size: 'tall' });
    const rangeCard = plotCard('Reducible and residual components', {
        xLabel: 'iteration k', yLabel: 'fraction of ||e||', y: [0, 1.05],
    }, { size: 'tall' });

    workspace.append(
        h('div', { class: 'stage' }, h('div', {}, sc.el, globalCard.el), h('div', { class: 'rail' }, paramsCard.el)),
        h('section', { class: 'plots' }, errorCard.el, commandCard.el, rangeCard.el));

    function newRun() {
        return {
            k: [0], err: [1], cmd: [], ck: [], rk: [S.k],
            acted: [S.split.range_fraction], residual: [S.split.orth_fraction],
            exact: [S.split.exact_range_fraction], kind: S.kind, exactJ: S.params.exact,
        };
    }
    function archive() {
        if (run.k.length > 2) { pastRuns.unshift(run); pastRuns.length = Math.min(pastRuns.length, 2); }
    }
    function restart() { run = newRun(); update(); }

    function apply(result) {
        S = result;
        for (const sample of result.samples) {
            run.k.push(sample.k); run.err.push(sample.error / S.initial_error);
            run.cmd.push(sample.command); run.ck.push(sample.k);
        }
        run.rk.push(S.k);
        run.acted.push(S.split.range_fraction);
        run.residual.push(S.split.orth_fraction);
        run.exact.push(S.split.exact_range_fraction);
        run.kind = S.kind;
        update();
    }

    function syncButtons() {
        runButton.textContent = runner.running ? '❚❚ Pause' : '▶ Run';
        resetButton.disabled = runner.running;
    }

    function draw() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background();
            if (S.target.nodes) g.target(S.target, { rings: false });
            else g.curve(S.target.points, { color: p.target, width: Math.max(1.6, 0.05 * g.scale), dash: [7, 6] });
            g.arms(S.scene.q);
            g.rod(S.scene.nodes);
            if (showSplit) {
                const { e_range: parallel, e_orth: orthogonal } = S.split;
                S.scene.points.forEach((point, i) => {
                    const mid = [point[0] + parallel[2 * i], point[1] + parallel[2 * i + 1]];
                    g.arrow(point, mid, { color: p.aqua, width: 2.4, head: 8 });
                    g.arrow(mid, [mid[0] + orthogonal[2 * i], mid[1] + orthogonal[2 * i + 1]],
                        { color: p.red, width: 2.4, head: 8 });
                });
            }
            g.points(S.scene.points);
            for (const point of S.target.points) g.disc(point, 0.17 * g.scale, null, p.targetPoint, 2);
            g.grippers(S.scene.q);
            g.corner('k = ' + S.k);
        });
    }

    function update() {
        draw(); drawGlobal(); kind.set(S.kind);
        const p = sc.scene.p;
        out.set('k', String(S.k));
        const ratio = S.error / S.initial_error;
        out.set('err', pct(ratio, 1), ratio < 0.01 ? 'good' : '');
        out.set('acted', pct(S.split.range_fraction, 1));
        out.set('residual', pct(S.split.orth_fraction, 1));
        out.set('exact', pct(S.split.exact_range_fraction, 1));
        const kMax = Math.max(90, run.k[run.k.length - 1]);
        const colorOf = (item) => item.kind === 'reachable' ? p.rod : p.red;
        const names = { reachable: 'recorded, reached target', unfeasible: 'synthetic infeasible target', custom: 'edited target' };
        const nameOf = (item) => (names[item.kind] || item.kind) + (item.exactJ ? ', exact J' : '');
        const series = [], labels = [];
        pastRuns.slice().reverse().forEach((item) => {
            const color = mix(colorOf(item), p.canvas, 0.5);
            series.push({ x: item.k, y: item.err, color, width: 1.5 });
            labels.push({ label: 'earlier: ' + nameOf(item), color });
        });
        series.push({ x: run.k, y: run.err, color: colorOf(S), width: 2.4 });
        labels.push({ label: 'this run: ' + nameOf({ kind: S.kind, exactJ: S.params.exact }), color: colorOf(S) });
        errorCard.plot.opts.x = [0, kMax]; errorCard.plot.set({ series }); errorCard.setLegend(labels);
        commandCard.plot.opts.x = [0, kMax];
        commandCard.plot.set({ series: [{ x: run.ck, y: run.cmd, color: p.accentStrong, width: 1.8 }] });
        rangeCard.plot.opts.x = [0, kMax];
        rangeCard.plot.set({ series: [
            { x: run.rk, y: run.acted, color: p.aqua, width: 2.2 },
            { x: run.rk, y: run.residual, color: p.red, width: 2.2 },
            { x: run.rk, y: run.exact, color: alpha(p.ink, 0.6), width: 1.6, dash: [5, 4] },
        ] });
        rangeCard.setLegend([
            { label: 'reducible ||e_parallel|| / ||e||', color: p.aqua },
            { label: 'residual ||e_perp|| / ||e||', color: p.red },
            { label: 'exact reducible fraction', color: alpha(p.ink, 0.6), dash: 'dashed' },
        ]);
        syncButtons();
    }

    // Schematic of the feature space R^n (not computed): the feasible set S has
    // two components (equilibrium branches); the current state lies in the left
    // one. The three cases of the tutorial are drawn together; the selected one is
    // emphasized and the others are faded.
    const GLOBAL_NOTES = {
        reachable: '<b>Feasible and reachable.</b> The target lies in <span class="tex">\\mathcal{S}</span>, and an admissible manipulation path connects it to the current state. The recorded target above is of this kind: it was realized at another admissible grasp, and the loop reaches it.',
        blocked: '<b>Feasible, not reachable from here.</b> The target lies in <span class="tex">\\mathcal{S}</span>, but on a branch that no admissible path connects to the current state (e.g., the rod buckled the other way). The Jacobian describes only the local directions <span class="tex">\\Range(\\vJ)</span>, so the loop cannot detect this.',
        unfeasible: '<b>Infeasible.</b> The target lies outside <span class="tex">\\mathcal{S}</span>: no admissible configuration realizes it. The beyond-limits target above is of this kind, and the loop settles with a residual error.',
    };

    function drawGlobal() {
        const dpr = Math.min(window.devicePixelRatio || 1, 2), w = globalCanvas.clientWidth, height = globalCanvas.clientHeight;
        if (!w || !height) return;
        globalCanvas.width = Math.round(w * dpr); globalCanvas.height = Math.round(height * dpr);
        const ctx = globalCanvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const p = palette(), font = getComputedStyle(document.body).fontFamily;
        ctx.fillStyle = p.canvas; ctx.fillRect(0, 0, w, height);
        // Drawing coordinates: u in [0, 1] across, v in [0, 1] down, aspect kept.
        const span = Math.min(w - 32, 3.1 * (height - 28));
        const ox = (w - span) / 2, oy = 14, sy = height - 28;
        const P = ([u, v]) => [ox + u * span, oy + v * sy];
        const sel = globalSelection;
        const faded = (key) => (sel === key ? 1 : 0.32);

        // Feasible set: two smooth components.
        const branchA = [[0.06, 0.55], [0.12, 0.22], [0.33, 0.10], [0.52, 0.20], [0.60, 0.48],
            [0.53, 0.80], [0.30, 0.92], [0.12, 0.84]].map(P);
        const branchB = [[0.69, 0.42], [0.76, 0.24], [0.86, 0.30], [0.89, 0.55], [0.82, 0.76], [0.72, 0.70]].map(P);
        for (const blob of [branchA, branchB]) {
            smoothClosed(ctx, blob);
            ctx.fillStyle = alpha(p.aqua, 0.10); ctx.fill();
            ctx.strokeStyle = alpha(p.aqua, 0.85); ctx.lineWidth = 1.5; ctx.stroke();
        }
        label(ctx, 'feasible set 𝒮', P([0.10, 0.10]), p.aqua, 'left', 12, 720);
        label(ctx, 'other branch of 𝒮', P([0.79, 0.14]), alpha(p.aqua, 1), 'center', 11, 650);
        label(ctx, 'feature space ℝⁿ (schematic)', [w - 10, height - 9], p.muted, 'right', 10.5, 600);

        const current = P([0.20, 0.62]);
        const reachable = P([0.44, 0.34]), blocked = P([0.80, 0.52]), outside = P([0.97, 0.80]);

        // Local information: Range(J) at the current state (tangent directions only).
        const tangent = [Math.cos(-0.55), Math.sin(-0.55)], half = 0.085 * span;
        ctx.save(); ctx.strokeStyle = p.rod; ctx.lineWidth = 2; ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.moveTo(current[0] - half * tangent[0], current[1] - half * tangent[1]);
        ctx.lineTo(current[0] + half * tangent[0], current[1] + half * tangent[1]); ctx.stroke(); ctx.restore();
        label(ctx, 'local Range(𝐉)', [current[0] + 0.015 * span, current[1] + 0.14 * sy], p.rod, 'center', 11, 650);

        // 1) Admissible path inside the same component.
        ctx.globalAlpha = faded('reachable');
        ctx.strokeStyle = p.green; ctx.lineWidth = 2.4; ctx.beginPath(); ctx.moveTo(...current);
        const c1 = P([0.26, 0.40]), c2 = P([0.36, 0.52]);
        const dir = [reachable[0] - c2[0], reachable[1] - c2[1]], len = Math.hypot(...dir);
        const tip = [reachable[0] - 11 * dir[0] / len, reachable[1] - 11 * dir[1] / len];
        ctx.bezierCurveTo(...c1, ...c2, ...tip); ctx.stroke();
        canvasArrow(ctx, c2, tip, p.green);
        label(ctx, 'admissible path', P([0.36, 0.60]), p.green, 'center', 11, 650);
        targetRing(ctx, reachable, p.green, sel === 'reachable');

        // 2) Target on the other branch: every path leaves S.
        ctx.globalAlpha = faded('blocked');
        ctx.save(); ctx.strokeStyle = p.muted; ctx.lineWidth = 1.8; ctx.setLineDash([4, 4]);
        ctx.beginPath(); ctx.moveTo(...current);
        ctx.bezierCurveTo(...P([0.38, 0.86]), ...P([0.62, 0.86]), ...blocked); ctx.stroke(); ctx.restore();
        cross(ctx, P([0.645, 0.70]), p.red);
        label(ctx, 'no admissible path', P([0.645, 0.92]), p.red, 'center', 11, 650);
        targetRing(ctx, blocked, p.targetPoint, sel === 'blocked');

        // 3) Target outside S.
        ctx.globalAlpha = faded('unfeasible');
        targetRing(ctx, outside, p.red, sel === 'unfeasible');
        label(ctx, 'outside 𝒮', [outside[0], outside[1] - 15], p.red, 'center', 11, 650);
        ctx.globalAlpha = 1;

        // Current state on top.
        disc(ctx, current, 6, p.feature, p.featureRing, 1.6);
        label(ctx, 'current state', [current[0] - 10, current[1] - 12], p.ink, 'right', 11, 700);

        globalNote.innerHTML = GLOBAL_NOTES[sel];
        renderMath(globalNote);
    }

    function smoothClosed(ctx, pts) {
        const n = pts.length, mid = (a, b) => [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2];
        ctx.beginPath(); ctx.moveTo(...mid(pts[n - 1], pts[0]));
        for (let i = 0; i < n; i += 1) ctx.quadraticCurveTo(...pts[i], ...mid(pts[i], pts[(i + 1) % n]));
        ctx.closePath();
    }
    function label(ctx, text, [x, y], color, align, size, weight) {
        ctx.font = `${weight} ${size}px ${getComputedStyle(document.body).fontFamily}`;
        ctx.textAlign = align; ctx.textBaseline = 'middle'; ctx.fillStyle = color; ctx.fillText(text, x, y);
    }
    function targetRing(ctx, point, color, active) {
        disc(ctx, point, active ? 9 : 7, alpha(color, 0.15), color, active ? 2.6 : 1.8);
        disc(ctx, point, 2.4, color, null);
    }
    function cross(ctx, [x, y], color, r = 6) {
        ctx.strokeStyle = color; ctx.lineWidth = 2.4; ctx.beginPath();
        ctx.moveTo(x - r, y - r); ctx.lineTo(x + r, y + r); ctx.moveTo(x + r, y - r); ctx.lineTo(x - r, y + r); ctx.stroke();
    }
    function canvasArrow(ctx, from, to, color) {
        const angle = Math.atan2(to[1] - from[1], to[0] - from[0]); ctx.fillStyle = color;
        ctx.beginPath(); ctx.moveTo(...to);
        ctx.lineTo(to[0] - 9 * Math.cos(angle - 0.42), to[1] - 9 * Math.sin(angle - 0.42));
        ctx.lineTo(to[0] - 9 * Math.cos(angle + 0.42), to[1] - 9 * Math.sin(angle + 0.42)); ctx.closePath(); ctx.fill();
    }
    function disc(ctx, point, radius, fill, stroke, width = 1.5) {
        ctx.beginPath(); ctx.arc(point[0], point[1], radius, 0, 2 * Math.PI);
        if (fill) { ctx.fillStyle = fill; ctx.fill(); }
        if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = width; ctx.stroke(); }
    }

    new ResizeObserver(drawGlobal).observe(globalCanvas);
    onThemeChange(drawGlobal);
    update();
}

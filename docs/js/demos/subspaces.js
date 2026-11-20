// Demo 06 - The four fundamental subspaces of the deformation Jacobian.
import { DOF_COLS, FEATURE_ROWS } from '../lib/matrix.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { button, card, h, readouts, segmented, slider, fmt, pct } from '../lib/ui.js';
import { matrixCard, sceneCard } from '../lib/widgets.js';

export async function mount({ workspace, session, state }) {
    let S = state;

    const sc = sceneCard('What can change locally?', {
        guide: 'Both views use tracked (x,y) positions. The blue component lies in the active range; the red component lies in the orthogonal null space.',
        legendItems: (p) => [
            { label: 'current / combined shape', color: p.rod, kind: 'line' },
            { label: 'range component', color: p.aqua, kind: 'line' },
            { label: 'null-space component / preserved points', color: p.red, kind: 'line' },
        ],
    });
    const caseChoice = segmented({
        options: [
            { value: 'error', label: 'Feature error: $n>m$' },
            { value: 'motion', label: 'Two points: $n<m$' },
        ],
        value: S.case,
        onChange: async (value) => { S = await session.call('set_case', { case: value }); update(); },
    });
    const resetButton = button('Reset components', async () => { S = await session.call('reset'); update(); });
    sc.toolbar.append(caseChoice.el, resetButton);

    const controls = card('Build a decomposition');
    const setParams = session.stream('set_params', (next) => { S = next; update(); });
    const rangeWeight = slider({
        label: 'reducible component $\\ve_{\\parallel}$',
        description: 'Scales the component in $\\Range(\\vJ)$.',
        min: 0, max: 1.5, step: 0.01, value: S.params.range_weight,
        onInput: (value) => setParams({ range_weight: value }),
    });
    const residualWeight = slider({
        label: 'non-reducible component $\\ve_{\\perp}$',
        description: 'Scales the local residual in $\\Null(\\vJ^{\\top})$.',
        min: 0, max: 1.5, step: 0.01, value: S.params.residual_weight,
        onInput: (value) => setParams({ residual_weight: value }),
    });
    const taskWeight = slider({
        label: 'task-producing motion $\\dq_{\\mathrm{task}}$',
        description: 'Scales the component in $\\Range(\\vJ^{\\top})$.',
        min: 0, max: 1.5, step: 0.01, value: S.params.task_weight,
        onInput: (value) => setParams({ task_weight: value }),
    });
    const nullWeight = slider({
        label: 'point-preserving motion $\\dq_{\\mathrm{null}}$',
        description: 'Scales a gripper motion in $\\Null(\\vJ)$; the two tracked positions do not move to first order.',
        min: 0, max: 1.5, step: 0.01, value: S.params.null_weight,
        onInput: (value) => setParams({ null_weight: value }),
    });
    const errorControls = h('div', {}, rangeWeight.el, residualWeight.el);
    const motionControls = h('div', {}, taskWeight.el, nullWeight.el);
    controls.body.append(errorControls, motionControls);

    const errorStatsCard = card('Feature-space split');
    const errorStats = readouts([
        { key: 'map', label: 'local map' },
        { key: 'rank', label: '$\\operatorname{rank}(\\vJ)$' },
        { key: 'range', label: '$\\dim\\Range(\\vJ)$' },
        { key: 'null', label: '$\\dim\\Null(\\vJ^{\\top})$' },
        { key: 'reducible', label: 'reducible fraction' },
        { key: 'residual', label: 'non-reducible fraction' },
    ]);
    errorStatsCard.body.append(errorStats.el);

    const motionStatsCard = card('Robot-space split');
    const motionStats = readouts([
        { key: 'map', label: 'local map' },
        { key: 'rank', label: '$\\operatorname{rank}(\\vJ)$' },
        { key: 'range', label: '$\\dim\\Range(\\vJ^{\\top})$' },
        { key: 'null', label: '$\\dim\\Null(\\vJ)$' },
        { key: 'task', label: '$\\norm{\\Delta\\vs_{\\mathrm{task}}}$' },
        { key: 'nullchange', label: '$\\norm{\\Delta\\vs_{\\mathrm{null}}}$' },
    ]);
    motionStatsCard.body.append(motionStats.el);

    const diagramCard = card('Orthogonal subspace decomposition', 'exact Jacobian at the displayed configuration');
    const diagram = h('canvas', { class: 'subspace-canvas', role: 'img',
        'aria-label': 'Orthogonal range and null-space decomposition' });
    const diagramCaption = h('p', { class: 'hint' });
    diagramCard.body.append(diagram, diagramCaption);

    const jCard = matrixCard('Local deformation Jacobian $\\vJ$', {
        rowLabels: FEATURE_ROWS, colLabels: DOF_COLS, colColors: palette().dof, maxCell: 27,
    }, { note: 'five tracked points: 10×6 · two tracked points: 4×6', height: 250 });

    workspace.append(
        h('div', { class: 'stage' }, sc.el,
            h('div', { class: 'rail' }, controls.el, errorStatsCard.el, motionStatsCard.el)),
        h('section', { class: 'plots' }, diagramCard.el, jCard.el));

    function drawScene() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background();
            if (S.case === 'error') {
                g.curve(S.target.points, { color: p.target, width: Math.max(1.6, 0.05 * g.scale), dash: [7, 6] });
                g.arms(S.scene.q);
                g.rod(S.scene.nodes);
                S.scene.points.forEach((point, i) => {
                    const mid = [point[0] + S.e_range[2 * i], point[1] + S.e_range[2 * i + 1]];
                    const target = [mid[0] + S.e_orth[2 * i], mid[1] + S.e_orth[2 * i + 1]];
                    g.arrow(point, mid, { color: p.aqua, width: 2.5, head: 8 });
                    g.arrow(mid, target, { color: p.red, width: 2.5, head: 8 });
                    g.disc(target, 0.17 * g.scale, null, p.targetPoint, 2);
                });
                g.points(S.scene.points);
                g.grippers(S.scene.q);
                g.badge('e = e_parallel + e_perp');
            } else {
                g.arms(S.combined_scene.q);
                g.rod(S.scene.nodes, { color: p.muted, width: 0.07, halo: false, alphaValue: 0.45 });
                g.rod(S.task_scene.nodes, { color: p.aqua, width: 0.08, halo: false, alphaValue: 0.70 });
                g.rod(S.null_scene.nodes, { color: p.red, width: 0.08, halo: false, alphaValue: 0.70 });
                g.rod(S.combined_scene.nodes);
                g.command(S.scene.q, S.dq_task, { color: p.aqua, radius: 0.52 });
                g.command(S.scene.q, S.dq_null, { color: p.red, radius: 0.68 });
                g.points(S.scene.points, { color: p.canvas, r: 0.15, ring: p.feature });
                g.points(S.null_scene.points, { color: p.red, r: 0.075, ring: p.canvas });
                g.points(S.combined_scene.points);
                g.grippers(S.combined_scene.q);
                g.badge('Δq = Δq_task + Δq_null');
            }
        });
    }

    function drawDiagram() {
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = diagram.clientWidth, height = diagram.clientHeight;
        if (!w || !height) return;
        diagram.width = Math.round(w * dpr); diagram.height = Math.round(height * dpr);
        const ctx = diagram.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const p = palette(), font = getComputedStyle(document.body).fontFamily;
        ctx.fillStyle = p.canvas; ctx.fillRect(0, 0, w, height);
        const origin = [66, height - 48], xEnd = [w - 28, height - 48], yEnd = [66, 25];
        const isError = S.case === 'error';
        const xWeight = isError ? S.params.range_weight : S.params.task_weight;
        const yWeight = isError ? S.params.residual_weight : S.params.null_weight;
        const x = origin[0] + (xEnd[0] - origin[0]) * (0.12 + 0.52 * xWeight / 1.5);
        const y = origin[1] - (origin[1] - yEnd[1]) * (0.12 + 0.58 * yWeight / 1.5);

        ctx.fillStyle = alpha(p.aqua, 0.08); ctx.fillRect(origin[0], origin[1] - 17, xEnd[0] - origin[0], 34);
        ctx.fillStyle = alpha(p.red, 0.07); ctx.fillRect(origin[0] - 17, yEnd[1], 34, origin[1] - yEnd[1]);
        arrow(ctx, origin, xEnd, p.lineStrong, 1.3, 7);
        arrow(ctx, origin, yEnd, p.lineStrong, 1.3, 7);
        arrow(ctx, origin, [x, origin[1]], p.aqua, 3, 9);
        arrow(ctx, [x, origin[1]], [x, y], p.red, 3, 9);
        arrow(ctx, origin, [x, y], p.target, 1.7, 8, [6, 4]);
        ctx.fillStyle = p.targetPoint; ctx.beginPath(); ctx.arc(x, y, 5, 0, 2 * Math.PI); ctx.fill();

        ctx.font = '650 11px ' + font; ctx.textAlign = 'center'; ctx.fillStyle = p.aqua;
        ctx.fillText(isError ? 'Range(J)  · reducible' : 'Range(Jᵀ)  · task-producing', (origin[0] + xEnd[0]) / 2, height - 17);
        ctx.save(); ctx.translate(18, (origin[1] + yEnd[1]) / 2); ctx.rotate(-Math.PI / 2); ctx.fillStyle = p.red;
        ctx.fillText(isError ? 'Null(Jᵀ)  · residual' : 'Null(J)  · point-preserving', 0, 0); ctx.restore();
        ctx.fillStyle = p.muted; ctx.textAlign = 'left';
        ctx.fillText(isError ? 'feature error e' : 'robot motion Δq', x + 8, y - 7);
        ctx.fillStyle = p.inkSoft; ctx.font = '600 10px ' + font;
        ctx.fillText('orthogonal projection', origin[0] + 8, 17);
    }

    function arrow(ctx, from, to, color, width, head, dash = []) {
        const angle = Math.atan2(to[1] - from[1], to[0] - from[0]);
        ctx.save(); ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
        ctx.beginPath(); ctx.moveTo(...from); ctx.lineTo(...to); ctx.stroke(); ctx.setLineDash([]);
        ctx.beginPath(); ctx.moveTo(...to);
        ctx.lineTo(to[0] - head * Math.cos(angle - 0.42), to[1] - head * Math.sin(angle - 0.42));
        ctx.lineTo(to[0] - head * Math.cos(angle + 0.42), to[1] - head * Math.sin(angle + 0.42));
        ctx.closePath(); ctx.fill(); ctx.restore();
    }

    function update() {
        caseChoice.set(S.case);
        const isError = S.case === 'error';
        errorControls.style.display = isError ? '' : 'none';
        motionControls.style.display = isError ? 'none' : '';
        errorStatsCard.el.style.display = isError ? '' : 'none';
        motionStatsCard.el.style.display = isError ? 'none' : '';
        rangeWeight.set(S.params.range_weight); residualWeight.set(S.params.residual_weight);
        taskWeight.set(S.params.task_weight); nullWeight.set(S.params.null_weight);
        const d = S.dimensions;
        if (isError) {
            errorStats.set('map', 'R' + d.m + ' → R' + d.n);
            errorStats.set('rank', d.rank + ' / ' + Math.min(d.n, d.m));
            errorStats.set('range', String(d.range_dim));
            errorStats.set('null', String(d.null_jt_dim));
            errorStats.set('reducible', pct(S.range_fraction, 1));
            errorStats.set('residual', pct(S.orth_fraction, 1));
            diagramCaption.textContent = 'Only e_parallel produces a command. Since Jᵀ e_perp = 0, e_perp is the local least-squares residual.';
            jCard.matrix.opts.rowLabels = FEATURE_ROWS;
        } else {
            motionStats.set('map', 'R' + d.m + ' → R' + d.n);
            motionStats.set('rank', d.rank + ' / ' + Math.min(d.n, d.m));
            motionStats.set('range', String(d.range_jt_dim));
            motionStats.set('null', String(d.null_j_dim));
            motionStats.set('task', fmt(Math.hypot(...S.task_change), 4));
            motionStats.set('nullchange', fmt(Math.hypot(...S.null_change), 4), Math.hypot(...S.null_change) < 0.01 ? 'good' : '');
            diagramCaption.textContent = 'The null component changes the grippers while leaving both tracked (x,y) positions unchanged to first order.';
            jCard.matrix.opts.rowLabels = ['x₁', 'y₁', 'x₂', 'y₂'];
        }
        jCard.matrix.opts.colColors = palette().dof;
        jCard.matrix.set(S.J);
        drawScene(); drawDiagram();
    }

    new ResizeObserver(drawDiagram).observe(diagram);
    onThemeChange(() => { drawScene(); drawDiagram(); });
    update();
}

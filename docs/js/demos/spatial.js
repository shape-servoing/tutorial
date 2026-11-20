// Demo 11 - Spatial shape servoing with local grasp patches and robot context.
import { Runner } from '../lib/runner.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { button, card, h, legend, readouts, segmented, slider, toggle, fmt, pct } from '../lib/ui.js';
import { matrixCard, plotCard } from '../lib/widgets.js';

export async function mount({ workspace, session, state }) {
    let S = state;
    let yaw = -0.65, pitch = 0.42, zoom = 1;
    let run = makeHistory(S);

    const scene = card('Robot-assisted 3-D shape control', 'drag to orbit · wheel to zoom');
    scene.body.append(h('p', { class: 'interaction-guide' },
        'The robots hold local graph-neighborhoods, not whole boundaries. The dashed object is the reachable target; yellow points define the feedback feature.'));
    const canvas = h('canvas', { class: 'scene-canvas spatial-canvas',
        role: 'img', 'aria-label': 'Interactive three-dimensional deformable-object and robot view' });
    scene.body.append(canvas);

    const miniViews = [
        { label: 'front  x–y', yaw: 0, pitch: 0 },
        { label: 'top  x–z', yaw: 0, pitch: -Math.PI / 2 },
        { label: 'side  z–y', yaw: Math.PI / 2, pitch: 0 },
    ].map((view) => {
        const mini = h('canvas', { class: 'spatial-mini-canvas', role: 'img',
            'aria-label': view.label + ' orthographic view' });
        return { ...view, canvas: mini, el: h('div', { class: 'spatial-mini-view' }, mini, h('span', {}, view.label)) };
    });
    scene.body.append(h('div', { class: 'spatial-view-grid' }, miniViews.map((view) => view.el)));
    scene.body.append(legend([
        { label: 'current object', color: palette().rod, kind: 'line' },
        { label: 'target', color: palette().target, kind: 'dashed' },
        { label: 'regulated vertices', color: palette().feature, kind: 'dot' },
        { label: 'fixed grasp', color: palette().gripL, kind: 'ring' },
        { label: 'actuated grasp', color: palette().gripR, kind: 'ring' },
    ]));
    const toolbar = h('div', { class: 'toolbar' });
    scene.body.append(toolbar);

    const runner = new Runner(session, { rate: 12, maxBatch: 2, onResult: apply, onStop: syncButtons });
    const runButton = button('▶ Run', () => { if (runner.running) runner.stop(); else runner.start(); syncButtons(); }, { primary: true });
    const stepButton = button('Step once', async () => {
        runner.stop();
        apply(await runner.once());
    });
    const resetButton = button('Reset', async () => {
        runner.stop();
        S = await session.call('reset');
        restartHistory();
        update();
    });
    const objectChoice = segmented({
        options: [
            { value: 'rod', label: 'Spatial rod' },
            { value: 'sheet', label: 'ARAP sheet' },
            { value: 'solid', label: 'Volumetric solid' },
        ],
        value: S.kind,
        onChange: async (kind) => {
            runner.stop();
            S = await session.call('set_object', { kind });
            restartHistory();
            update();
        },
    });
    toolbar.append(objectChoice.el, h('span', { class: 'toolbar-divider' }), runButton, stepButton, resetButton);

    const controlCard = card('Experiment');
    const setParams = session.stream('set_params', (next) => {
        const targetChanged = next.params.severity !== S.params.severity;
        S = next;
        if (targetChanged) restartHistory();
        update();
    });
    const severity = slider({
        label: 'target deformation ×', description: 'Scales the reachable displacement and rotation used to create the target.',
        min: 0.45, max: 1.35, step: 0.05, value: S.params.severity,
        format: (v) => '×' + v.toFixed(2), onInput: (v) => { runner.stop(); setParams({ severity: v }); },
    });
    const gain = slider({
        label: 'feedback gain $\\alpha$', description: 'Scales the pseudoinverse command at each local linearization.',
        min: 0.05, max: 0.45, step: 0.01, value: S.params.gain,
        onInput: (v) => setParams({ gain: v }),
    });
    const lattice = toggle({
        label: 'show volume lattice', description: 'Reveal the ARAP edges coupling the solid boundary and interior.',
        value: false, onChange: draw,
    });
    controlCard.body.append(severity.el, gain.el, lattice.el);

    const statsCard = card('Current model');
    const modelNote = h('p', { class: 'hint' });
    const stats = readouts([
        { key: 'mesh', label: 'discretization' },
        { key: 'grasp', label: 'grasp patches' },
        { key: 'features', label: 'feature dimension $n$' },
        { key: 'rank', label: 'Jacobian rank' },
        { key: 'condition', label: '$\\cond(\\vJ_k)$' },
        { key: 'k', label: 'iteration $k$' },
        { key: 'error', label: '$\\norm{\\ve_k}/\\norm{\\ve_0}$' },
        { key: 'command', label: '$\\norm{\\Delta\\vq_k}$' },
    ]);
    statsCard.body.append(modelNote, stats.el);

    const errorPlot = plotCard('Error and command size', {
        xLabel: 'iteration k', yLabel: 'ratio / ||Δq_k||', logY: true, y: [1e-4, 2],
    }, { size: 'tall', note: '3-D feature convergence and control effort' });
    const actuationPlot = plotCard('Generalized actuation', {
        xLabel: 'iteration k', yLabel: 'q', yIncludeZero: true,
    }, { size: 'tall', note: 'solid: q_k · dashed: reachable target' });
    const singularPlot = plotCard('Jacobian singular values', {
        xLabel: 'iteration k', yLabel: 'σ_i(J_k)', logY: true,
    }, { size: 'tall', note: 'local sensitivity and conditioning' });
    const jCard = matrixCard('Feature Jacobian $\\vJ_k$', {
        colLabels: ['q₁', 'q₂', 'q₃'], maxCell: 28,
    }, { note: 'hover for entries · latest controller linearization', height: 248 });

    workspace.append(
        h('div', { class: 'stage' }, scene.el, h('div', { class: 'rail' }, controlCard.el, statsCard.el)),
        h('section', { class: 'plots' }, errorPlot.el, actuationPlot.el, singularPlot.el, jCard.el));

    function makeHistory(source) {
        return {
            k: [source.k || 0],
            error: [source.k ? source.error / source.initial_error : 1],
            command: [source.command || 0],
            q: source.q.map((v) => [v]),
            singular: source.singular_values.map((v) => [v]),
        };
    }

    function restartHistory() { run = makeHistory(S); }

    function apply(result) {
        S = result;
        for (const sample of result.samples || []) {
            run.k.push(sample.k);
            run.error.push(sample.error);
            run.command.push(sample.command);
            sample.q.forEach((v, i) => run.q[i].push(v));
            sample.singular_values.forEach((v, i) => run.singular[i].push(v));
        }
        update();
    }

    function syncButtons() {
        runButton.textContent = runner.running ? '❚❚ Pause' : '▶ Run';
        stepButton.disabled = runner.running;
        resetButton.disabled = runner.running;
    }

    function update() {
        severity.set(S.params.severity);
        gain.set(S.params.gain);
        objectChoice.set(S.kind);
        lattice.el.style.display = S.kind === 'solid' ? '' : 'none';
        stats.set('mesh', S.faces.length ? S.vertices.length + ' v · ' + S.faces.length + ' △' : S.vertices.length + ' nodes');
        stats.set('grasp', S.fixed.length + ' fixed · ' + S.driven.length + ' driven · radius ' + S.grasp_radius + (S.grasp_radius === 1 ? ' edge' : ' edges'));
        stats.set('features', String(3 * S.features.length));
        stats.set('rank', S.rank + ' / 3', S.rank === 3 ? 'good' : 'bad');
        stats.set('condition', fmt(S.condition, 2), S.condition < 20 ? 'good' : 'bad');
        stats.set('k', String(S.k));
        const ratio = S.error / S.initial_error;
        stats.set('error', pct(ratio, 1), ratio < 0.03 ? 'good' : '');
        stats.set('command', fmt(S.command, 4));
        modelNote.textContent = S.label + ' · ' + S.deformation + ' · local radius-' + S.grasp_radius + ' grasps';

        const p = palette();
        const xMax = Math.max(40, run.k[run.k.length - 1] || 0);
        errorPlot.plot.opts.x = [0, xMax];
        errorPlot.plot.set({
            series: [
                { x: run.k, y: run.error, color: p.rod, width: 2.4 },
                { x: run.k, y: run.command, color: p.accentStrong, width: 1.8 },
            ],
            hlines: [{ y: 0.03, color: alpha(p.green, 0.75), dash: [5, 4] }],
        });
        errorPlot.setLegend([
            { label: 'normalized feature error', color: p.rod },
            { label: 'command ||Δq_k||', color: p.accentStrong },
            { label: '3% target band', color: p.green, dash: 'dashed' },
        ]);

        actuationPlot.plot.opts.x = [0, xMax];
        const qSeries = run.q.map((values, i) => ({ x: run.k, y: values, color: p.dof[i], width: 2.2 }));
        const qTargets = S.q_target.map((value, i) => ({
            x: [0, xMax], y: [value, value], color: p.dof[i], width: 1.2, dash: [5, 4], alpha: 0.65,
        }));
        actuationPlot.plot.set({ series: qSeries.concat(qTargets) });
        actuationPlot.setLegend(S.q_labels.map((label, i) => ({ label: 'q' + (i + 1) + ' · ' + label, color: p.dof[i] })));

        singularPlot.plot.opts.x = [0, xMax];
        singularPlot.plot.set({ series: run.singular.map((values, i) => ({
            x: run.k, y: values, color: p.series[i], width: 2.0,
        })) });
        singularPlot.setLegend(run.singular.map((_, i) => ({ label: 'σ' + (i + 1), color: p.series[i] })));
        jCard.matrix.opts.colColors = p.dof.slice(0, 3);
        jCard.matrix.set(S.jacobian);
        syncButtons();
        draw();
    }

    // ------------------------------------------------------------ 3D views
    function rotatePoint(point, center, viewYaw, viewPitch) {
        const x = point[0] - center[0], y = point[1] - center[1], z = point[2] - center[2];
        const cy = Math.cos(viewYaw), sy = Math.sin(viewYaw), cp = Math.cos(viewPitch), sp = Math.sin(viewPitch);
        const x1 = cy * x + sy * z, z1 = -sy * x + cy * z;
        return [x1, cp * y - sp * z1, sp * y + cp * z1];
    }

    function mean(points) {
        return points.reduce((sum, point) => sum.map((v, i) => v + point[i] / points.length), [0, 0, 0]);
    }

    function robotGeometry(floorY) {
        const fixed = S.vertices[S.fixed_anchor], driven = S.vertices[S.driven_anchor];
        // Bases from the rest geometry, so that they stay fixed while the object moves.
        const xs = S.rest.map((v) => v[0]), zs = S.rest.map((v) => v[2]);
        const build = (contact, side) => {
            const base = [side < 0 ? Math.min(...xs) - 1.7 : Math.max(...xs) + 1.7,
                floorY, side < 0 ? Math.min(...zs) - 0.65 : Math.max(...zs) + 0.65];
            const shoulder = [base[0], floorY + 0.78, base[2]];
            const elbow = [(shoulder[0] + contact[0]) / 2,
                Math.max(contact[1] + 0.75, floorY + 1.45), (shoulder[2] + contact[2]) / 2 - side * 0.38];
            const wrist = [contact[0] + side * 0.38, contact[1] + 0.08, contact[2]];
            return { side, contact, chain: [base, shoulder, elbow, wrist, contact] };
        };
        return [build(fixed, -1), build(driven, 1)];
    }

    function draw() {
        if (!S || !canvas.clientWidth) return;
        renderView(canvas, yaw, pitch, true);
        for (const view of miniViews) renderView(view.canvas, view.yaw, view.pitch, false);
    }

    function renderView(targetCanvas, viewYaw, viewPitch, main) {
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = targetCanvas.clientWidth, height = targetCanvas.clientHeight;
        if (!w || !height) return;
        targetCanvas.width = Math.round(w * dpr);
        targetCanvas.height = Math.round(height * dpr);
        const ctx = targetCanvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const p = palette();
        ctx.fillStyle = p.canvas;
        ctx.fillRect(0, 0, w, height);

        const center = mean(S.rest);
        const floorY = Math.min(...S.rest.map((v) => v[1])) - 1.15;
        const robots = robotGeometry(floorY);
        const current = S.vertices.map((v) => rotatePoint(v, center, viewYaw, viewPitch));
        const target = S.target.map((v) => rotatePoint(v, center, viewYaw, viewPitch));
        const robotPoints = robots.flatMap((robot) => robot.chain).map((v) => rotatePoint(v, center, viewYaw, viewPitch));
        const fit = current.concat(target, main ? robotPoints : []);
        const xs = fit.map((v) => v[0]), ys = fit.map((v) => v[1]);
        const spanX = Math.max(...xs) - Math.min(...xs), spanY = Math.max(...ys) - Math.min(...ys);
        const margin = main ? 30 : 10;
        const localZoom = main ? zoom : 1;
        const scale = localZoom * Math.min((w - 2 * margin) / Math.max(spanX, 1), (height - 2 * margin) / Math.max(spanY, 1));
        const midX = (Math.max(...xs) + Math.min(...xs)) / 2;
        const midY = (Math.max(...ys) + Math.min(...ys)) / 2;
        const project = (v) => [w / 2 + scale * (v[0] - midX), height / 2 - scale * (v[1] - midY)];
        const worldToScreen = (v) => project(rotatePoint(v, center, viewYaw, viewPitch));

        const edgePath = (vertices, edges, color, width, dash = []) => {
            ctx.save();
            ctx.strokeStyle = color;
            ctx.lineWidth = width;
            ctx.lineCap = 'round';
            ctx.setLineDash(dash);
            ctx.beginPath();
            for (const [a, b] of edges) {
                ctx.moveTo(...project(vertices[a]));
                ctx.lineTo(...project(vertices[b]));
            }
            ctx.stroke();
            ctx.restore();
        };

        if (main) {
            drawGround(ctx, worldToScreen, floorY, p);
            drawShadows(ctx, robots, floorY, worldToScreen, p);
            drawRobots(ctx, robots, worldToScreen, p);
        }

        edgePath(target, S.edges, alpha(p.target, main ? 0.82 : 0.65), main ? 1.5 : 1, main ? [6, 5] : [4, 3]);
        if (S.faces.length) {
            const order = S.faces.map((face) => ({
                face, depth: (current[face[0]][2] + current[face[1]][2] + current[face[2]][2]) / 3,
            })).sort((a, b) => a.depth - b.depth);
            for (const { face } of order) {
                const a = current[face[0]], b = current[face[1]], c = current[face[2]];
                const ux = b[0] - a[0], uy = b[1] - a[1], vx = c[0] - a[0], vy = c[1] - a[1];
                const shade = Math.min(1, Math.abs(ux * vy - uy * vx) / 2);
                ctx.beginPath();
                ctx.moveTo(...project(a)); ctx.lineTo(...project(b)); ctx.lineTo(...project(c)); ctx.closePath();
                ctx.fillStyle = alpha(p.rod, (main ? 0.10 : 0.08) + 0.13 * shade);
                ctx.fill();
            }
            edgePath(current, S.edges, alpha(p.rod, main ? 0.74 : 0.62), main ? 1.05 : 0.8);
        } else {
            edgePath(current, S.edges, p.rod, main ? 5 : 2.5);
        }

        if (main && S.kind === 'solid' && lattice.get()) {
            edgePath(current, S.model_edges, alpha(p.aqua, 0.28), 0.85);
            for (const v of current) disc(ctx, project(v), 2.1, alpha(p.aqua, 0.45), null);
        }
        drawGraspPatch(ctx, current, project, S.fixed, p.gripL, main);
        drawGraspPatch(ctx, current, project, S.driven, p.gripR, main);
        for (const i of S.features) disc(ctx, project(current[i]), main ? 4 : 2.3, p.feature, p.featureRing);
        if (main) {
            for (const i of S.driven) disc(ctx, project(target[i]), 5.2, null, alpha(p.target, 0.9));
            drawGrippers(ctx, robots, worldToScreen, p);
            drawAxisTriad(ctx, p, w, height);
            ctx.fillStyle = p.muted;
            ctx.font = '600 11px ' + getComputedStyle(document.body).fontFamily;
            ctx.textAlign = 'right';
            ctx.fillText('orbit ' + (viewYaw * 180 / Math.PI).toFixed(0) + '° / ' + (viewPitch * 180 / Math.PI).toFixed(0) + '°', w - 12, height - 11);
        }
    }

    function drawGround(ctx, project, floorY, p) {
        const xs = S.rest.map((v) => v[0]), zs = S.rest.map((v) => v[2]);
        const x0 = Math.floor(Math.min(...xs) - 2), x1 = Math.ceil(Math.max(...xs) + 2);
        const z0 = Math.floor(Math.min(...zs) - 2), z1 = Math.ceil(Math.max(...zs) + 2);
        ctx.save();
        ctx.lineWidth = 0.8;
        for (let x = x0; x <= x1; x += 0.5) {
            ctx.strokeStyle = alpha(Number.isInteger(x) ? p.gridStrong : p.grid, Number.isInteger(x) ? 0.52 : 0.32);
            line(ctx, project([x, floorY, z0]), project([x, floorY, z1]));
        }
        for (let z = z0; z <= z1; z += 0.5) {
            ctx.strokeStyle = alpha(Number.isInteger(z) ? p.gridStrong : p.grid, Number.isInteger(z) ? 0.52 : 0.32);
            line(ctx, project([x0, floorY, z]), project([x1, floorY, z]));
        }
        ctx.restore();
    }

    function drawShadows(ctx, robots, floorY, project, p) {
        const flattened = S.vertices.map((v) => [v[0] + 0.18, floorY + 0.015, v[2] + 0.12]);
        ctx.save();
        ctx.strokeStyle = alpha(p.ink, 0.10);
        ctx.fillStyle = alpha(p.ink, 0.045);
        ctx.lineWidth = S.faces.length ? 2.2 : 7;
        for (const [a, b] of S.edges) line(ctx, project(flattened[a]), project(flattened[b]));
        ctx.lineWidth = 7;
        for (const robot of robots) {
            const chain = robot.chain.map((v) => [v[0] + 0.18, floorY + 0.01, v[2] + 0.12]);
            for (let i = 0; i + 1 < chain.length; i++) line(ctx, project(chain[i]), project(chain[i + 1]));
        }
        ctx.restore();
    }

    function drawRobots(ctx, robots, project, p) {
        for (const robot of robots) {
            const points = robot.chain.map(project);
            ctx.save();
            ctx.lineCap = 'round'; ctx.lineJoin = 'round';
            ctx.strokeStyle = p.robotDeep; ctx.lineWidth = 12;
            for (let i = 0; i + 2 < points.length; i++) line(ctx, points[i], points[i + 1]);
            ctx.strokeStyle = p.robot; ctx.lineWidth = 7;
            for (let i = 0; i + 2 < points.length; i++) line(ctx, points[i], points[i + 1]);
            for (let i = 0; i < points.length - 1; i++) disc(ctx, points[i], i === 0 ? 8 : 6, p.joint, p.robotDeep);
            ctx.restore();
        }
    }

    function drawGrippers(ctx, robots, project, p) {
        for (const robot of robots) {
            const color = robot.side < 0 ? p.gripL : p.gripR;
            const jawHalf = S.kind === 'solid' ? 0.16 : 0.22;
            const palmOffset = S.kind === 'solid' ? 0.16 : 0.22;
            const c = robot.contact, palm = [c[0] + robot.side * palmOffset, c[1], c[2]];
            const jawA = [palm[0], palm[1], palm[2] - jawHalf], jawB = [palm[0], palm[1], palm[2] + jawHalf];
            const tipA = [c[0], c[1], c[2] - jawHalf], tipB = [c[0], c[1], c[2] + jawHalf];
            ctx.save(); ctx.strokeStyle = color; ctx.lineWidth = 5; ctx.lineCap = 'round';
            line(ctx, project(jawA), project(jawB));
            line(ctx, project(jawA), project(tipA));
            line(ctx, project(jawB), project(tipB));
            ctx.restore();
            disc(ctx, project(c), 5.5, alpha(color, 0.22), color);
        }
    }

    function drawGraspPatch(ctx, vertices, project, indices, color, main) {
        const selected = new Set(indices);
        const localEdges = S.model_edges.filter(([a, b]) => selected.has(a) && selected.has(b));
        ctx.save(); ctx.strokeStyle = alpha(color, main ? 0.72 : 0.50); ctx.lineWidth = main ? 4 : 2;
        for (const [a, b] of localEdges) line(ctx, project(vertices[a]), project(vertices[b]));
        ctx.restore();
        for (const i of indices) disc(ctx, project(vertices[i]), main ? 4.7 : 2.4, alpha(color, 0.22), color);
    }

    function drawAxisTriad(ctx, p, w, height) {
        const origin = [25, height - 24];
        const axes = [
            { to: [origin[0] + 23, origin[1]], color: p.red, label: 'x' },
            { to: [origin[0], origin[1] - 23], color: p.green, label: 'y' },
            { to: [origin[0] - 12, origin[1] + 9], color: p.aqua, label: 'z' },
        ];
        ctx.save(); ctx.font = '600 10px ' + getComputedStyle(document.body).fontFamily;
        for (const axis of axes) {
            ctx.strokeStyle = axis.color; ctx.lineWidth = 2; line(ctx, origin, axis.to);
            ctx.fillStyle = axis.color; ctx.textAlign = 'center'; ctx.fillText(axis.label, axis.to[0], axis.to[1] - 4);
        }
        ctx.restore();
    }

    function line(ctx, a, b) {
        ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.stroke();
    }

    function disc(ctx, [x, y], radius, fill, stroke) {
        ctx.beginPath(); ctx.arc(x, y, radius, 0, 2 * Math.PI);
        if (fill) { ctx.fillStyle = fill; ctx.fill(); }
        if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = 1.5; ctx.stroke(); }
    }

    let drag = null;
    canvas.addEventListener('pointerdown', (event) => {
        drag = { x: event.clientX, y: event.clientY, yaw, pitch };
        canvas.setPointerCapture(event.pointerId);
        canvas.style.cursor = 'grabbing';
    });
    canvas.addEventListener('pointermove', (event) => {
        if (!drag) { canvas.style.cursor = 'grab'; return; }
        yaw = drag.yaw + 0.009 * (event.clientX - drag.x);
        pitch = Math.max(-1.35, Math.min(1.35, drag.pitch + 0.009 * (event.clientY - drag.y)));
        draw();
    });
    const endDrag = (event) => {
        drag = null;
        if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
        canvas.style.cursor = 'grab';
    };
    canvas.addEventListener('pointerup', endDrag);
    canvas.addEventListener('pointercancel', endDrag);
    canvas.addEventListener('wheel', (event) => {
        event.preventDefault();
        zoom = Math.max(0.72, Math.min(1.55, zoom * Math.exp(-event.deltaY * 0.001)));
        draw();
    }, { passive: false });
    new ResizeObserver(draw).observe(canvas);
    for (const view of miniViews) new ResizeObserver(draw).observe(view.canvas);
    onThemeChange(draw);

    update();
}

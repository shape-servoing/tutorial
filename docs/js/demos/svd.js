// Demo 08 - Singular directions of the Jacobian: (sigma_l, psi_l, phi_l).
import { DOF_COLS, FEATURE_ROWS } from '../lib/matrix.js';
import { gripperDrag, lerp, lerpNodes } from '../lib/scene.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { button, card, h, readouts, segmented, fmt } from '../lib/ui.js';
import { barsCard, matrixCard, sceneCard } from '../lib/widgets.js';

const ROW_H = 58;

export async function mount({ workspace, session, state }) {
    let S = state;
    let frame = null;

    const sc = sceneCard('Robot motion along $\\vpsi_l$', {
        note: 'drag the grippers: the estimate is recomputed at the new configuration',
        guide: 'Click a singular-triplet row to select and animate it. You can also drag either gripper to see how the singular directions change with configuration.',
        view: { x0: -6.2, x1: 6.2, y0: -2.2, y1: 4.4 },
        legendItems: (p) => [
            { label: 'rod', color: p.rod, kind: 'line' },
            { label: 'first-order prediction $\\pm\\sigma_l\\vphi_l$', color: p.rod, kind: 'ring' },
        ],
    });
    const move = session.stream('move', (next) => { S = next; update(); });
    gripperDrag(sc.scene, { getQ: () => S.scene.q, getNodes: () => S.scene.nodes, enabled: () => !frame, onMove: (q) => move({ q }) });
    const playButton = button('▶ Animate selected direction', () => play(S.selected), { primary: true });
    const source = segmented({ options: [{ value: 'estimate', label: 'offline $\\vJh_0$' }, { value: 'exact', label: 'exact $\\vJ$' }],
        value: 'estimate', onChange: async (v) => { S = await session.call('set', { source: v }); update(); } });
    const units = segmented({ options: [{ value: 'rad', label: 'angles in rad' }, { value: 'deg', label: 'angles in deg' }],
        value: 'rad', onChange: async (v) => { S = await session.call('set', { units: v }); update(); } });
    const resetButton = button('Reset', async () => { if (frame) return; S = await session.call('reset'); source.set('estimate'); units.set('rad'); update(); });
    sc.toolbar.append(playButton, resetButton, h('span', { class: 'toolbar-divider' }), source.el, units.el);

    // ---- triplet panel
    const tripCard = card('Singular triplets, strongest first', 'click a row to play it');
    const tripCanvas = h('canvas', { style: `display:block;width:100%;height:${6 * ROW_H + 26}px;cursor:pointer` });
    tripCard.body.append(tripCanvas);
    tripCanvas.addEventListener('click', (e) => {
        const r = tripCanvas.getBoundingClientRect();
        const l = Math.floor((e.clientY - r.top - 22) / ROW_H);
        if (l >= 0 && l < S.sigma.length) { S.selected = l; session.call('set', { selected: l }); update(); play(l); }
    });
    new ResizeObserver(() => drawTriplets()).observe(tripCanvas);
    onThemeChange(() => drawTriplets());

    const stats = card('Summaries');
    const out = readouts([
        { key: 'rank', label: 'rank' },
        { key: 'w', label: '$w=\\prod_l\\sigma_l$' },
        { key: 'cond', label: '$\\cond=\\sigma_{\\max}/\\sigma_{\\min}$' },
    ]);
    stats.body.append(out.el);
    const inv = barsCard('Robot speed per unit error along $\\vphi_l$: $1/\\sigma_l$', { labels: ['1', '2', '3', '4', '5', '6'], yTicks: true }, { height: 130 });
    const jCard = matrixCard('Jacobian (in the chosen units)', { rowLabels: FEATURE_ROWS, colLabels: DOF_COLS, maxCell: 24 }, { height: 280 });

    workspace.append(
        h('div', { class: 'stage wide-rail' }, h('div', {}, sc.el, h('section', { class: 'plots' }, jCard.el, h('div', {}, stats.el, inv.el))),
            h('div', { class: 'rail' }, tripCard.el)));

    // ---------------------------------------------------------------- triplets
    function drawTriplets() {
        const c = tripCanvas, dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = c.clientWidth, hgt = c.clientHeight;
        if (!w) return;
        c.width = Math.round(w * dpr); c.height = Math.round(hgt * dpr);
        const ctx = c.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        const p = palette(), font = getComputedStyle(document.body).fontFamily;
        const colS = [28, 104], colV = [114, 210], colU = [220, w - 4];
        ctx.font = `700 10px ${font}`; ctx.fillStyle = p.muted; ctx.textAlign = 'center';
        ctx.fillText('gain σ_l', (colS[0] + colS[1]) / 2, 12);
        ctx.fillText('robot ψ_l', (colV[0] + colV[1]) / 2, 12);
        ctx.fillText('features φ_l on the rod', (colU[0] + colU[1]) / 2, 12);
        const nodes = S.scene.nodes, pts = S.scene.points;
        const xs = nodes.map((n) => n[0]), ys = nodes.map((n) => n[1]);
        const xMid = (Math.min(...xs) + Math.max(...xs)) / 2, yMid = (Math.min(...ys) + Math.max(...ys)) / 2;
        const span = Math.max(Math.max(...xs) - Math.min(...xs) + 2.0, (Math.max(...ys) - Math.min(...ys) + 2.2) * (colU[1] - colU[0]) / (ROW_H - 6));
        const k = (colU[1] - colU[0]) / span;
        const lg = (v) => Math.log10(Math.max(v, 1e-6));
        for (let l = 0; l < S.sigma.length; l++) {
            const top = 22 + l * ROW_H, mid = top + ROW_H / 2;
            if (l === S.selected) { ctx.fillStyle = p.accentSoft; ctx.fillRect(0, top + 1, w, ROW_H - 2); }
            ctx.fillStyle = p.inkSoft; ctx.font = `700 11px ${font}`; ctx.textAlign = 'left';
            ctx.fillText(`${l + 1}`, 6, mid + 4);
            // sigma on a log axis from 0.01 to 10
            const sx = (v) => colS[0] + (lg(v) + 2) / 3 * (colS[1] - colS[0]);
            ctx.strokeStyle = p.grid; ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(colS[0], mid + 8); ctx.lineTo(colS[1], mid + 8); ctx.stroke();
            ctx.fillStyle = p.ink; ctx.beginPath(); ctx.arc(sx(S.sigma[l]), mid + 8, 4, 0, 2 * Math.PI); ctx.fill();
            ctx.font = `600 10px ${font}`; ctx.textAlign = 'center';
            ctx.fillText(S.sigma[l] < 1e-3 ? S.sigma[l].toExponential(1) : S.sigma[l].toFixed(2), sx(S.sigma[l]), mid - 4);
            // psi bars
            const bw = (colV[1] - colV[0]) / 6;
            ctx.strokeStyle = p.gridStrong; ctx.beginPath(); ctx.moveTo(colV[0], mid); ctx.lineTo(colV[1], mid); ctx.stroke();
            for (let j = 0; j < 6; j++) {
                const v = S.Psi[j][l];
                ctx.fillStyle = p.dof[j];
                ctx.fillRect(colV[0] + j * bw + bw * 0.18, Math.min(mid, mid - v * (ROW_H / 2 - 5)), bw * 0.64, Math.abs(v) * (ROW_H / 2 - 5));
            }
            // phi on the rod
            const X = (x) => colU[0] + (x - xMid) * k + (colU[1] - colU[0]) / 2, Y = (y) => mid - (y - yMid) * k;
            ctx.strokeStyle = alpha(p.muted, 0.6); ctx.lineWidth = 2; ctx.beginPath();
            nodes.forEach((n, i) => (i ? ctx.lineTo(X(n[0]), Y(n[1])) : ctx.moveTo(X(n[0]), Y(n[1])))); ctx.stroke();
            ctx.strokeStyle = p.rod; ctx.fillStyle = p.rod; ctx.lineWidth = 1.6;
            pts.forEach((pt, i) => {
                const d = [S.Phi[2 * i][l], S.Phi[2 * i + 1][l]];
                const x0 = X(pt[0]), y0 = Y(pt[1]), x1 = X(pt[0] + d[0]), y1 = Y(pt[1] + d[1]);
                if (Math.hypot(x1 - x0, y1 - y0) < 2) return;
                const a = Math.atan2(y1 - y0, x1 - x0);
                ctx.beginPath(); ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); ctx.stroke();
                ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x1 - 5 * Math.cos(a - 0.45), y1 - 5 * Math.sin(a - 0.45));
                ctx.lineTo(x1 - 5 * Math.cos(a + 0.45), y1 - 5 * Math.sin(a + 0.45)); ctx.closePath(); ctx.fill();
            });
            pts.forEach((pt) => { ctx.fillStyle = p.feature; ctx.beginPath(); ctx.arc(X(pt[0]), Y(pt[1]), 2.6, 0, 2 * Math.PI); ctx.fill(); });
        }
    }

    // ---------------------------------------------------------------- playback
    async function play(index) {
        if (frame) return;
        playButton.disabled = true;
        const { frames } = await session.call('play', { index });
        await new Promise((resolve) => {
            const t0 = performance.now(), duration = 2800;
            const tick = (now) => {
                const u = Math.min(1, Math.max(0, now - t0) / duration) * (frames.length - 1);
                const i = Math.min(frames.length - 2, Math.floor(u)), t = u - i, a = frames[i], b = frames[i + 1];
                frame = { q: lerp(a.q, b.q, t), nodes: lerpNodes(a.nodes, b.nodes, t), points: lerpNodes(a.points, b.points, t),
                    predicted: lerpNodes(a.predicted, b.predicted, t), index };
                draw();
                if (now - t0 < duration) requestAnimationFrame(tick); else { frame = null; draw(); resolve(); }
            };
            requestAnimationFrame(tick);
        });
        playButton.disabled = false;
    }

    function draw() {
        sc.scene.draw((g) => {
            const p = g.p;
            g.background();
            const q = frame ? frame.q : S.scene.q;
            g.arms(q);
            if (frame) g.rod(S.scene.nodes, { alphaValue: 0.25, halo: false, width: 0.07 });
            g.rod(frame ? frame.nodes : S.scene.nodes);
            if (frame) frame.predicted.forEach((pr) => g.disc(pr, 0.17 * g.scale, null, p.rod, 2));
            else S.scene.points.forEach((pt, i) => g.arrow(pt, [pt[0] + S.Phi[2 * i][S.selected], pt[1] + S.Phi[2 * i + 1][S.selected]],
                { color: p.rod, width: 2.2, head: 8 }));
            g.points(frame ? frame.points : S.scene.points);
            g.grippers(q);
            if (!frame) g.handles(q);
            const l = frame ? frame.index : S.selected;
            g.badge(`l = ${l + 1} · σ = ${S.sigma[l].toFixed(3)}${frame ? ' · robot motion ±ψ_l' : ' · arrows: φ_l'}`);
        });
    }

    function update() {
        draw();
        drawTriplets();
        const p = sc.scene.p;
        out.set('rank', `${S.rank} / 6`);
        out.set('w', fmt(S.manipulability, 4));
        out.set('cond', fmt(S.condition, 1));
        inv.bars.opts.colors = (i) => (i === S.selected ? p.accentStrong : p.rod);
        inv.bars.set(S.sigma.map((s) => 1 / s));
        jCard.matrix.opts.colColors = p.dof;
        jCard.matrix.set(S.J);
    }
    update();
}

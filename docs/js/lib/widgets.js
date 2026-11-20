// Composite widgets shared by the demos: scene card, plot card, matrix card,
// bar card, and the six robot-coordinate sliders.
import { BarChart } from './bars.js';
import { MatrixView } from './matrix.js';
import { LinePlot } from './plot.js';
import { SceneView } from './scene.js';
import { palette } from './theme.js';
import { card, h, label, legend, seriesLegend, slider, DOF_TEX } from './ui.js';

export function sceneCard(title, { note = '', guide = '', legendItems = null, tall = false, compact = false, view, arms = true, camera = true } = {}) {
    const c = card(title, note);
    const canvas = h('canvas', { class: `scene-canvas${tall ? ' tall' : ''}${compact ? ' compact' : ''}` });
    const toolbar = h('div', { class: 'toolbar' });
    if (guide) c.body.append(h('p', { class: 'interaction-guide' }, label(guide)));
    c.body.append(canvas);
    if (legendItems) c.body.append(legend(legendItems(palette())));
    c.body.append(toolbar);
    const scene = new SceneView(canvas, { view, arms, camera });
    return { ...c, canvas, toolbar, scene };
}

export function plotCard(title, opts = {}, { note = '', size = '', legendItems = null } = {}) {
    const c = card(title, note);
    const canvas = h('canvas', { class: `plot-canvas ${size}` });
    c.body.append(canvas);
    let legendEl = null;
    const setLegend = (items) => {
        const next = seriesLegend(items);
        if (legendEl) legendEl.replaceWith(next); else c.body.append(next);
        legendEl = next;
    };
    if (legendItems) setLegend(legendItems);
    return { ...c, plot: new LinePlot(canvas, opts), setLegend };
}

export function matrixCard(title, opts = {}, { note = '', height = 220 } = {}) {
    const c = card(title, note);
    const canvas = h('canvas', { class: 'matrix-canvas', style: `height:${height}px` });
    c.body.append(canvas);
    return { ...c, matrix: new MatrixView(canvas, opts) };
}

export function barsCard(title, opts = {}, { note = '', height = 120 } = {}) {
    const c = card(title, note);
    const canvas = h('canvas', { class: 'bars-canvas', style: `height:${height}px` });
    c.body.append(canvas);
    return { ...c, bars: new BarChart(canvas, opts) };
}

// Six sliders for q = [x_L, y_L, theta_L, x_R, y_R, theta_R].
export function robotSliders(q, onInput) {
    const ranges = [[-4.2, -0.6], [-1.2, 3.8], [-1.45, 1.45], [0.6, 4.2], [-1.2, 3.8], [-1.45, 1.45]];
    const values = [...q];
    const sliders = ranges.map(([lo, hi], j) => slider({
        label: `$${DOF_TEX[j]}$`, min: lo, max: hi, step: 0.01, value: q[j],
        format: (v) => v.toFixed(2) + (j % 3 === 2 ? ' rad' : ''),
        onInput: (v) => { values[j] = v; onInput([...values]); },
    }));
    return {
        el: h('div', {}, sliders.map((s) => s.el)),
        set(qNew) { qNew.forEach((v, j) => { values[j] = v; sliders[j].set(v); }); },
        disable(flag) { sliders.forEach((s) => s.disable(flag)); },
    };
}

// Grid of two columns of cards under the stage.
export function plotsGrid(...cards) {
    return h('section', { class: 'plots' }, cards.map((c) => c.el));
}

// Feature-vector entry labels for the point coordinates.
export const POINT_LABELS = ['x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'x4', 'y4', 'x5', 'y5'];
export const pointColors = (p) => (i) => (i % 2 === 0 ? p.rod : p.aqua);

// The menu: one card per demo, in tutorial order, with a drawn preview.
import { catalog } from './lib/api.js';
import { initPage } from './lib/page.js';
import { celebrateCompletedParts, progressBadge, resetProgress, visitedIds } from './lib/rewards.js';
import { SceneView } from './lib/scene.js';
import { reloadKeepingSettings } from './lib/store.js';
import { citeDialog, initTools } from './lib/tools.js';
import { h, label } from './lib/ui.js';

await initPage();
initTools(document.getElementById('toolsButton'));
document.getElementById('citeButton').addEventListener('click', () => citeDialog());

const visited = visitedIds();
const demos = await catalog({ previews: true });

function drawPreview(view, preview) {
    view.draw((g) => {
        g.background({ grid: true, floor: false });
        g.target(preview.target);
        preview.rods.forEach((nodes, i) => g.rod(nodes, {
            alphaValue: i === preview.rods.length - 1 ? 1 : 0.3 + 0.4 * i / preview.rods.length,
            halo: i === preview.rods.length - 1,
        }));
        for (const [a, b] of preview.arrows) g.arrow(a, b, { color: g.p.rod, width: 1.6, head: 6 });
        g.points(preview.points, { r: 0.13 });
    });
}

for (const demo of demos) {
    const canvas = h('canvas', { 'aria-hidden': 'true' });
    const card = h('a', { class: 'demo-card', href: `demo.html?id=${demo.id}` },
        canvas,
        h('div', { class: 'demo-card-body' },
            h('div', { class: 'demo-card-top' },
                h('span', { class: 'demo-index' }, String(demo.number).padStart(2, '0')),
                h('h3', {}, demo.title)),
            h('p', {}, label(demo.summary)),
            h('div', { class: 'concepts' }, demo.concepts.map((c) => h('span', { class: 'concept' }, c)))),
        visited.has(demo.id) ? h('span', { class: 'visited-badge' }, '✓ visited') : null);
    document.getElementById(demo.part === 1 ? 'part1Cards' : 'part2Cards').append(card);
    const view = new SceneView(canvas, { arms: false, view: { x0: -4.2, x1: 4.2, y0: -0.9, y1: 4.0 } });
    view.redraw = () => drawPreview(view, demo.preview);
    view.redraw();
}

// Progress toward the level medals, and medals of parts completed earlier.
function showProgress() {
    for (const part of [1, 2]) document.getElementById(`part${part}Progress`).replaceChildren(progressBadge(demos, part));
}
showProgress();
document.getElementById('resetProgress').addEventListener('click', () => {
    if (!confirm('Forget which demos you have visited and the medals you have earned?')) return;
    resetProgress();
    reloadKeepingSettings();
});
celebrateCompletedParts(demos).then(showProgress);

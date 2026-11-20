// Progress through the demos and the two level medals: visiting every level 1
// demo (fundamentals) earns the level 1 medal, every level 2 demo (advanced
// concepts) the level 2 medal; `part` in the catalog is the level. The medal
// is given when the user leaves the demo that completed the level (by going to
// another demo or back to the menu), once, with confetti and a short dialog.
// Progress lives in this browser only (js/lib/store.js: localStorage, or the
// links between the pages where storage is unavailable).
import { confetti } from './confetti.js';
import { medalSVG } from './medal.js';
import { getItem, removeItem, setItem } from './store.js';
import { h } from './ui.js';

const VISITED = 'shape-servoing-visited';
const MEDALS = 'shape-servoing-medals';
const PENDING = 'shape-servoing-medal-pending';   // {part: id of the demo that completed it}

function read(key, fallback = []) {
    try { return JSON.parse(getItem(key) || 'null') ?? fallback; } catch { return fallback; }
}
function write(key, value) {
    setItem(key, JSON.stringify(value));
}

export const visitedIds = () => new Set(read(VISITED));
export const earnedMedals = () => new Set(read(MEDALS).map(Number));

// Record a visit; if it completes a part, the medal waits until the user leaves this demo.
export function markVisited(id, demos = null) {
    const before = visitedIds(), visited = new Set(before);
    visited.add(id);
    write(VISITED, [...visited]);
    if (demos) {
        const medals = earnedMedals(), pending = read(PENDING, {});
        for (const part of [1, 2]) {
            const was = partProgress(demos, part, before), now = partProgress(demos, part, visited);
            if (now.total > 0 && now.done === now.total && was.done < was.total && !medals.has(part)) pending[part] = id;
        }
        write(PENDING, pending);
    }
    return visited;
}

export function partProgress(demos, part, visited = visitedIds()) {
    const ids = demos.filter((d) => d.part === part).map((d) => d.id);
    return { done: ids.filter((id) => visited.has(id)).length, total: ids.length };
}

export function resetProgress() {
    [VISITED, MEDALS, PENDING].forEach(removeItem);
}

// Award (once) the medal of every completed part, provided the user has left
// the demo that completed it. `currentId` is the demo on screen (null: menu).
// The part stays pending until the user acknowledges the dialog (closes it or
// follows one of its buttons): if the page is left before that, the
// celebration comes again on the next page.
export async function celebrateCompletedParts(demos, currentId = null) {
    const visited = visitedIds();
    for (const part of [1, 2]) {
        const pending = read(PENDING, {}), medals = earnedMedals();
        const { done, total } = partProgress(demos, part, visited);
        if (total === 0 || done < total) {
            if (pending[part]) { delete pending[part]; write(PENDING, pending); }
            continue;
        }
        if (!pending[part]) {
            if (medals.has(part)) continue;                                     // already celebrated
            if (currentId) { pending[part] = currentId; write(PENDING, pending); continue; }  // completed earlier
        } else if (pending[part] === currentId) continue;                       // still on that demo
        medals.add(part);
        write(MEDALS, [...medals]);
        const acknowledge = () => { const p = read(PENDING, {}); delete p[part]; write(PENDING, p); };
        await celebrate(part, demos, medals, currentId, acknowledge);
    }
}

const TEXT = {
    1: {
        part: 'Level 1 · Fundamentals',
        body: 'You have explored every level 1 demo: the shape features and error, the deformation Jacobian, its offline estimation and online update, the closed loop, target feasibility, and local stability.',
    },
    2: {
        part: 'Level 2 · Advanced concepts',
        body: 'You have explored every level 2 demo: the singular value analysis of the deformation Jacobian, the choice of shape features, the failure modes of the loop, its extensions, and three-dimensional objects.',
    },
};

// Confetti plus a dialog with the medal; resolves when the dialog is closed.
export function celebrate(level, demos, medals = earnedMedals(), currentId = null, acknowledge = () => {}) {
    const firstPart2 = demos.find((d) => d.part === 2);
    const current = demos.find((d) => d.id === currentId);
    const both = medals.has(1) && medals.has(2);
    // Where to go next depends on where the user already is.
    const goOn = level === 1 ? !(current && current.part === 2) && firstPart2 : currentId !== null;
    const opened = performance.now();
    return new Promise((resolve) => {
        const close = () => {
            acknowledge();
            overlay.classList.add('closing');
            document.removeEventListener('keydown', onKey);
            setTimeout(() => { overlay.remove(); resolve(); }, 220);
        };
        const onKey = (event) => { if (event.key === 'Escape') close(); };
        // A click on the backdrop closes the dialog, but not in its first moments,
        // so that a click meant for the page does not dismiss it unseen.
        const onBackdrop = (event) => { if (event.target === overlay && performance.now() - opened > 1500) close(); };
        const primary = !goOn
            ? h('button', { class: 'button primary', type: 'button', onclick: close }, 'Continue ▶')
            : level === 1
                ? h('a', { class: 'button primary', href: `demo.html?id=${firstPart2.id}`, onclick: acknowledge }, 'Continue to level 2 ▶')
                : h('a', { class: 'button primary', href: 'index.html', onclick: acknowledge }, 'Back to all demos');
        const stay = goOn ? h('button', { class: 'button', type: 'button', onclick: close }, 'Keep exploring') : null;
        const medal = h('div', { class: 'medal-stage', html: medalSVG(level, { size: 230 }) });
        const card = h('div', {
            class: 'medal-card', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'medalTitle',
            'data-part': String(level),
        },
            medal,
            h('p', { class: 'kicker' }, TEXT[level].part),
            h('h2', { id: 'medalTitle' }, `Level ${level} complete!`),
            h('p', { class: 'medal-text' }, TEXT[level].body),
            both ? h('p', { class: 'medal-text medal-final' }, 'Both medals earned: you have completed the interactive tutorial.') : null,
            h('div', { class: 'medal-actions' }, stay, primary));
        const overlay = h('div', { class: 'medal-overlay', onclick: onBackdrop }, card);
        document.body.append(overlay);
        document.addEventListener('keydown', onKey);
        primary.focus({ preventScroll: true });
        confetti();
    });
}

// Small medal (earned) or progress text, for the menu headings.
export function progressBadge(demos, part, visited = visitedIds(), medals = earnedMedals()) {
    if (medals.has(part)) {
        return h('span', { class: 'part-progress earned', title: `Level ${part} medal earned` },
            h('span', { class: 'medal-mini', html: medalSVG(part, { size: 44, title: false }) }),
            `Level ${part} medal`);
    }
    const { done, total } = partProgress(demos, part, visited);
    return h('span', { class: 'part-progress', title: `Visit all ${total} demos of level ${part} to earn the level ${part} medal` },
        h('span', { class: 'progress-track' }, h('span', { class: 'progress-fill', style: `width: ${(100 * done / total).toFixed(0)}%` })),
        `${done} / ${total} visited`);
}

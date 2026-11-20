// The "⋯" menu of the top bar: citation, figure and animation export,
// teaching mode, the glossary, and the useful definitions. The colour-blind
// palette has its own top-bar button (theme.js), and the landing page opens the
// citation directly.
import { sandboxed } from './bridge.js';
import { syncPlayer } from './frame.js';
import { go } from './page.js';
import { h } from './ui.js';

const pad2 = (n) => String(n).padStart(2, '0');

// ------------------------------------------------------------------ citation
// The citation of the tutorial (one article; the demos follow it in two levels),
// in one place. It holds placeholders until the article appears; update AUTHORS,
// BIB_AUTHORS, and PUBLICATION (and the README citation) first with the preprint
// (entry 'misc'; fields such as eprint, archivePrefix, primaryClass) and later
// with the published article (entry 'article'; journal, volume, number, pages,
// doi). The dialog and the BibTeX follow, and the placeholder note disappears
// once AUTHORS is filled in.
const AUTHORS = '<Authors>';                     // citation text
const BIB_AUTHORS = '<Authors>';                 // BibTeX: 'Last, First and Last, First'
const PUBLICATION = { details: '<publication details>', year: '<year>', entry: 'misc', fields: { howpublished: '<publication details>' } };
const PLACEHOLDER = AUTHORS.startsWith('<');
const TITLE = 'Shape Servoing of Deformable Objects: A Tutorial';
const BIB_TITLE = 'Shape Servoing of Deformable Objects: A Tutorial';
const LEVELS = { 1: 'Level 1 · Fundamentals', 2: 'Level 2 · Advanced concepts' };
const demoLabel = (demo) => `Interactive demo ${pad2(demo.number)}: ${demo.title}`;

function citation(demo) {
    const { details, year } = PUBLICATION;
    return `${AUTHORS}, "${TITLE}," ${details}, ${year}.`
        + (demo ? ` ${demoLabel(demo)}.` : '');
}

function bibtex(demo) {
    const { entry, year, fields } = PUBLICATION;
    const all = [['author', BIB_AUTHORS], ['title', BIB_TITLE], ...Object.entries(fields),
        ['year', year], ...(demo ? [['note', demoLabel(demo)]] : [])];
    const width = Math.max(...all.map(([key]) => key.length));
    return [`@${entry}{shapeservoing_tutorial,`,
        ...all.map(([key, value]) => `  ${key.padEnd(width)} = {${value}},`), '}'].join('\n');
}

// The address of this page through the player, without the teaching-mode flag.
function stableLink(demo) {
    const page = demo ? `demo.html?id=${demo.id}` : (location.pathname.split('/').pop() || 'index.html');
    return new URL(page === 'index.html' ? 'player.html' : `player.html?p=${encodeURIComponent(page)}`, location.href).href;
}

async function copy(text, button) {
    try {
        await navigator.clipboard.writeText(text);
    } catch {
        const area = h('textarea', { style: 'position:fixed;opacity:0' });
        area.value = text;
        document.body.append(area);
        area.select();
        document.execCommand('copy');
        area.remove();
    }
    button.textContent = 'Copied ✓';
    setTimeout(() => { button.textContent = 'Copy'; }, 1400);
}

export function citeDialog(demo = null) {
    const block = (title, text) => {
        const copyButton = h('button', { class: 'button small', type: 'button' }, 'Copy');
        copyButton.addEventListener('click', () => copy(text, copyButton));
        return h('div', { class: 'cite-block' },
            h('div', { class: 'cite-block-head' }, h('span', {}, title), copyButton), h('pre', {}, text));
    };
    const close = h('button', { class: 'button', type: 'button' }, 'Close');
    const dialog = h('dialog', { class: 'cite-dialog', 'aria-labelledby': 'citeTitle' },
        h('p', { class: 'kicker' }, demo ? `${LEVELS[demo.part]} · Demo ${pad2(demo.number)}` : 'Interactive tutorial'),
        h('h2', { id: 'citeTitle' }, demo ? 'Cite this demo' : 'Cite the tutorial'),
        h('p', { class: 'cite-note' }, (PLACEHOLDER ? 'Author names and publication details are placeholders until the article appears. ' : '')
            + 'The link opens this page wherever the tutorial runs locally.'),
        block('Link', stableLink(demo)),
        block('Citation', citation(demo)),
        block('BibTeX', bibtex(demo)),
        h('div', { class: 'cite-actions' }, close));
    close.addEventListener('click', () => dialog.close());
    dialog.addEventListener('click', (event) => {           // a click on the backdrop closes
        const r = dialog.getBoundingClientRect();
        if (event.clientX < r.left || event.clientX > r.right || event.clientY < r.top || event.clientY > r.bottom) dialog.close();
    });
    dialog.addEventListener('close', () => dialog.remove());
    document.body.append(dialog);
    dialog.showModal();
}

// ------------------------------------------------------------------ figure and animation export
// One image of the workspace canvases at their places on screen, without the
// controls between them: empty horizontal bands (toolbars, sliders) collapse
// to a small gap, so the figure keeps the arrangement of the demo.
function composer(root) {
    const canvases = [...root.querySelectorAll('canvas')].filter((c) => c.clientWidth > 0 && c.clientHeight > 0);
    if (!canvases.length) return null;
    const GAP = 12;
    const boxes = canvases.map((c) => {
        const r = c.getBoundingClientRect();
        return { c, x: r.left + c.clientLeft, y: r.top + c.clientTop, w: c.clientWidth, h: c.clientHeight };
    });
    const bands = [];
    for (const b of [...boxes].sort((a, z) => a.y - z.y)) {
        const last = bands[bands.length - 1];
        if (last && b.y <= last.y1) last.y1 = Math.max(last.y1, b.y + b.h);
        else bands.push({ y0: b.y, y1: b.y + b.h });
    }
    let y = GAP;
    for (const band of bands) { band.top = y; y += band.y1 - band.y0 + GAP; }
    for (const b of boxes) {
        const band = bands.find((s) => b.y >= s.y0 && b.y < s.y1 + 0.5);
        b.y = band.top + b.y - band.y0;
    }
    const x0 = Math.min(...boxes.map((b) => b.x)) - GAP;
    const width = Math.max(...boxes.map((b) => b.x + b.w)) + GAP - x0, height = y;
    const scale = Math.max(1, ...canvases.map((c) => c.width / c.clientWidth));
    const out = document.createElement('canvas');
    out.width = 2 * Math.round(width * scale / 2);          // even sizes suit video encoders
    out.height = 2 * Math.round(height * scale / 2);
    const ctx = out.getContext('2d');
    const background = getComputedStyle(root).getPropertyValue('--surface').trim() || '#ffffff';
    const draw = () => {
        ctx.setTransform(out.width / width, 0, 0, out.height / height, 0, 0);
        ctx.fillStyle = background;
        ctx.fillRect(0, 0, width, height);
        for (const b of boxes) ctx.drawImage(b.c, b.x - x0, b.y, b.w, b.h);
    };
    draw();
    return { canvas: out, draw };
}

// A host that serves the pages in a sandboxed origin (e.g., anonymous.4open.science)
// blocks downloads; there, the figure or video is shown in a dialog instead, to be
// saved with the browser's context menu.
function showForSaving(blob, name) {
    const url = URL.createObjectURL(blob);
    const media = blob.type.startsWith('video/')
        ? h('video', { src: url, controls: true, class: 'export-media' })
        : h('img', { src: url, alt: name, class: 'export-media' });
    const close = h('button', { class: 'button', type: 'button' }, 'Close');
    const dialog = h('dialog', { class: 'cite-dialog export-dialog', 'aria-label': name },
        h('h2', {}, name),
        h('p', { class: 'cite-note' }, 'This host does not allow downloads: right-click the '
            + `${blob.type.startsWith('video/') ? 'video' : 'image'} and choose “Save as” to keep it.`),
        media, h('div', { class: 'cite-actions' }, close));
    close.addEventListener('click', () => dialog.close());
    dialog.addEventListener('close', () => { dialog.remove(); URL.revokeObjectURL(url); });
    document.body.append(dialog);
    dialog.showModal();
}

function download(blob, name) {
    if (sandboxed()) { showForSaving(blob, name); return; }
    const url = URL.createObjectURL(blob);
    const link = h('a', { href: url, download: name });
    document.body.append(link);
    link.click();
    link.remove();
    setTimeout(() => URL.revokeObjectURL(url), 4000);
}

function savePng(root, name) {
    composer(root)?.canvas.toBlob((blob) => blob && download(blob, `${name}.png`), 'image/png');
}

const VIDEO_TYPES = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm', 'video/mp4'];
const MAX_SECONDS = 60;
const videoType = () => (window.MediaRecorder && HTMLCanvasElement.prototype.captureStream
    ? VIDEO_TYPES.find((t) => MediaRecorder.isTypeSupported(t)) : undefined);
let recording = null;

// Records the composed canvases until the user stops (or MAX_SECONDS), then saves the video.
function startRecording(root, name) {
    const type = videoType(), frame = composer(root);
    if (!type || !frame) return;
    const recorder = new MediaRecorder(frame.canvas.captureStream(30), { mimeType: type, videoBitsPerSecond: 8e6 });
    const chunks = [];
    recorder.ondataavailable = (event) => { if (event.data.size) chunks.push(event.data); };
    const clock = h('span', {}, 'REC 0:00');
    const pill = h('button', { class: 'rec-pill', type: 'button', title: 'Stop recording and save the video' },
        h('i', { class: 'rec-dot', 'aria-hidden': 'true' }), clock, h('span', { class: 'rec-stop' }, 'Stop'));
    const started = performance.now();
    let raf = 0;
    const loop = () => { frame.draw(); raf = requestAnimationFrame(loop); };
    const timer = setInterval(() => {
        const s = Math.floor((performance.now() - started) / 1000);
        clock.textContent = `REC ${Math.floor(s / 60)}:${pad2(s % 60)}`;
        if (s >= MAX_SECONDS) stop();
    }, 250);
    function stop() { if (recorder.state !== 'inactive') recorder.stop(); }
    recorder.onstop = () => {
        cancelAnimationFrame(raf);
        clearInterval(timer);
        pill.remove();
        recording = null;
        download(new Blob(chunks, { type }), `${name}.${type.startsWith('video/mp4') ? 'mp4' : 'webm'}`);
    };
    pill.addEventListener('click', stop);
    document.body.append(pill);
    loop();
    recorder.start(500);
    recording = { stop };
}

// ------------------------------------------------------------------ teaching mode
// A demo without its lesson column and page chrome, with larger text and
// canvases. The mode is kept in the address (?present) and in the demo links,
// so a lecture can move from demo to demo; Esc or "Exit" leaves it.
const isTeaching = () => document.documentElement.classList.contains('teaching');

function topDocument() {
    try { if (window.top !== window && window.top.document.fullscreenEnabled) return window.top.document; } catch { /* other origin */ }
    return null;
}

async function enterFullscreen() {
    for (const doc of [topDocument(), document]) {
        if (!doc) continue;
        try { await doc.documentElement.requestFullscreen(); return; } catch { /* try the next document */ }
    }
}

function exitFullscreen() {
    for (const doc of [topDocument(), document]) {
        try { if (doc?.fullscreenElement) doc.exitFullscreen(); } catch { /* not in fullscreen */ }
    }
}

function withPresent(href, on) {
    const url = new URL(href, location.href);
    if (on) url.searchParams.set('present', ''); else url.searchParams.delete('present');
    return url;
}

export function setTeaching(on, { fullscreen = true } = {}) {
    document.documentElement.classList.toggle('teaching', on);
    try { history.replaceState(null, '', withPresent(location.href, on)); } catch { /* sandboxed document */ }
    syncPlayer();
    for (const link of document.querySelectorAll('a[href^="demo.html?id="]')) {
        link.setAttribute('href', `demo.html${withPresent(link.getAttribute('href'), on).search}`);
    }
    if (!fullscreen) return;
    if (on) enterFullscreen(); else exitFullscreen();
}

// ------------------------------------------------------------------ the menu
export function initTools(button, { demo = null, workspace = null } = {}) {
    if (!button) return;
    const wrap = h('span', { class: 'tools' });
    button.replaceWith(wrap);
    const menu = h('div', { class: 'tools-menu', role: 'menu', id: 'toolsMenu', hidden: true, 'aria-label': 'More options' });
    wrap.append(button, menu);
    button.setAttribute('aria-haspopup', 'menu');
    button.setAttribute('aria-controls', 'toolsMenu');
    button.setAttribute('aria-expanded', 'false');
    const name = demo ? `shape-servoing-demo-${pad2(demo.number)}-${demo.id}` : 'shape-servoing';
    const page = location.pathname.split('/').pop();

    const item = (icon, text, onSelect, { checked, title, disabled = false } = {}) => {
        const el = h('button', {
            type: 'button', role: checked === undefined ? 'menuitem' : 'menuitemcheckbox',
            'aria-checked': checked === undefined ? null : String(checked), title, disabled, tabindex: '-1',
        }, h('span', { class: 'tools-icon', 'aria-hidden': 'true' }, icon), h('span', {}, text));
        el.addEventListener('click', () => { close(); onSelect(); });
        return el;
    };

    function render() {
        const entries = [item('❝', demo ? 'Cite this demo…' : 'Cite the tutorial…', () => citeDialog(demo))];
        if (demo && workspace) {
            const canExport = !!workspace.querySelector('canvas');
            entries.push(
                item('⤓', 'Save figure (PNG)', () => savePng(workspace, name),
                    { disabled: !canExport, title: 'The canvases of this demo as one image, without the controls' }),
                recording
                    ? item('■', 'Stop recording', () => recording?.stop())
                    : item('●', 'Record animation', () => startRecording(workspace, name),
                        { disabled: !canExport || !videoType(),
                            title: videoType() ? `Record the canvases as a video (up to ${MAX_SECONDS} s); stop with the REC button`
                                : 'Video recording is not supported by this browser' }),
                item('⛶', 'Teaching mode', () => setTeaching(!isTeaching()),
                    { checked: isTeaching(), title: 'Larger demo without the lesson, for lectures and screen sharing (Esc to leave)' }));
        }
        const pages = [['≡', 'Glossary', 'glossary.html'], ['∂', 'Useful definitions', 'definitions.html']]
            .filter(([, , href]) => href !== page);
        entries.push(h('hr'), ...pages.map(([icon, text, href]) => item(icon, text, () => go(href))));
        menu.replaceChildren(...entries);
    }

    const items = () => [...menu.querySelectorAll('button:not(:disabled)')];
    const onOutside = (event) => { if (!wrap.contains(event.target)) close(); };
    function open() {
        render();
        menu.hidden = false;
        button.setAttribute('aria-expanded', 'true');
        document.addEventListener('pointerdown', onOutside);
        items()[0]?.focus();
    }
    function close(refocus = false) {
        if (menu.hidden) return;
        menu.hidden = true;
        button.setAttribute('aria-expanded', 'false');
        document.removeEventListener('pointerdown', onOutside);
        if (refocus) button.focus();
    }
    button.addEventListener('click', () => (menu.hidden ? open() : close()));
    menu.addEventListener('keydown', (event) => {
        const list = items(), i = list.indexOf(document.activeElement);
        const moves = { ArrowDown: i + 1, ArrowUp: i - 1, Home: 0, End: list.length - 1 };
        if (event.key in moves) {
            event.preventDefault();
            list[(moves[event.key] + list.length) % list.length]?.focus();
        } else if (event.key === 'Escape') {
            event.preventDefault();
            event.stopPropagation();
            close(true);
        } else if (event.key === 'Tab') {
            close();
        }
    });

    if (!demo) return;
    // Teaching mode: from the address, an "Exit" button in the top bar, and Esc.
    const exit = h('button', { class: 'button small teaching-exit', type: 'button', title: 'Leave teaching mode (Esc)' }, '✕ Exit teaching mode');
    exit.addEventListener('click', () => setTeaching(false));
    wrap.before(exit);
    if (new URLSearchParams(location.search).has('present')) setTeaching(true, { fullscreen: false });
    document.addEventListener('keydown', (event) => {
        if (event.key === 'Escape' && isTeaching() && menu.hidden && !document.querySelector('dialog[open]')) setTeaching(false);
    });
}

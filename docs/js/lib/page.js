// Common start of every page: player frame, theme and palette, music, the ⋯
// menu, and the equations. On hosts without storage (e.g., the sandboxed pages of
// anonymous.4open.science), the links between the pages carry the settings, the
// progress, and the music that is playing (?st=, ?music=<track>-<seconds>).
import { enterPlayer, syncPlayer } from './frame.js';
import { renderMath } from './math.js';
import { initMusic } from './music.js';
import { STATE_PARAM, carriedState, storageAvailable } from './store.js';
import { initPaletteButton, initTheme, toggleTheme } from './theme.js';

const INTERNAL = /^(index|guide|glossary|definitions|demo|player)\.html(\?[^#]*)?(#.*)?$/;
let music = null;

// The address of an internal page with the settings it should carry (unchanged
// where storage keeps them).
export function carry(href) {
    const match = INTERNAL.exec(href);
    if (!match || storageAvailable()) return href;
    const params = new URLSearchParams(match[2] || '');
    const state = carriedState();
    if (state) params.set(STATE_PARAM, state); else params.delete(STATE_PARAM);
    if (music?.on && music.position) params.set('music', `${music.position.track}-${music.position.time.toFixed(1)}`);
    else params.delete('music');
    const query = params.toString().replace(/present=(&|$)/, 'present$1');
    return `${match[1]}.html${query ? `?${query}` : ''}${match[3] || ''}`;
}

// Navigate to an internal page.
export function go(href) { location.href = carry(href); }

function carryStateInLinks() {
    document.addEventListener('click', (event) => {
        const link = event.target.closest?.('a[href]');
        if (!link || link.target === '_blank') return;
        const href = link.getAttribute('href');
        const carried = carry(href);
        if (carried !== href) link.setAttribute('href', carried);
    }, true);
}

// Returns false if the page is being moved into player.html (stop initializing).
export async function initPage({ math = true } = {}) {
    if (enterPlayer()) await new Promise(() => {});        // reloading inside player.html
    syncPlayer();
    initTheme();
    document.getElementById('themeButton')?.addEventListener('click', toggleTheme);
    initPaletteButton(document.getElementById('paletteButton'));
    music = initMusic(document.getElementById('musicButton')) || null;
    if (!storageAvailable()) carryStateInLinks();
    if (math && window.katex) renderMath(document);
}

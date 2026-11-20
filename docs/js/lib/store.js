// Settings and progress kept between visits: the theme, the colour palette, the
// demos visited, and the medals. They live in localStorage. Where storage is
// unavailable (e.g., the sandboxed pages of anonymous.4open.science, where even
// reading localStorage throws), they live in memory for the current page, and
// the links between the pages carry them in the address (?st=..., js/lib/page.js).

const PARAM = 'st';
const memory = new Map();

function probe() {
    try {
        localStorage.setItem('shape-servoing-probe', '1');
        localStorage.removeItem('shape-servoing-probe');
        return true;
    } catch {
        return false;
    }
}
const persistent = probe();

// Values carried by the link that opened this page (base64url JSON of the entries).
try {
    const carried = new URLSearchParams(location.search).get(PARAM);
    if (carried) {
        const json = atob(carried.replace(/-/g, '+').replace(/_/g, '/'));
        for (const [key, value] of Object.entries(JSON.parse(decodeURIComponent(escape(json))))) {
            if (typeof key === 'string' && typeof value === 'string') {
                memory.set(key, value);
                if (persistent) { try { localStorage.setItem(key, value); } catch { /* full */ } }
            }
        }
    }
} catch { /* malformed: ignore */ }

export const storageAvailable = () => persistent;

export function getItem(key) {
    if (persistent) {
        try { return localStorage.getItem(key); } catch { return memory.get(key) ?? null; }
    }
    return memory.has(key) ? memory.get(key) : null;
}

export function setItem(key, value) {
    memory.set(key, String(value));
    if (persistent) { try { localStorage.setItem(key, String(value)); } catch { /* full or blocked */ } }
}

export function removeItem(key) {
    memory.delete(key);
    if (persistent) { try { localStorage.removeItem(key); } catch { /* blocked */ } }
}

// The kept values, encoded for a link (null when there is nothing to carry).
export function carriedState() {
    if (!memory.size) return null;
    const json = unescape(encodeURIComponent(JSON.stringify(Object.fromEntries(memory))));
    return btoa(json).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
}

export const STATE_PARAM = PARAM;

// Reload the page, keeping the values where they live only in memory.
export function reloadKeepingSettings() {
    if (persistent) { location.reload(); return; }
    const url = new URL(location.href);
    const state = carriedState();
    if (state) url.searchParams.set(PARAM, state); else url.searchParams.delete(PARAM);
    location.replace(url.href);
}

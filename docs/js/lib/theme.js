// Light / dark theme, and the colour palette read from the CSS variables, so
// canvases and HTML always share one set of colours. The choices are kept by
// js/lib/store.js and, inside player.html, also by the player, which passes them
// to each page (js/lib/bridge.js).
import { fromPage, fromPlayer, inFrame, toPage, toPlayer } from './bridge.js';
import { getItem, reloadKeepingSettings, setItem } from './store.js';

const KEY = 'shape-servoing-theme';
const PALETTE_KEY = 'shape-servoing-palette';
const listeners = new Set();
let cached = null;

function setTheme(theme) {
    if ((theme !== 'dark' && theme !== 'light') || document.documentElement.dataset.theme === theme) return;
    document.documentElement.dataset.theme = theme;
    setItem(KEY, theme);
    cached = null;
    listeners.forEach((fn) => fn());
}

export function initTheme() {
    const saved = getItem(KEY), palette = getItem(PALETTE_KEY);
    const system = window.matchMedia('(prefers-color-scheme: dark)').matches ? 'dark' : 'light';
    document.documentElement.dataset.theme = saved === 'dark' || saved === 'light' ? saved : system;
    if (palette === 'cvd') document.documentElement.dataset.palette = 'cvd';
    cached = null;
    if (inFrame()) {
        fromPlayer('theme', ({ theme, palette: wanted }) => {
            setTheme(theme);
            if ((wanted === 'cvd') !== colourBlindPalette() && (wanted === 'cvd' || wanted === 'default')) {
                setItem(PALETTE_KEY, wanted);
                reloadKeepingSettings();
            }
        });
        toPlayer('theme-hello');
    }
}

// Player side: keep the theme and palette of the pages loaded in `frame`.
export function serveTheme(frame) {
    fromPage(frame, 'theme', ({ theme, palette }) => {
        setTheme(theme);
        if (palette === 'cvd' || palette === 'default') setItem(PALETTE_KEY, palette);
    });
    fromPage(frame, 'theme-hello', () => toPage(frame, 'theme', {
        theme: document.documentElement.dataset.theme, palette: getItem(PALETTE_KEY) || 'default' }));
}

// Optional colour-blind palette (css: [data-palette="cvd"]), in light and dark.
// Legends take their colours when a demo mounts, so switching reloads the page.
export const colourBlindPalette = () => document.documentElement.dataset.palette === 'cvd';
export function setColourBlindPalette(on) {
    const palette = on ? 'cvd' : 'default';
    setItem(PALETTE_KEY, palette);
    toPlayer('theme', { theme: document.documentElement.dataset.theme, palette });
    reloadKeepingSettings();
}

// The eye button of the top bar switches the colour-blind palette on and off.
export function initPaletteButton(button) {
    if (!button) return;
    button.setAttribute('aria-pressed', String(colourBlindPalette()));
    button.addEventListener('click', () => setColourBlindPalette(!colourBlindPalette()));
}

export function toggleTheme() {
    setTheme(document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark');
    toPlayer('theme', { theme: document.documentElement.dataset.theme, palette: colourBlindPalette() ? 'cvd' : 'default' });
}

export function onThemeChange(fn) { listeners.add(fn); }

const NAMES = ['bg', 'surface', 'surface-soft', 'ink', 'ink-soft', 'muted', 'line', 'line-strong',
    'accent', 'accent-strong', 'accent-soft', 'canvas', 'grid', 'grid-strong', 'robot', 'robot-deep',
    'joint', 'rod', 'feature', 'feature-ring', 'target', 'target-point', 'grip-l', 'grip-r', 'red',
    'aqua', 'green', 'rep-points', 'rep-edges', 'rep-angles', 'div-neg', 'div-pos', 'div-mid'];

// palette().rod, palette().gripL, ... (CSS names in camelCase). The page
// background, canvas, grid, robot, and accent colours depend on the demo level
// (level 2 pages and menu cards carry data-part="2"), so the palette is
// read from the element's context; entity colours are the same in both levels.
export function palette(el = document.documentElement) {
    const host = (el && el.closest && el.closest('[data-part]')) || document.documentElement;
    const key = host.dataset.part || 'root';
    if (!cached) cached = new Map();
    if (cached.has(key)) return cached.get(key);
    const style = getComputedStyle(host);
    const p = { dark: document.documentElement.dataset.theme === 'dark' };
    for (const name of NAMES) {
        const name2 = name.replace(/-([a-z])/g, (_, c) => c.toUpperCase());
        p[name2] = style.getPropertyValue(`--${name}`).trim();
    }
    // One colour per robot coordinate, left gripper violet tones, right orange.
    p.dof = p.dark
        ? ['#8d80ee', '#b3aaf5', '#6b5ed6', '#f08a5a', '#f5b08f', '#d9692f']
        : ['#4a3aa7', '#8a7fd6', '#2e2270', '#eb6834', '#f2a07c', '#b84a1c'];
    // Plot series; the colour-blind variant follows the Okabe-Ito set.
    const cvd = colourBlindPalette();
    p.series = p.dark
        ? (cvd ? ['#4ba3e3', '#f2b52a', '#1db38a', '#e09bc4', '#8ed0f2', '#f07f3c']
            : ['#4b93ea', '#2bbd88', '#f2b52a', '#4caf63', '#8d80ee', '#ec6b6a'])
        : (cvd ? ['#0072b2', '#e69f00', '#009e73', '#cc79a7', '#56b4e9', '#d55e00']
            : ['#2a78d6', '#1baf7a', '#eda100', '#2e8540', '#4a3aa7', '#e34948']);
    cached.set(key, p);
    return p;
}

// colour helpers
const toRgb = (hex) => [1, 3, 5].map((i) => parseInt(hex.slice(i, i + 2), 16));
export function mix(a, b, t) {
    const x = toRgb(a), y = toRgb(b);
    return '#' + x.map((v, i) => Math.round(v + (y[i] - v) * t).toString(16).padStart(2, '0')).join('');
}
export function alpha(hex, a) {
    const [r, g, b] = toRgb(hex);
    return `rgba(${r}, ${g}, ${b}, ${a})`;
}

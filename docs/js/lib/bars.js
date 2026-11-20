// Bar chart for short vectors (robot directions, errors, eigenvalues).
import { palette, onThemeChange } from './theme.js';

const tickText = (v) => (v === 0 ? '0'
    : Math.abs(v) >= 1e3 || Math.abs(v) < 1e-2 ? v.toExponential(0) : String(+v.toPrecision(2)));

export class BarChart {
    // opts: {labels, colors (array or fn(i, v)), range: [lo, hi] | null, logY, unit, yTicks}
    constructor(canvas, opts = {}) {
        this.canvas = canvas;
        this.opts = { range: null, logY: false, ...opts };
        this.values = [];
        this.ghost = null;
        new ResizeObserver(() => this.draw()).observe(canvas);
        onThemeChange(() => this.draw());
    }

    set(values, { ghost = null, highlight = -1 } = {}) {
        this.values = values;
        this.ghost = ghost;
        this.highlight = highlight;
        this.draw();
    }

    draw() {
        const canvas = this.canvas;
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = canvas.clientWidth, h = canvas.clientHeight;
        if (!w || !h) return;
        if (canvas.width !== Math.round(w * dpr)) { canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr); }
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, w, h);
        const p = palette();
        const font = getComputedStyle(document.body).fontFamily;
        const n = this.values.length;
        if (!n) return;
        const { logY } = this.opts;
        const all = [...this.values, ...(this.ghost || [])].filter(Number.isFinite);
        let [lo, hi] = this.opts.range || [Math.min(0, ...all), Math.max(0, ...all)];
        if (logY) [lo, hi] = this.opts.range || [10 ** Math.floor(Math.log10(Math.min(...all.filter((v) => v > 0)))), 10 ** Math.ceil(Math.log10(Math.max(...all)))];
        if (hi - lo < 1e-12) hi = lo + 1;
        const L = this.opts.yTicks ? 38 : 8, R = w - 8, T = 8, B = h - (this.opts.labels ? 20 : 8);
        const f = (v) => (logY ? Math.log10(Math.max(v, 1e-300)) : v);
        const Y = (v) => B - (f(Math.min(Math.max(v, lo), hi)) - f(lo)) / (f(hi) - f(lo)) * (B - T);
        const base = logY ? B : Y(0);
        const slot = (R - L) / n, bw = Math.min(28, slot * 0.62);
        ctx.strokeStyle = p.gridStrong; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(L, base); ctx.lineTo(R, base); ctx.stroke();
        this.values.forEach((v, i) => {
            const x = L + (i + 0.5) * slot - bw / 2;
            if (this.ghost && Number.isFinite(this.ghost[i])) {
                ctx.strokeStyle = p.muted; ctx.setLineDash([3, 3]);
                const gy = Y(this.ghost[i]);
                ctx.strokeRect(x, Math.min(gy, base), bw, Math.abs(base - gy));
                ctx.setLineDash([]);
            }
            if (!Number.isFinite(v)) return;
            const colors = this.opts.colors;
            ctx.fillStyle = typeof colors === 'function' ? colors(i, v) : (colors?.[i] || p.rod);
            ctx.globalAlpha = this.highlight >= 0 && this.highlight !== i ? 0.35 : 1;
            const y = Y(v);
            ctx.fillRect(x, Math.min(y, base), bw, Math.max(1, Math.abs(base - y)));
            ctx.globalAlpha = 1;
        });
        if (this.opts.labels) {
            ctx.fillStyle = p.muted; ctx.font = `600 10px ${font}`;
            ctx.textAlign = 'center'; ctx.textBaseline = 'top';
            this.opts.labels.forEach((t, i) => ctx.fillText(t, L + (i + 0.5) * slot, B + 5));
        }
        if (this.opts.yTicks) {
            ctx.strokeStyle = p.gridStrong; ctx.lineWidth = 1;
            ctx.beginPath(); ctx.moveTo(L - 2, T); ctx.lineTo(L - 2, B); ctx.stroke();
            ctx.fillStyle = p.muted; ctx.font = `600 9px ${font}`;
            ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
            for (const v of logY ? [lo, hi] : [...new Set([lo, 0, hi])]) ctx.fillText(tickText(v), L - 6, Y(v));
        }
    }
}

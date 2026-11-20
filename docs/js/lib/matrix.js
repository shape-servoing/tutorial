// Heat map of a signed matrix (Jacobians, data matrices), with a diverging
// blue-red scale as in the paper figures, labels, and hover values.
import { palette, mix, alpha, onThemeChange } from './theme.js';

export class MatrixView {
    // opts: {rowLabels, colLabels, colColors, annotate, cell (max px), title}
    constructor(canvas, opts = {}) {
        this.canvas = canvas;
        this.opts = { annotate: false, maxCell: 34, ...opts };
        this.matrix = null;
        this.highlight = { col: -1, row: -1 };
        this.hover = null;
        this.flash = 0;
        new ResizeObserver(() => this.draw()).observe(canvas);
        onThemeChange(() => this.draw());
        canvas.addEventListener('pointermove', (e) => {
            const r = canvas.getBoundingClientRect();
            this.hover = this.cellAt(e.clientX - r.left, e.clientY - r.top);
            this.draw();
        });
        canvas.addEventListener('pointerleave', () => { this.hover = null; this.draw(); });
        canvas.addEventListener('click', (e) => {
            const r = canvas.getBoundingClientRect();
            const cell = this.cellAt(e.clientX - r.left, e.clientY - r.top);
            if (cell && this.opts.onClick) this.opts.onClick(cell);
        });
    }

    set(matrix, { vmax, col = -1, row = -1 } = {}) {
        this.matrix = matrix;
        this.vmax = vmax ?? Math.max(1e-12, ...matrix.flat().map((v) => Math.abs(v ?? 0)));
        this.highlight = { col, row };
        this.draw();
    }

    layout() {
        const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
        const rows = this.matrix.length, cols = this.matrix[0].length;
        const left = this.opts.rowLabels ? 34 : 4, top = this.opts.colLabels ? 18 : 4;
        const cell = Math.min(this.opts.maxCell, (w - left - 4) / cols, (h - top - 16) / rows);
        const x0 = left + Math.max(0, (w - left - 4 - cell * cols) / 2);
        return { w, h, rows, cols, cell, x0, y0: top };
    }

    cellAt(x, y) {
        if (!this.matrix) return null;
        const { rows, cols, cell, x0, y0 } = this.layout();
        const c = Math.floor((x - x0) / cell), r = Math.floor((y - y0) / cell);
        return r >= 0 && r < rows && c >= 0 && c < cols ? { r, c } : null;
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
        if (!this.matrix) return;
        const p = palette();
        const font = getComputedStyle(document.body).fontFamily;
        const { rows, cols, cell, x0, y0 } = this.layout();
        for (let r = 0; r < rows; r++) {
            for (let c = 0; c < cols; c++) {
                const v = this.matrix[r][c];
                if (v === null || v === undefined) {
                    ctx.strokeStyle = p.grid; ctx.lineWidth = 1;
                    ctx.strokeRect(x0 + c * cell + 1, y0 + r * cell + 1, cell - 2, cell - 2);
                    continue;
                }
                ctx.fillStyle = divergent(v, this.vmax, p);
                ctx.fillRect(x0 + c * cell + 0.5, y0 + r * cell + 0.5, cell - 1, cell - 1);
                if (this.opts.annotate && cell > 26) {
                    ctx.fillStyle = Math.abs(v) / this.vmax > 0.6 ? '#ffffff' : p.ink;
                    ctx.font = `${Math.min(10, cell * 0.32)}px ${font}`;
                    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
                    // Rounding noise (e.g. -1e-17) is shown as 0.00, not -0.00.
                    ctx.fillText((Math.abs(v) < 5e-8 ? 0 : v).toFixed(2), x0 + (c + 0.5) * cell, y0 + (r + 0.5) * cell);
                }
            }
        }
        const { col, row } = this.highlight;
        ctx.lineWidth = 2;
        ctx.strokeStyle = p.ink;
        if (col >= 0) ctx.strokeRect(x0 + col * cell, y0, cell, rows * cell);
        if (row >= 0) ctx.strokeRect(x0, y0 + row * cell, cols * cell, cell);
        if (this.flash > 0) {
            ctx.strokeStyle = alpha(p.accent, this.flash);
            ctx.lineWidth = 3;
            ctx.strokeRect(x0 - 2, y0 - 2, cols * cell + 4, rows * cell + 4);
        }

        ctx.font = `600 10px ${font}`;
        ctx.textBaseline = 'middle';
        if (this.opts.colLabels) {
            ctx.textAlign = 'center';
            this.opts.colLabels.forEach((t, c) => {
                ctx.fillStyle = this.opts.colColors?.[c] || p.muted;
                ctx.fillText(t, x0 + (c + 0.5) * cell, y0 - 9);
            });
        }
        if (this.opts.rowLabels) {
            ctx.textAlign = 'right';
            ctx.fillStyle = p.muted;
            this.opts.rowLabels.forEach((t, r) => ctx.fillText(t, x0 - 5, y0 + (r + 0.5) * cell));
        }
        const v = this.hover ? this.matrix[this.hover.r][this.hover.c] : null;
        ctx.textAlign = 'left';
        ctx.fillStyle = p.muted;
        ctx.font = `10px ${font}`;
        const caption = this.hover
            ? `[${this.opts.rowLabels?.[this.hover.r] ?? this.hover.r + 1}, ${this.opts.colLabels?.[this.hover.c] ?? this.hover.c + 1}] = ${(v ?? 0).toFixed(3)}`
            : `color scale ±${this.vmax.toFixed(2)}`;
        ctx.fillText(caption, x0, y0 + rows * cell + 9);
    }

    pulse() {
        this.flash = 1;
        const step = () => { this.flash = Math.max(0, this.flash - 0.05); this.draw(); if (this.flash > 0) requestAnimationFrame(step); };
        requestAnimationFrame(step);
    }
}

export function divergent(v, vmax, p = palette()) {
    const t = Math.min(1, Math.abs(v) / Math.max(vmax, 1e-12));
    return mix(p.divMid, v >= 0 ? p.divPos : p.divNeg, 0.08 + 0.92 * t);
}

export const FEATURE_ROWS = ['p1x', 'p1y', 'p2x', 'p2y', 'p3x', 'p3y', 'p4x', 'p4y', 'p5x', 'p5y'];
export const DOF_COLS = ['xL', 'yL', 'θL', 'xR', 'yR', 'θR'];

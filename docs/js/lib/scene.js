// The scene canvas: two planar arms, the rod, tracked points, the target.
// World coordinates are the simulation units (rest length 6, y up).
import { palette, alpha, mix, onThemeChange } from './theme.js';

const TAU = Math.PI * 2;
export const DEFAULT_VIEW = { x0: -7.8, x1: 7.8, y0: -3.3, y1: 4.9 };
const ARM = { l1: 3.9, l2: 3.4, l3: 0.75, bases: [[-6.2, -2.6], [6.2, -2.6]], floor: -3.15 };
const HANDLE_RADIUS = 0.95;   // rotation handle distance from the gripper

export class SceneView {
    constructor(canvas, { view = DEFAULT_VIEW, arms = true, camera = false, onResize } = {}) {
        this.canvas = canvas;
        this.view = view;
        this.showArms = arms;
        this.showCamera = camera;
        this.redraw = onResize || (() => {});
        new ResizeObserver(() => this.redraw()).observe(canvas);
        onThemeChange(() => this.redraw());
    }

    // ------------------------------------------------------------ frame
    begin() {
        const dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = this.canvas.clientWidth, h = this.canvas.clientHeight;
        if (!w || !h) return false;
        if (this.canvas.width !== Math.round(w * dpr) || this.canvas.height !== Math.round(h * dpr)) {
            this.canvas.width = Math.round(w * dpr);
            this.canvas.height = Math.round(h * dpr);
        }
        this.ctx = this.canvas.getContext('2d');
        this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        this.w = w; this.h = h;
        const v = this.view;
        this.scale = Math.min(w / (v.x1 - v.x0), h / (v.y1 - v.y0));
        this.ox = (w - this.scale * (v.x1 - v.x0)) / 2;
        this.oy = (h - this.scale * (v.y1 - v.y0)) / 2;
        this.p = palette(this.canvas);
        return true;
    }

    draw(fn) {
        if (!this.begin()) return;
        fn(this);
    }

    px([x, y]) { return [this.ox + (x - this.view.x0) * this.scale, this.oy + (this.view.y1 - y) * this.scale]; }
    world(px, py) { return [this.view.x0 + (px - this.ox) / this.scale, this.view.y1 - (py - this.oy) / this.scale]; }

    // ------------------------------------------------------------ background
    background({ grid = true, floor = this.showArms } = {}) {
        const { ctx, p, w, h } = this;
        ctx.clearRect(0, 0, w, h);
        ctx.fillStyle = p.canvas;
        ctx.fillRect(0, 0, w, h);
        if (grid) {
            ctx.strokeStyle = p.grid;
            ctx.lineWidth = 1;
            const [wx0, wy1] = this.world(0, 0), [wx1, wy0] = this.world(w, h);
            for (let x = Math.ceil(wx0); x <= wx1; x += 1) {
                const X = this.px([x, 0])[0];
                ctx.beginPath(); ctx.moveTo(X, 0); ctx.lineTo(X, h); ctx.stroke();
            }
            for (let y = Math.ceil(wy0); y <= wy1; y += 1) {
                const Y = this.px([0, y])[1];
                ctx.beginPath(); ctx.moveTo(0, Y); ctx.lineTo(w, Y); ctx.stroke();
            }
        }
        if (floor) {
            const Y = this.px([0, ARM.floor])[1];
            ctx.strokeStyle = p.gridStrong;
            ctx.lineWidth = 1.5;
            ctx.beginPath(); ctx.moveTo(0, Y); ctx.lineTo(w, Y); ctx.stroke();
        }
        if (this.showCamera) this.camera();
    }

    camera(at = [0, this.view.y1 - 0.45]) {
        const { ctx, p } = this;
        const [x, y] = this.px(at);
        const s = this.scale;
        ctx.save();
        ctx.strokeStyle = alpha(p.muted, 0.18);
        ctx.setLineDash([3, 5]);
        ctx.lineWidth = 1;
        for (const dx of [-4.3, 4.3]) {
            const [fx, fy] = this.px([at[0] + dx, at[1] - 4.6]);
            ctx.beginPath(); ctx.moveTo(x, y + 0.12 * s); ctx.lineTo(fx, fy); ctx.stroke();
        }
        ctx.setLineDash([]);
        ctx.fillStyle = p.robotDeep;
        roundRect(ctx, x - 0.42 * s, y - 0.2 * s, 0.84 * s, 0.34 * s, 0.07 * s);
        ctx.fill();
        ctx.fillStyle = p.joint;
        ctx.beginPath(); ctx.arc(x, y + 0.14 * s, 0.13 * s, 0, TAU); ctx.fill();
        ctx.restore();
    }

    // ------------------------------------------------------------ robot
    arms(q, { active = -1 } = {}) {
        if (!this.showArms) return;
        const poses = [armPose(ARM.bases[0], [q[0], q[1]], q[2] + Math.PI),
            armPose(ARM.bases[1], [q[3], q[4]], q[5])];
        poses.forEach((pose) => this.arm(pose));
    }

    arm(pose) {
        const { ctx, p, scale } = this;
        const chain = [pose.base, pose.elbow, pose.wrist, pose.tip];
        const widths = [0.2, 0.15, 0.1];
        ctx.save();
        ctx.lineCap = 'round';
        for (let i = 0; i < 3; i++) {
            const a = this.px(chain[i]), b = this.px(chain[i + 1]);
            ctx.strokeStyle = i === 0 ? p.robotDeep : p.robot;
            ctx.lineWidth = widths[i] * scale;
            ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
            ctx.strokeStyle = alpha(p.canvas, 0.5);
            ctx.lineWidth = Math.max(1, widths[i] * scale * 0.16);
            ctx.beginPath(); ctx.moveTo(...a); ctx.lineTo(...b); ctx.stroke();
        }
        const [bx, by] = this.px(pose.base), [, fy] = this.px([0, ARM.floor]);
        ctx.fillStyle = p.robotDeep;
        ctx.beginPath();
        ctx.moveTo(bx - 0.16 * scale, by); ctx.lineTo(bx + 0.16 * scale, by);
        ctx.lineTo(bx + 0.45 * scale, fy); ctx.lineTo(bx - 0.45 * scale, fy);
        ctx.closePath(); ctx.fill();
        for (const j of [pose.base, pose.elbow, pose.wrist]) {
            this.disc(j, 0.09 * scale, p.canvas, p.joint, Math.max(1.5, 0.03 * scale));
        }
        ctx.restore();
    }

    // Jaws clamping the rod end, in the gripper colour.
    grippers(q, { active = -1, faded = false } = {}) {
        const { p } = this;
        const ends = [[[q[0], q[1]], q[2], p.gripL], [[q[3], q[4]], q[5] + Math.PI, p.gripR]];
        ends.forEach(([tip, dir, color], side) => {
            const c = Math.cos(dir), s = Math.sin(dir), n = [-s, c];
            const open = side === active ? 0.17 : 0.14;
            const col = faded ? alpha(color, 0.35) : color;
            for (const sgn of [1, -1]) {
                const a = [tip[0] - 0.08 * c + sgn * open * n[0], tip[1] - 0.08 * s + sgn * open * n[1]];
                const b = [a[0] + 0.38 * c, a[1] + 0.38 * s];
                this.line([a, b], { color: col, width: Math.max(2.4, 0.07 * this.scale), cap: 'round' });
            }
            const back = [tip[0] - 0.08 * c, tip[1] - 0.08 * s];
            this.line([[back[0] + open * n[0], back[1] + open * n[1]], [back[0] - open * n[0], back[1] - open * n[1]]],
                { color: col, width: Math.max(2.4, 0.07 * this.scale), cap: 'round' });
        });
    }

    // Rotation handles above each gripper (drag them to rotate the gripper).
    handles(q, { active = -1 } = {}) {
        const { p } = this;
        handlePositions(q).forEach((hp, i) => {
            const grip = i === 0 ? [q[0], q[1]] : [q[3], q[4]];
            this.line([grip, hp], { color: alpha(p.accent, 0.5), width: 1.2, dash: [3, 3] });
            this.disc(hp, (i === active ? 0.17 : 0.13) * this.scale, alpha(p.canvas, 0.92), p.accent, 2);
        });
    }

    // A robot command dq (or velocity u): an arrow for each gripper translation
    // and an arc for each rotation, both multiplied by `gain` for visibility.
    command(q, dq, { color = this.p.accentStrong, gain = 1, radius = 0.6 } = {}) {
        for (const i of [0, 1]) {
            const grip = [q[3 * i], q[3 * i + 1]];
            this.arrow(grip, [grip[0] + gain * dq[3 * i], grip[1] + gain * dq[3 * i + 1]], { color, width: 2.4, head: 8 });
            this.rotationArrow(grip, radius, q[3 * i + 2] + Math.PI / 2, gain * dq[3 * i + 2], { color, width: 2 });
        }
    }

    // ------------------------------------------------------------ object
    rod(nodes, { color = this.p.rod, width = 0.11, halo = true, dash = null, alphaValue = 1 } = {}) {
        if (!nodes) return;
        if (halo) this.curve(nodes, { color: alpha(color, 0.16 * alphaValue), width: width * 2.1 * this.scale });
        this.curve(nodes, { color: alphaValue < 1 ? alpha(color, alphaValue) : color, width: width * this.scale, dash });
    }

    target(target, { rings = true } = {}) {
        if (!target) return;
        const { p } = this;
        if (target.nodes) this.curve(target.nodes, { color: p.target, width: Math.max(1.6, 0.05 * this.scale), dash: [7, 6] });
        if (rings && target.points) for (const pt of target.points) this.disc(pt, 0.15 * this.scale, null, p.targetPoint, 1.6);
    }

    points(points, { color = this.p.feature, r = 0.1, ring = this.p.featureRing } = {}) {
        for (const pt of points || []) this.disc(pt, r * this.scale, color, ring, 1.2);
    }

    // ------------------------------------------------------------ primitives
    curve(pts, { color, width = 2, dash = null }) {
        const { ctx } = this;
        const P = pts.map((q) => this.px(q));
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';
        ctx.setLineDash(dash || []);
        ctx.beginPath();
        ctx.moveTo(...P[0]);
        for (let i = 0; i < P.length - 1; i++) {
            ctx.quadraticCurveTo(P[i][0], P[i][1], (P[i][0] + P[i + 1][0]) / 2, (P[i][1] + P[i + 1][1]) / 2);
        }
        ctx.lineTo(...P[P.length - 1]);
        ctx.stroke();
        ctx.restore();
    }

    line(pts, { color, width = 1.5, dash = null, cap = 'round' }) {
        const { ctx } = this;
        ctx.save();
        ctx.strokeStyle = color;
        ctx.lineWidth = width;
        ctx.lineCap = cap;
        ctx.setLineDash(dash || []);
        ctx.beginPath();
        pts.forEach((q, i) => (i ? ctx.lineTo(...this.px(q)) : ctx.moveTo(...this.px(q))));
        ctx.stroke();
        ctx.restore();
    }

    arrow(from, to, { color, width = 1.8, head = 7, dash = null } = {}) {
        const { ctx } = this;
        const [x0, y0] = this.px(from), [x1, y1] = this.px(to);
        const len = Math.hypot(x1 - x0, y1 - y0);
        if (len < 1.5) return;
        const ang = Math.atan2(y1 - y0, x1 - x0);
        const hd = Math.min(head, len * 0.6);
        ctx.save();
        ctx.strokeStyle = color; ctx.fillStyle = color;
        ctx.lineWidth = width; ctx.lineCap = 'round';
        ctx.setLineDash(dash || []);
        ctx.beginPath(); ctx.moveTo(x0, y0);
        ctx.lineTo(x1 - 0.6 * hd * Math.cos(ang), y1 - 0.6 * hd * Math.sin(ang)); ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.moveTo(x1, y1);
        ctx.lineTo(x1 - hd * Math.cos(ang - 0.42), y1 - hd * Math.sin(ang - 0.42));
        ctx.lineTo(x1 - hd * Math.cos(ang + 0.42), y1 - hd * Math.sin(ang + 0.42));
        ctx.closePath(); ctx.fill();
        ctx.restore();
    }

    // Curved arrow around a gripper, for a rotation command (angle in rad).
    rotationArrow(center, radius, start, sweep, { color, width = 2 } = {}) {
        if (Math.abs(sweep) < 0.02) return;
        const { ctx } = this;
        const [cx, cy] = this.px(center), r = radius * this.scale;
        ctx.save();
        ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width; ctx.lineCap = 'round';
        ctx.beginPath(); ctx.arc(cx, cy, r, -start, -(start + sweep), sweep > 0); ctx.stroke();
        const end = start + sweep, tip = [cx + r * Math.cos(-end), cy + r * Math.sin(-end)];
        const tangent = -end - Math.sign(sweep) * Math.PI / 2;
        ctx.beginPath(); ctx.moveTo(...tip);
        ctx.lineTo(tip[0] - 7 * Math.cos(tangent - 0.45), tip[1] - 7 * Math.sin(tangent - 0.45));
        ctx.lineTo(tip[0] - 7 * Math.cos(tangent + 0.45), tip[1] - 7 * Math.sin(tangent + 0.45));
        ctx.closePath(); ctx.fill();
        ctx.restore();
    }

    disc(pt, r, fill, stroke, lw = 1.5) {
        const { ctx } = this;
        const [x, y] = this.px(pt);
        ctx.beginPath();
        ctx.arc(x, y, r, 0, TAU);
        if (fill) { ctx.fillStyle = fill; ctx.fill(); }
        if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = lw; ctx.stroke(); }
    }

    text(str, pt, { color = this.p.muted, size = 11, align = 'center', baseline = 'middle', weight = 600 } = {}) {
        const { ctx } = this;
        ctx.save();
        ctx.font = `${weight} ${size}px ${getComputedStyle(document.body).fontFamily}`;
        ctx.fillStyle = color; ctx.textAlign = align; ctx.textBaseline = baseline;
        ctx.fillText(str, ...this.px(pt));
        ctx.restore();
    }

    badge(str, { color = this.p.ink, x = 12, y = 12 } = {}) {
        const { ctx, p } = this;
        ctx.save();
        ctx.font = `650 12px ${getComputedStyle(document.body).fontFamily}`;
        const tw = ctx.measureText(str).width;
        ctx.fillStyle = alpha(p.accent, 0.13);
        ctx.strokeStyle = alpha(p.accent, 0.5);
        roundRect(ctx, x, y, tw + 20, 24, 7);
        ctx.fill(); ctx.stroke();
        ctx.fillStyle = color; ctx.textBaseline = 'middle';
        ctx.fillText(str, x + 10, y + 12.5);
        ctx.restore();
    }

    corner(str, { color = this.p.muted } = {}) {
        const { ctx } = this;
        ctx.save();
        ctx.font = `600 11px ${getComputedStyle(document.body).fontFamily}`;
        ctx.fillStyle = color; ctx.textAlign = 'right'; ctx.textBaseline = 'bottom';
        ctx.fillText(str, this.w - 12, this.h - 10);
        ctx.restore();
    }
}

// ---------------------------------------------------------------- dragging
// Drag the grippers (translate), their handles (rotate), or the rod (both).
// getQ() returns the current grasp; onMove(q) receives the requested grasp.
export function gripperDrag(view, { getQ, getNodes, onMove, onStart, onEnd, enabled = () => true }) {
    function hit(w) {
        const q = getQ();
        const hp = handlePositions(q);
        for (let i = 0; i < 2; i++) if (dist(w, hp[i]) < 0.3) return { mode: 'angle', side: i };
        for (let i = 0; i < 2; i++) if (dist(w, [q[3 * i], q[3 * i + 1]]) < 0.42) return { mode: 'grip', side: i };
        for (const n of getNodes?.() || []) if (dist(w, n) < 0.3) return { mode: 'body', side: -1 };
        return null;
    }
    function move(handle, w) {
        const q = [...handle.q0];
        const d = [w[0] - handle.w0[0], w[1] - handle.w0[1]];
        if (handle.mode === 'angle') {
            const i = 3 * handle.side;
            q[i + 2] = Math.atan2(w[1] - q[i + 1], w[0] - q[i]) - Math.PI / 2;
        } else if (handle.mode === 'grip') {
            const i = 3 * handle.side;
            q[i] += d[0]; q[i + 1] += d[1];
        } else {
            q[0] += d[0]; q[1] += d[1]; q[3] += d[0]; q[4] += d[1];
        }
        onMove(q);
    }
    return dragController(view, { hit, move, onStart, onEnd, enabled, prepare: (handle, w) => ({ ...handle, q0: [...getQ()], w0: w }) });
}

// Generic pointer handling: hit(w) -> handle | null, move(handle, w).
export function dragController(view, { hit, move, onStart, onEnd, enabled = () => true, prepare = (h) => h }) {
    const canvas = view.canvas;
    let active = null;
    const worldOf = (event) => {
        const rect = canvas.getBoundingClientRect();
        return view.world(event.clientX - rect.left, event.clientY - rect.top);
    };
    canvas.addEventListener('pointermove', (event) => {
        const w = worldOf(event);
        if (active) { move(active, w); return; }
        const handle = enabled() ? hit(w) : null;
        canvas.style.cursor = handle ? (handle.mode === 'angle' ? 'grab' : 'move') : 'default';
    });
    canvas.addEventListener('pointerdown', (event) => {
        if (!enabled()) return;
        const w = worldOf(event);
        const handle = hit(w);
        if (!handle) return;
        event.preventDefault();
        canvas.setPointerCapture(event.pointerId);
        active = prepare(handle, w);
        onStart?.(active);
    });
    const end = (event) => {
        if (!active) return;
        const handle = active;
        active = null;
        if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
        onEnd?.(handle);
    };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
    return { get active() { return active; } };
}

// ---------------------------------------------------------------- geometry
export function handlePositions(q) {
    return [0, 1].map((i) => {
        const t = q[3 * i + 2];
        return [q[3 * i] - HANDLE_RADIUS * Math.sin(t), q[3 * i + 1] + HANDLE_RADIUS * Math.cos(t)];
    });
}

const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1]);

// Planar 3R arm reaching `tip` with its last link along `psi`, elbow up.
function armPose(base, tip, psi) {
    let wx = tip[0] + ARM.l3 * Math.cos(psi), wy = tip[1] + ARM.l3 * Math.sin(psi);
    let dx = wx - base[0], dy = wy - base[1];
    let r = Math.hypot(dx, dy);
    const rMax = ARM.l1 + ARM.l2 - 1e-3, rMin = Math.abs(ARM.l1 - ARM.l2) + 1e-3;
    if (r > rMax || r < rMin) {
        const f = Math.min(Math.max(r, rMin), rMax) / Math.max(r, 1e-9);
        dx *= f; dy *= f; r = Math.hypot(dx, dy);
    }
    const c2 = Math.min(1, Math.max(-1, (r * r - ARM.l1 ** 2 - ARM.l2 ** 2) / (2 * ARM.l1 * ARM.l2)));
    const s2 = Math.sqrt(1 - c2 * c2);
    const phi = Math.atan2(dy, dx);
    let best = null;
    for (const sign of [1, -1]) {
        const t1 = phi - Math.atan2(ARM.l2 * sign * s2, ARM.l1 + ARM.l2 * c2);
        const elbow = [base[0] + ARM.l1 * Math.cos(t1), base[1] + ARM.l1 * Math.sin(t1)];
        if (!best || elbow[1] > best[1]) best = elbow;
    }
    return { base, elbow: best, wrist: [base[0] + dx, base[1] + dy], tip };
}

function roundRect(ctx, x, y, w, h, r) {
    ctx.beginPath();
    if (ctx.roundRect) ctx.roundRect(x, y, w, h, r); else ctx.rect(x, y, w, h);
}

// Interpolate two node arrays (for smooth playback between computed frames).
export function lerpNodes(a, b, t) {
    return a.map((p, i) => [p[0] + (b[i][0] - p[0]) * t, p[1] + (b[i][1] - p[1]) * t]);
}
export const lerp = (a, b, t) => a.map((v, i) => v + (b[i] - v) * t);
export const smoothstep = (t) => t * t * (3 - 2 * t);
export { mix };

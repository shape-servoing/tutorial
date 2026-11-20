// The elastic rod of the running example (port of shape_servoing/rod.py).
//
// A planar Kirchhoff rod with stretching and bending stiffness, discretized with
// 25 nodes and subject to gravity. Each gripper clamps one end node and the
// tangent of the adjacent edge, so the robot coordinates
// q = [x_L, y_L, theta_L, x_R, y_R, theta_R] fix four nodes. For each q, the
// quasistatic equilibrium is the configuration of the free nodes that minimizes
// the total potential energy; Newton's method with the analytic Hessian finds it.
// Nodes are arrays of [x, y] pairs.
import { cholesky, choleskySolve, interp, linspace, maxAbs } from './linalg.js';

const perp = (v) => [-v[1], v[0]];
const turning = (a, b) => Math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]);
const edges = (nodes) => nodes.slice(1).map((p, j) => [p[0] - nodes[j][0], p[1] - nodes[j][1]]);
const arclength = (points) => {
    const arc = [0];
    for (let i = 1; i < points.length; i++) {
        arc.push(arc[i - 1] + Math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1]));
    }
    return arc;
};

export class Rod {
    constructor({ nNodes = 25, length = 6.0, kStretch = 80.0, kBend = 4.0, gravity = 0.45 } = {}) {
        Object.assign(this, { nNodes, length, kStretch, kBend, gravity });
        Object.freeze(this);
    }

    get restEdge() { return this.length / (this.nNodes - 1); }

    // Node indices fixed by the grippers.
    get clamped() { return [0, 1, this.nNodes - 2, this.nNodes - 1]; }

    // ------------------------------------------------------------ boundary conditions
    clampedNodes(q) {
        const [xl, yl, tl, xr, yr, tr] = q;
        const h = this.restEdge;
        return [[xl, yl], [xl + h * Math.cos(tl), yl + h * Math.sin(tl)],
            [xr - h * Math.cos(tr), yr - h * Math.sin(tr)], [xr, yr]];
    }

    _withClamps(nodes, c) {
        this.clamped.forEach((i, j) => { nodes[i] = [c[j][0], c[j][1]]; });
        return nodes;
    }

    // Free nodes evenly spaced on the chord between the inner clamps.
    straightNodes(q) {
        const c = this.clampedNodes(q);
        const t = linspace(0, 1, this.nNodes - 2);
        const nodes = new Array(this.nNodes);
        t.forEach((ti, i) => { nodes[i + 1] = [(1 - ti) * c[1][0] + ti * c[2][0], (1 - ti) * c[1][1] + ti * c[2][1]]; });
        return this._withClamps(nodes, c);
    }

    // Starting guess of a solve without history: a cubic Hermite curve along the
    // clamped tangents, bulged along its own bulge (or gravity) to the rest length.
    tangentCurveNodes(q) {
        const c = this.clampedNodes(q);
        const h = this.restEdge;
        const tl = [(c[1][0] - c[0][0]) / h, (c[1][1] - c[0][1]) / h];
        const tr = [(c[3][0] - c[2][0]) / h, (c[3][1] - c[2][1]) / h];
        const chord = Math.max(Math.hypot(c[2][0] - c[1][0], c[2][1] - c[1][1]), h);
        const freeLength = this.length - 2 * h;
        const t = linspace(0, 1, 241);
        const hermite = t.map((s) => {
            const h00 = 2 * s ** 3 - 3 * s ** 2 + 1, h10 = s ** 3 - 2 * s ** 2 + s;
            const h01 = -2 * s ** 3 + 3 * s ** 2, h11 = s ** 3 - s ** 2;
            return [0, 1].map((d) => h00 * c[1][d] + h10 * chord * tl[d] + h01 * c[2][d] + h11 * chord * tr[d]);
        });
        const bulge = [hermite[120][0] - 0.5 * (c[1][0] + c[2][0]), hermite[120][1] - 0.5 * (c[1][1] + c[2][1])];
        const nb = Math.hypot(bulge[0], bulge[1]);
        const direction = nb > 1e-3 * chord ? [bulge[0] / nb, bulge[1] / nb] : [0, -1];
        const bumped = (height) => hermite.map((p, i) => {
            const w = height * Math.sin(Math.PI * t[i]);
            return [p[0] + w * direction[0], p[1] + w * direction[1]];
        });
        let lo = 0, hi = freeLength;
        const total = (pts) => { const a = arclength(pts); return a[a.length - 1]; };
        if (total(hermite) < freeLength) {
            for (let k = 0; k < 40; k++) {
                const mid = 0.5 * (lo + hi);
                if (total(bumped(mid)) < freeLength) lo = mid; else hi = mid;
            }
        }
        const points = bumped(lo);
        const arc = arclength(points);
        const s = linspace(0, arc[arc.length - 1], this.nNodes - 2);
        const xs = points.map((p) => p[0]), ys = points.map((p) => p[1]);
        const nodes = new Array(this.nNodes);
        s.forEach((si, i) => { nodes[i + 1] = [interp(si, arc, xs), interp(si, arc, ys)]; });
        return this._withClamps(nodes, c);
    }

    // ------------------------------------------------------------ deformation imposed on the rod
    // Axial strain of every edge, |e_j| / h - 1 (positive when stretched).
    axialStrain(nodes) {
        const h = this.restEdge;
        return edges(nodes).map((e) => Math.hypot(e[0], e[1]) / h - 1);
    }

    // Stretching plus bending energy: the potential energy without gravity.
    elasticEnergy(nodes) {
        const h = this.restEdge, e = edges(nodes);
        let stretch = 0, bend = 0;
        for (const s of this.axialStrain(nodes)) stretch += s * s;
        for (let j = 0; j + 1 < e.length; j++) bend += turning(e[j], e[j + 1]) ** 2;
        return 0.5 * this.kStretch * h * stretch + 0.5 * this.kBend / h * bend;
    }

    // ------------------------------------------------------------ total potential energy, gradient, Hessian
    energy(nodes) {
        const h = this.restEdge, e = edges(nodes);
        let stretch = 0, bend = 0, height = 0;
        for (const v of e) stretch += (Math.hypot(v[0], v[1]) - h) ** 2;
        for (let j = 0; j + 1 < e.length; j++) bend += turning(e[j], e[j + 1]) ** 2;
        for (const p of nodes) height += p[1];
        return 0.5 * this.kStretch / h * stretch + 0.5 * this.kBend / h * bend + this.gravity * h * height;
    }

    // dPi/dnodes, as [x, y] pairs.
    gradient(nodes) {
        const h = this.restEdge, e = edges(nodes), n = nodes.length;
        const grad = Array.from({ length: n }, () => [0, 0]);
        e.forEach((v, j) => {
            const L = Math.hypot(v[0], v[1]);
            const f = (this.kStretch / h) * ((L - h) / L);
            for (let d = 0; d < 2; d++) { grad[j][d] -= f * v[d]; grad[j + 1][d] += f * v[d]; }
        });
        for (let j = 0; j + 1 < e.length; j++) {
            const a = e[j], b = e[j + 1];
            const c = (this.kBend / h) * turning(a, b);
            const a2 = a[0] * a[0] + a[1] * a[1], b2 = b[0] * b[0] + b[1] * b[1];
            const pa = perp(a), pb = perp(b);
            const dka = [-pa[0] / a2, -pa[1] / a2], dkb = [pb[0] / b2, pb[1] / b2];
            for (let d = 0; d < 2; d++) {
                grad[j][d] -= c * dka[d];
                grad[j + 1][d] += c * (dka[d] - dkb[d]);
                grad[j + 2][d] += c * dkb[d];
            }
        }
        for (const g of grad) g[1] += this.gravity * h;
        return grad;
    }

    // d^2 Pi / dnodes^2 over all 2 n_nodes coordinates (dense).
    hessian(nodes) {
        const h = this.restEdge, n = this.nNodes, e = edges(nodes);
        const H = Array.from({ length: 2 * n }, () => new Float64Array(2 * n));
        // Stretch: one 2x2 block per edge, coupling its two nodes.
        e.forEach((v, j) => {
            const L = Math.hypot(v[0], v[1]);
            const t = [v[0] / L, v[1] / L];
            const Hs = [0, 1].map((r) => [0, 1].map((c) => (this.kStretch / h)
                * ((1 - h / L) * (r === c ? 1 : 0) + (h / L) * t[r] * t[c])));
            for (let r = 0; r < 2; r++) {
                for (let c = 0; c < 2; c++) {
                    H[2 * j + r][2 * j + c] += Hs[r][c];
                    H[2 * j + r][2 * j + 2 + c] -= Hs[r][c];
                    H[2 * j + 2 + r][2 * j + c] -= Hs[r][c];
                    H[2 * j + 2 + r][2 * j + 2 + c] += Hs[r][c];
                }
            }
        });
        // Bend: kappa_j depends on a = e_{j-1}, b = e_j, i.e., on three nodes.
        const angleHessian = (v, v2) => {         // d/dv [perp(v) / |v|^2], symmetric
            const pv = perp(v);
            return [[-2 * pv[0] * v[0] / (v2 * v2), -1 / v2 - 2 * pv[0] * v[1] / (v2 * v2)],
                [1 / v2 - 2 * pv[1] * v[0] / (v2 * v2), -2 * pv[1] * v[1] / (v2 * v2)]];
        };
        // B maps the six node coordinates to [a, b]: a = x1 - x0, b = x2 - x1.
        const B = [[-1, 0, 1, 0, 0, 0], [0, -1, 0, 1, 0, 0], [0, 0, -1, 0, 1, 0], [0, 0, 0, -1, 0, 1]];
        for (let j = 0; j + 1 < e.length; j++) {
            const a = e[j], b = e[j + 1];
            const kappa = turning(a, b);
            const a2 = a[0] * a[0] + a[1] * a[1], b2 = b[0] * b[0] + b[1] * b[1];
            const pa = perp(a), pb = perp(b);
            const g = [-pa[0] / a2, -pa[1] / a2, pb[0] / b2, pb[1] / b2];
            const Hab = g.map((x) => g.map((y) => x * y));
            const Ha = angleHessian(a, a2), Hb = angleHessian(b, b2);
            for (let r = 0; r < 2; r++) {
                for (let c = 0; c < 2; c++) {
                    Hab[r][c] -= kappa * Ha[r][c];
                    Hab[2 + r][2 + c] += kappa * Hb[r][c];
                }
            }
            const kb = this.kBend / h;
            for (let r = 0; r < 6; r++) {
                for (let c = 0; c < 6; c++) {
                    let s = 0;
                    for (let p = 0; p < 4; p++) {
                        if (B[p][r] === 0) continue;
                        for (let q = 0; q < 4; q++) if (B[q][c] !== 0) s += B[p][r] * Hab[p][q] * B[q][c];
                    }
                    H[2 * j + r][2 * j + c] += kb * s;
                }
            }
        }
        return H;
    }

    // ------------------------------------------------------------ quasistatic equilibrium
    // Indices of the free node coordinates in the flattened node array.
    get freeCoordinates() { return Array.from({ length: 2 * this.nNodes - 8 }, (_, i) => i + 4); }

    // Hessian Pi_xx with respect to the free coordinates x.
    freeHessian(nodes) {
        const H = this.hessian(nodes), free = this.freeCoordinates;
        return free.map((i) => free.map((j) => H[i][j]));
    }

    // Equilibrium nodes for grasp q. `initial` is the previous equilibrium when the
    // object moves continuously; without it, the solve starts from tangentCurveNodes.
    equilibrium(q, initial = null, tol = 1e-10, maxIterations = 200) {
        let nodes = initial === null || initial === undefined ? this.tangentCurveNodes(q) : initial.map((p) => [p[0], p[1]]);
        this._withClamps(nodes, this.clampedNodes(q));
        const free = this.freeCoordinates;
        for (let it = 0; it < maxIterations; it++) {
            const G = this.gradient(nodes);
            const g = free.map((i) => G[i >> 1][i & 1]);
            if (maxAbs(g) < tol) break;
            const step = descentStep(this.freeHessian(nodes), g);
            nodes = this._lineSearch(nodes, step, g);
        }
        return nodes;
    }

    // Backtracking on the energy, with each node moving at most maxMove.
    _lineSearch(nodes, step, g, maxMove = 0.5) {
        const free = this.freeCoordinates;
        const f = Math.min(1, maxMove / Math.max(maxAbs(step), 1e-12));
        step = step.map((x) => x * f);
        const energy0 = this.energy(nodes);
        let slope = 0;
        for (let i = 0; i < g.length; i++) slope += g[i] * step[i];
        let t = 1, trial = null;
        for (let k = 0; k < 40; k++) {
            trial = nodes.map((p) => [p[0], p[1]]);
            free.forEach((idx, i) => { trial[idx >> 1][idx & 1] += t * step[i]; });
            // The tolerance term lets the final Newton steps through when the
            // energy change is below floating-point resolution.
            if (this.energy(trial) <= energy0 + 1e-4 * t * slope + 1e-13 * Math.abs(energy0)) return trial;
            t *= 0.5;
        }
        return trial;
    }

    // ------------------------------------------------------------ perception: the tracked points
    // P tracked points, evenly spaced in arclength from 12% to 88% of the rod.
    static samplePoints(nodes, nPoints = 5) {
        const arc = arclength(nodes);
        const xs = nodes.map((p) => p[0]), ys = nodes.map((p) => p[1]);
        return linspace(0.12, 0.88, nPoints).map((f) => {
            const s = f * arc[arc.length - 1];
            return [interp(s, arc, xs), interp(s, arc, ys)];
        });
    }
}

// Newton step, shifted toward gradient descent where H is not positive definite.
function descentStep(H, g) {
    let shift = 0;
    const n = g.length;
    let scaleH = 1;
    for (let i = 0; i < n; i++) scaleH = Math.max(scaleH, Math.abs(H[i][i]));
    for (let k = 0; k < 30; k++) {
        const Hs = shift === 0 ? H : H.map((row, i) => row.map((x, j) => (i === j ? x + shift : x)));
        const L = cholesky(Hs);
        if (L) return choleskySolve(L, g).map((x) => -x);
        shift = Math.max(10 * shift, 1e-6 * scaleH);
    }
    return g.map((x) => -x / scaleH);
}

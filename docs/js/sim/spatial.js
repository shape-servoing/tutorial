// Small 3-D deformation models of the spatial shape-control demo
// (port of shape_servoing/spatial.py).
//
// The spatial rod uses a cubic centerline between two finite grasp regions. The
// sheet and the solid use the local-global as-rigid-as-possible (ARAP) algorithm
// with uniform edge weights and local positional grasp constraints; the solid is
// a volumetric lattice whose exterior is triangulated only for rendering. Each
// model exposes the map q -> x(q) -> s(q) used throughout the tutorial.
import { cholesky, choleskySolve, columnStack, det3, linspace, svd } from './linalg.js';

function uniqueEdges(faces) {
    const set = new Set();
    for (const f of faces) {
        for (const [a, b] of [[f[0], f[1]], [f[1], f[2]], [f[2], f[0]]]) set.add(a < b ? `${a},${b}` : `${b},${a}`);
    }
    return [...set].map((k) => k.split(',').map(Number)).sort((p, q) => p[0] - q[0] || p[1] - q[1]);
}

// Vertices within the given graph-edge radius of a grasp anchor.
export function topologicalBall(edges, seed, radius) {
    const adjacency = new Map();
    for (const [a, b] of edges) {
        if (!adjacency.has(a)) adjacency.set(a, new Set());
        if (!adjacency.has(b)) adjacency.set(b, new Set());
        adjacency.get(a).add(b);
        adjacency.get(b).add(a);
    }
    const reached = new Set([seed]);
    let frontier = new Set([seed]);
    for (let r = 0; r < Math.max(0, radius); r++) {
        const next = new Set();
        for (const i of frontier) for (const j of adjacency.get(i) || []) if (!reached.has(j)) next.add(j);
        next.forEach((j) => reached.add(j));
        frontier = next;
    }
    return [...reached].sort((a, b) => a - b);
}

function rodSpec(graspRadius) {
    const t = linspace(0, 1, 17);
    const rest = t.map((x) => [6.0 * (x - 0.5), 0, 0]);
    const edges = Array.from({ length: 16 }, (_, i) => [i, i + 1]);
    return {
        kind: 'rod', label: 'Spatial rod', deformation: 'grasped cubic centerline',
        rest, model_edges: edges, display_edges: edges, faces: [],
        fixed: topologicalBall(edges, 0, graspRadius), driven: topologicalBall(edges, 16, graspRadius),
        fixed_anchor: 0, driven_anchor: 16, grasp_radius: graspRadius,
        features: [2, 4, 6, 8, 10, 12, 14], q_target: [1.65, 1.35, 0.72],
        q_labels: ['right lift', 'right depth', 'tip angle'],
    };
}

function sheetSpec(graspRadius, nx = 7, ny = 5) {
    const xs = linspace(-3.0, 3.0, nx), ys = linspace(-1.7, 1.7, ny);
    const rest = [];
    for (const y of ys) for (const x of xs) rest.push([x, y, 0]);
    const faces = [];
    for (let j = 0; j < ny - 1; j++) {
        for (let i = 0; i < nx - 1; i++) {
            const a = j * nx + i, b = j * nx + i + 1, c = (j + 1) * nx + i, d = (j + 1) * nx + i + 1;
            if ((i + j) % 2 === 0) faces.push([a, b, d], [a, d, c]); else faces.push([a, b, c], [b, d, c]);
        }
    }
    const fixedAnchor = Math.floor(ny / 2) * nx, drivenAnchor = Math.floor(ny / 2) * nx + nx - 1;
    const features = [];
    for (const j of [0, 2, 4]) for (const i of [1, 3, 5]) features.push(j * nx + i);
    const edges = uniqueEdges(faces);
    return {
        kind: 'sheet', label: 'Planar sheet', deformation: 'triangular mesh · ARAP',
        rest, model_edges: edges, display_edges: edges, faces,
        fixed: topologicalBall(edges, fixedAnchor, graspRadius), driven: topologicalBall(edges, drivenAnchor, graspRadius),
        fixed_anchor: fixedAnchor, driven_anchor: drivenAnchor, grasp_radius: graspRadius,
        features, q_target: [-0.90, 0.75, 1.65], q_labels: ['right pull', 'right lift', 'edge twist'],
    };
}

function solidSpec(graspRadius, nx = 6, ny = 5, nz = 5) {
    const idx = (i, j, k) => k * nx * ny + j * nx + i;
    const xs = linspace(-2.4, 2.4, nx), ys = linspace(-1.3, 1.3, ny), zs = linspace(-1.1, 1.1, nz);
    const rest = [];
    for (const z of zs) for (const y of ys) for (const x of xs) rest.push([x, y, z]);
    // Tetrahedral-lattice connectivity: axes, face diagonals, and one body diagonal per cell.
    const offsets = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1], [1, 1, 1]];
    const modelEdges = [];
    for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
        for (const [di, dj, dk] of offsets) {
            if (i + di < nx && j + dj < ny && k + dk < nz) modelEdges.push([idx(i, j, k), idx(i + di, j + dj, k + dk)]);
        }
    }
    const faces = [];
    const quad = (a, b, c, d, flip = false) => {
        if (!flip) faces.push([a, b, d], [a, d, c]); else faces.push([a, d, b], [a, c, d]);
    };
    for (let k = 0; k < nz - 1; k++) for (let j = 0; j < ny - 1; j++) {
        quad(idx(0, j, k), idx(0, j + 1, k), idx(0, j, k + 1), idx(0, j + 1, k + 1), true);
        quad(idx(nx - 1, j, k), idx(nx - 1, j + 1, k), idx(nx - 1, j, k + 1), idx(nx - 1, j + 1, k + 1));
    }
    for (let k = 0; k < nz - 1; k++) for (let i = 0; i < nx - 1; i++) {
        quad(idx(i, 0, k), idx(i + 1, 0, k), idx(i, 0, k + 1), idx(i + 1, 0, k + 1));
        quad(idx(i, ny - 1, k), idx(i + 1, ny - 1, k), idx(i, ny - 1, k + 1), idx(i + 1, ny - 1, k + 1), true);
    }
    for (let j = 0; j < ny - 1; j++) for (let i = 0; i < nx - 1; i++) {
        quad(idx(i, j, 0), idx(i + 1, j, 0), idx(i, j + 1, 0), idx(i + 1, j + 1, 0), true);
        quad(idx(i, j, nz - 1), idx(i + 1, j, nz - 1), idx(i, j + 1, nz - 1), idx(i + 1, j + 1, nz - 1));
    }
    const fixedAnchor = idx(0, Math.floor(ny / 2), Math.floor(nz / 2));
    const drivenAnchor = idx(nx - 1, Math.floor(ny / 2), Math.floor(nz / 2));
    // Sample the visible boundary regularly, independently of the lattice refinement.
    const sampleI = [...new Set([1, Math.floor(nx / 2), nx - 2])].sort((a, b) => a - b);
    const features = [];
    for (let k = 0; k < nz; k += 2) for (let j = 0; j < ny; j += 2) for (const i of sampleI) {
        if (j === 0 || j === ny - 1 || k === 0 || k === nz - 1) features.push(idx(i, j, k));
    }
    return {
        kind: 'solid', label: 'Volumetric solid', deformation: 'volumetric lattice · ARAP',
        rest, model_edges: modelEdges, display_edges: uniqueEdges(faces), faces,
        fixed: topologicalBall(modelEdges, fixedAnchor, graspRadius), driven: topologicalBall(modelEdges, drivenAnchor, graspRadius),
        fixed_anchor: fixedAnchor, driven_anchor: drivenAnchor, grasp_radius: graspRadius,
        features, q_target: [-1.00, 1.00, 0.85], q_labels: ['compression', 'right lift', 'face twist'],
    };
}

export function spatialSpec(kind, graspRadius = 1) {
    if (kind === 'rod') return rodSpec(graspRadius);
    if (kind === 'sheet') return sheetSpec(graspRadius);
    if (kind === 'solid') return solidSpec(graspRadius);
    throw new Error(`unknown spatial object: ${kind}`);
}

// Equilibrium and feature maps for one object of the spatial demo.
export class SpatialModel {
    constructor(kind, graspRadius = 1) {
        this.spec = spatialSpec(kind, graspRadius);
        if (kind !== 'rod') this._prepareArap();
    }

    _prepareArap() {
        const n = this.spec.rest.length;
        const adjacency = Array.from({ length: n }, () => []);
        for (const [a, b] of this.spec.model_edges) { adjacency[a].push(b); adjacency[b].push(a); }
        this.adjacency = adjacency;
        const E = this.spec.model_edges;
        this.edgeI = [...E.map((e) => e[0]), ...E.map((e) => e[1])];
        this.edgeJ = [...E.map((e) => e[1]), ...E.map((e) => e[0])];
        const rest = this.spec.rest;
        this.edgeRest = this.edgeI.map((i, e) => [0, 1, 2].map((d) => rest[i][d] - rest[this.edgeJ[e]][d]));
        const constrained = [...new Set([...this.spec.fixed, ...this.spec.driven])].sort((a, b) => a - b);
        const isConstrained = new Set(constrained);
        this.constrained = constrained;
        this.free = Array.from({ length: n }, (_, i) => i).filter((i) => !isConstrained.has(i));
        const position = new Map(this.free.map((v, i) => [v, i]));
        const cpos = new Map(constrained.map((v, i) => [v, i]));
        // Graph Laplacian blocks: L_ff (factorized once) and L_fc (sparse rows).
        const nf = this.free.length;
        const Lff = Array.from({ length: nf }, () => new Array(nf).fill(0));
        this.Lfc = this.free.map(() => []);
        this.free.forEach((v, r) => {
            Lff[r][r] = adjacency[v].length;
            for (const w of adjacency[v]) {
                if (position.has(w)) Lff[r][position.get(w)] -= 1;
                else this.Lfc[r].push([cpos.get(w), -1]);
            }
        });
        this.LffChol = cholesky(Lff);
    }

    _transformed(q) {
        const p = this.spec.rest.map((v) => Array.from(v));
        if (this.spec.kind === 'sheet') {
            const ys = this.spec.rest.map((v) => v[1]);
            const y0 = Math.min(...ys), y1 = Math.max(...ys);
            p.forEach((v) => {
                const eta = (v[1] - y0) / (y1 - y0);
                v[0] += q[0];
                v[1] += q[1];
                v[2] += q[2] * (0.25 + 0.75 * eta);
            });
        } else if (this.spec.kind === 'solid') {
            const c = Math.cos(q[2]), s = Math.sin(q[2]);
            p.forEach((v) => {
                const y = v[1], z = v[2];
                v[0] += q[0];
                v[1] = c * y - s * z + q[1];
                v[2] = s * y + c * z;
            });
        }
        return p;
    }

    deform(q) {
        const spec = this.spec;
        if (spec.kind === 'rod') {
            // A cubic centerline joins two rigidly grasped regions. The right gripper
            // translates in y/z and rotates the local rod tangent.
            const p = spec.rest.map((v) => Array.from(v));
            const left = Math.max(...spec.fixed), right = Math.min(...spec.driven);
            const anchor = spec.rest[spec.driven_anchor];
            const endpoint = [anchor[0], anchor[1] + q[0], anchor[2] + q[1]];
            const direction = [Math.cos(q[2]), Math.sin(q[2]) / Math.SQRT2, Math.sin(q[2]) / Math.SQRT2];
            const spacing = spec.rest[1][0] - spec.rest[0][0];
            for (const i of spec.driven) {
                const steps = spec.driven_anchor - i;
                p[i] = endpoint.map((x, d) => x - steps * spacing * direction[d]);
            }
            for (const i of spec.fixed) p[i] = Array.from(spec.rest[i]);
            const p0 = p[left], p3 = p[right];
            const chord = Math.hypot(p3[0] - p0[0], p3[1] - p0[1], p3[2] - p0[2]);
            const c0 = [p0[0] + chord / 3, p0[1], p0[2]];
            const c1 = p3.map((x, d) => x - chord * direction[d] / 3);
            for (let i = left + 1; i < right; i++) {
                const u = (i - left) / (right - left);
                p[i] = [0, 1, 2].map((d) => (1 - u) ** 3 * p0[d] + 3 * (1 - u) ** 2 * u * c0[d]
                    + 3 * (1 - u) * u ** 2 * c1[d] + u ** 3 * p3[d]);
            }
            return p;
        }

        const rest = spec.rest;
        const transformed = this._transformed(q);
        const rx = rest.map((v) => v[0]);
        const xmin = Math.min(...rx), span = Math.max(...rx) - xmin;
        const vertices = rest.map((v, i) => {
            const blend = (v[0] - xmin) / span;
            return [0, 1, 2].map((d) => v[d] + blend * (transformed[i][d] - v[d]));
        });
        const fixedSet = new Set(spec.fixed);
        const constraints = this.constrained.map((i) => Array.from(fixedSet.has(i) ? rest[i] : transformed[i]));
        this.constrained.forEach((i, c) => { vertices[i] = Array.from(constraints[c]); });
        const n = rest.length, nE = this.edgeI.length;

        for (let it = 0; it < 7; it++) {
            // Local step: best rotation of every vertex neighbourhood.
            const covariance = Array.from({ length: n }, () => [[0, 0, 0], [0, 0, 0], [0, 0, 0]]);
            for (let e = 0; e < nE; e++) {
                const i = this.edgeI[e], j = this.edgeJ[e];
                const d = [vertices[i][0] - vertices[j][0], vertices[i][1] - vertices[j][1], vertices[i][2] - vertices[j][2]];
                const r = this.edgeRest[e], C = covariance[i];
                for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) C[a][b] += d[a] * r[b];
            }
            const rotations = covariance.map((C) => {
                const { U, V } = svd(C);
                let R = rotationOf(U, V);
                if (det3(R) < 0) {
                    const Uf = U.map((row) => [row[0], row[1], -row[2]]);
                    R = rotationOf(Uf, V);
                }
                return R;
            });
            // Global step: solve L_ff x_f = b_f - L_fc x_c for each coordinate.
            const b = Array.from({ length: n }, () => [0, 0, 0]);
            for (let e = 0; e < nE; e++) {
                const i = this.edgeI[e], j = this.edgeJ[e];
                const Ri = rotations[i], Rj = rotations[j], r = this.edgeRest[e];
                for (let a = 0; a < 3; a++) {
                    b[i][a] += 0.5 * ((Ri[a][0] + Rj[a][0]) * r[0] + (Ri[a][1] + Rj[a][1]) * r[1] + (Ri[a][2] + Rj[a][2]) * r[2]);
                }
            }
            for (let d = 0; d < 3; d++) {
                const rhs = this.free.map((v, row) => {
                    let s = b[v][d];
                    for (const [c, w] of this.Lfc[row]) s -= w * constraints[c][d];
                    return s;
                });
                const x = choleskySolve(this.LffChol, rhs);
                this.free.forEach((v, row) => { vertices[v][d] = x[row]; });
            }
            this.constrained.forEach((i, c) => { vertices[i] = Array.from(constraints[c]); });
        }
        return vertices;
    }

    features(vertices) { return this.spec.features.flatMap((i) => [vertices[i][0], vertices[i][1], vertices[i][2]]); }

    jacobian(q, eps = 1e-3) {
        const columns = q.map((_, j) => {
            const qp = Array.from(q), qm = Array.from(q);
            qp[j] += eps;
            qm[j] -= eps;
            const fp = this.features(this.deform(qp)), fm = this.features(this.deform(qm));
            return fp.map((x, i) => (x - fm[i]) / (2 * eps));
        });
        return columnStack(columns);
    }
}

// U V^T for 3 x 3 factors.
function rotationOf(U, V) {
    const R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) {
        R[a][b] = U[a][0] * V[b][0] + U[a][1] * V[b][1] + U[a][2] * V[b][2];
    }
    return R;
}

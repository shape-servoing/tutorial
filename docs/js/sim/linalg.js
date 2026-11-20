// Dense linear algebra for the small matrices of the tutorial: the subset of
// numpy.linalg that shape_servoing uses. Vectors are arrays of numbers and
// matrices are arrays of rows. The singular value decomposition uses one-sided
// Jacobi rotations and the symmetric eigendecomposition cyclic Jacobi
// rotations: both are accurate for matrices of this size (at most 150 x 150).

export const EPS = 2.220446049250313e-16;

// ------------------------------------------------------------------ construction
export const zeros = (n) => new Array(n).fill(0);
export const zerosM = (rows, cols) => Array.from({ length: rows }, () => new Array(cols).fill(0));
export const eye = (n) => Array.from({ length: n }, (_, i) => { const r = new Array(n).fill(0); r[i] = 1; return r; });
export const copyV = (v) => Array.from(v);
export const copyM = (A) => A.map((row) => Array.from(row));
export const linspace = (a, b, n) => (n === 1 ? [a] : Array.from({ length: n }, (_, i) => a + (b - a) * i / (n - 1)));
export const geomspace = (a, b, n) => linspace(Math.log(a), Math.log(b), n).map(Math.exp);

// ------------------------------------------------------------------ vectors
export const dot = (a, b) => { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; };
export const norm = (a) => Math.sqrt(dot(a, a));
export const add = (a, b) => a.map((x, i) => x + b[i]);
export const sub = (a, b) => a.map((x, i) => x - b[i]);
export const scale = (a, s) => a.map((x) => x * s);
export const maxAbs = (a) => a.reduce((m, x) => Math.max(m, Math.abs(x)), 0);
export const sum = (a) => a.reduce((s, x) => s + x, 0);
export const mean = (a) => sum(a) / a.length;
export const prod = (a) => a.reduce((p, x) => p * x, 1);

// ------------------------------------------------------------------ matrices
export const rows = (A) => A.length;
export const cols = (A) => (A.length ? A[0].length : 0);
export const transpose = (A) => Array.from({ length: cols(A) }, (_, j) => A.map((row) => row[j]));
export const column = (A, j) => A.map((row) => row[j]);
// Matrix whose columns are the given vectors (numpy.column_stack).
export const columnStack = (vectors) => Array.from({ length: vectors[0].length }, (_, i) => vectors.map((v) => v[i]));
export const addM = (A, B) => A.map((row, i) => row.map((x, j) => x + B[i][j]));
export const subM = (A, B) => A.map((row, i) => row.map((x, j) => x - B[i][j]));
export const scaleM = (A, s) => A.map((row) => row.map((x) => x * s));
export const normF = (A) => Math.sqrt(A.reduce((s, row) => s + dot(row, row), 0));
export const outer = (a, b) => a.map((x) => b.map((y) => x * y));
export const flatten = (A) => [].concat(...A);
export const reshape = (v, cols_) => Array.from({ length: v.length / cols_ }, (_, i) => v.slice(i * cols_, (i + 1) * cols_));

export function matmul(A, B) {
    const n = A.length, k = B.length, m = k ? B[0].length : 0;
    const C = zerosM(n, m);
    for (let i = 0; i < n; i++) {
        const a = A[i], c = C[i];
        for (let l = 0; l < k; l++) {
            const x = a[l];
            if (x === 0) continue;
            const b = B[l];
            for (let j = 0; j < m; j++) c[j] += x * b[j];
        }
    }
    return C;
}

export const matvec = (A, x) => A.map((row) => dot(row, x));
export const vecmat = (x, A) => { const out = zeros(cols(A)); A.forEach((row, i) => { for (let j = 0; j < row.length; j++) out[j] += x[i] * row[j]; }); return out; };
export const diag = (v) => v.map((x, i) => { const r = new Array(v.length).fill(0); r[i] = x; return r; });

// ------------------------------------------------------------------ factorizations and solves
// Cholesky factor L (A = L L^T), or null if A is not positive definite.
export function cholesky(A) {
    const n = A.length;
    const L = zerosM(n, n);
    for (let j = 0; j < n; j++) {
        let d = A[j][j];
        const lj = L[j];
        for (let k = 0; k < j; k++) d -= lj[k] * lj[k];
        if (!(d > 0)) return null;
        const djj = Math.sqrt(d);
        lj[j] = djj;
        for (let i = j + 1; i < n; i++) {
            const li = L[i];
            let s = A[i][j];
            for (let k = 0; k < j; k++) s -= li[k] * lj[k];
            li[j] = s / djj;
        }
    }
    return L;
}

// Solve L L^T x = b.
export function choleskySolve(L, b) {
    const n = L.length, y = new Array(n);
    for (let i = 0; i < n; i++) {
        let s = b[i];
        const li = L[i];
        for (let k = 0; k < i; k++) s -= li[k] * y[k];
        y[i] = s / li[i];
    }
    const x = new Array(n);
    for (let i = n - 1; i >= 0; i--) {
        let s = y[i];
        for (let k = i + 1; k < n; k++) s -= L[k][i] * x[k];
        x[i] = s / L[i][i];
    }
    return x;
}

// Gauss-Jordan elimination with partial pivoting: A^{-1}.
export function inv(A) {
    const n = A.length;
    const M = A.map((row, i) => [...row, ...eye(n)[i]]);
    for (let c = 0; c < n; c++) {
        let p = c;
        for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
        if (M[p][c] === 0) throw new Error('singular matrix');
        [M[c], M[p]] = [M[p], M[c]];
        const pivot = M[c][c];
        for (let j = 0; j < 2 * n; j++) M[c][j] /= pivot;
        for (let r = 0; r < n; r++) {
            if (r === c) continue;
            const f = M[r][c];
            if (f === 0) continue;
            for (let j = 0; j < 2 * n; j++) M[r][j] -= f * M[c][j];
        }
    }
    return M.map((row) => row.slice(n));
}

// Solve A x = b for a general square A (vector b), through LU with partial pivoting.
export function solve(A, b) {
    const n = A.length;
    const M = A.map((row, i) => [...row, b[i]]);
    for (let c = 0; c < n; c++) {
        let p = c;
        for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
        if (M[p][c] === 0) throw new Error('singular matrix');
        [M[c], M[p]] = [M[p], M[c]];
        for (let r = c + 1; r < n; r++) {
            const f = M[r][c] / M[c][c];
            if (f === 0) continue;
            for (let j = c; j <= n; j++) M[r][j] -= f * M[c][j];
        }
    }
    const x = new Array(n);
    for (let i = n - 1; i >= 0; i--) {
        let s = M[i][n];
        for (let j = i + 1; j < n; j++) s -= M[i][j] * x[j];
        x[i] = s / M[i][i];
    }
    return x;
}

// ------------------------------------------------------------------ singular value decomposition
// A (n x m) = U diag(s) V^T with s descending: U is n x min(n, m), and V is the
// full m x m orthogonal matrix (its last columns span Null(A)), as numpy.linalg.svd
// with full_matrices=False for U and s, and full_matrices=True for V.
export function svd(A) {
    const n = A.length, m = cols(A);
    const W = Array.from({ length: m }, (_, j) => Float64Array.from(A, (row) => row[j]));
    const V = Array.from({ length: m }, (_, j) => { const v = new Float64Array(m); v[j] = 1; return v; });
    for (let sweep = 0; sweep < 80; sweep++) {
        let rotated = false;
        for (let i = 0; i < m - 1; i++) {
            for (let j = i + 1; j < m; j++) {
                const wi = W[i], wj = W[j];
                let a = 0, b = 0, g = 0;
                for (let r = 0; r < n; r++) { a += wi[r] * wi[r]; b += wj[r] * wj[r]; g += wi[r] * wj[r]; }
                if (g === 0 || Math.abs(g) <= EPS * Math.sqrt(a * b)) continue;
                rotated = true;
                const zeta = (b - a) / (2 * g);
                const t = (zeta >= 0 ? 1 : -1) / (Math.abs(zeta) + Math.sqrt(1 + zeta * zeta));
                const c = 1 / Math.sqrt(1 + t * t), s = c * t;
                for (let r = 0; r < n; r++) { const x = wi[r], y = wj[r]; wi[r] = c * x - s * y; wj[r] = s * x + c * y; }
                const vi = V[i], vj = V[j];
                for (let r = 0; r < m; r++) { const x = vi[r], y = vj[r]; vi[r] = c * x - s * y; vj[r] = s * x + c * y; }
            }
        }
        if (!rotated) break;
    }
    const sigma = W.map((w) => Math.sqrt(w.reduce((acc, x) => acc + x * x, 0)));
    const order = sigma.map((_, j) => j).sort((p, q) => sigma[q] - sigma[p]);
    const k = Math.min(n, m);
    const s = order.slice(0, k).map((j) => sigma[j]);
    const tiny = Math.max(n, m) * EPS * (s[0] || 0);
    const Ucols = [];
    for (let l = 0; l < k; l++) {
        const j = order[l];
        if (s[l] > tiny && s[l] > 0) Ucols.push(Array.from(W[j], (x) => x / s[l]));
        else Ucols.push(null);
    }
    // Columns of U for zero singular values: complete to an orthonormal set.
    for (let l = 0; l < k; l++) {
        if (Ucols[l]) continue;
        let best = null;
        for (let e = 0; e < n && !best; e++) {
            let v = zeros(n); v[e] = 1;
            for (let pass = 0; pass < 2; pass++) {
                for (const u of Ucols) if (u) v = sub(v, scale(u, dot(u, v)));
            }
            const nv = norm(v);
            if (nv > 1e-6) best = scale(v, 1 / nv);
        }
        Ucols[l] = best;
    }
    if (n === 3 && k === 3 && s[2] <= tiny) {
        // 3 x 3 of rank 2: the third left vector is fixed (up to sign) by the first two.
        const [u1, u2] = Ucols;
        Ucols[2] = [u1[1] * u2[2] - u1[2] * u2[1], u1[2] * u2[0] - u1[0] * u2[2], u1[0] * u2[1] - u1[1] * u2[0]];
    }
    return { U: columnStack(Ucols), s, V: columnStack(order.map((j) => Array.from(V[j]))) };
}

export const svdvals = (A) => svd(A).s;

// Moore-Penrose pseudoinverse, singular values below rcond * sigma_max dropped (numpy default 1e-15).
export function pinv(A, rcond = 1e-15) {
    const n = A.length, m = cols(A);
    const { U, s, V } = svd(A);
    const P = zerosM(m, n);
    const cut = rcond * (s[0] || 0);
    for (let l = 0; l < s.length; l++) {
        if (!(s[l] > cut)) continue;
        const inv_s = 1 / s[l];
        for (let i = 0; i < m; i++) {
            const vi = V[i][l] * inv_s;
            if (vi === 0) continue;
            const row = P[i];
            for (let j = 0; j < n; j++) row[j] += vi * U[j][l];
        }
    }
    return P;
}

// ------------------------------------------------------------------ symmetric eigenproblem
// Eigenvalues (ascending) and eigenvectors (columns) of a symmetric matrix.
export function eigh(Ain, { vectors = true } = {}) {
    const n = Ain.length;
    const A = Ain.map((row) => Float64Array.from(row));
    const V = vectors ? Array.from({ length: n }, (_, i) => { const r = new Float64Array(n); r[i] = 1; return r; }) : null;
    let scaleA = 0;
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) scaleA += A[i][j] * A[i][j];
    for (let sweep = 0; sweep < 100; sweep++) {
        let off = 0;
        for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
        if (off <= 1e-34 * scaleA || off === 0) break;
        for (let p = 0; p < n - 1; p++) {
            for (let q = p + 1; q < n; q++) {
                const apq = A[p][q];
                if (apq === 0) continue;
                const theta = (A[q][q] - A[p][p]) / (2 * apq);
                const t = (theta >= 0 ? 1 : -1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
                const c = 1 / Math.sqrt(t * t + 1), s = t * c;
                for (let k = 0; k < n; k++) {               // columns p, q
                    const akp = A[k][p], akq = A[k][q];
                    A[k][p] = c * akp - s * akq;
                    A[k][q] = s * akp + c * akq;
                }
                for (let k = 0; k < n; k++) {               // rows p, q
                    const apk = A[p][k], aqk = A[q][k];
                    A[p][k] = c * apk - s * aqk;
                    A[q][k] = s * apk + c * aqk;
                }
                if (V) {
                    for (let k = 0; k < n; k++) {
                        const vkp = V[k][p], vkq = V[k][q];
                        V[k][p] = c * vkp - s * vkq;
                        V[k][q] = s * vkp + c * vkq;
                    }
                }
            }
        }
    }
    const values = Array.from({ length: n }, (_, i) => A[i][i]);
    const order = values.map((_, i) => i).sort((a, b) => values[a] - values[b]);
    return {
        values: order.map((i) => values[i]),
        vectors: V ? V.map((row) => order.map((i) => row[i])) : null,
    };
}

export const eigvalsh = (A) => eigh(A, { vectors: false }).values;

export const det3 = (M) => M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1])
    - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0])
    + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

// ------------------------------------------------------------------ numpy helpers
// numpy.interp for increasing xp: linear interpolation, clamped at the ends.
export function interp(x, xp, fp) {
    const n = xp.length;
    if (x <= xp[0]) return fp[0];
    if (x >= xp[n - 1]) return fp[n - 1];
    let lo = 0, hi = n - 1;
    while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (xp[mid] <= x) lo = mid; else hi = mid; }
    const dx = xp[hi] - xp[lo];
    if (dx === 0) return fp[hi];
    return fp[lo] + (fp[hi] - fp[lo]) * (x - xp[lo]) / dx;
}

// numpy.histogram counts for the given bin edges (the last bin includes its right edge).
export function histogram(values, edges) {
    const counts = new Array(edges.length - 1).fill(0);
    const last = edges.length - 1;
    for (const v of values) {
        if (!(v >= edges[0] && v <= edges[last])) continue;
        let lo = 0, hi = last;
        while (hi - lo > 1) { const mid = (lo + hi) >> 1; if (edges[mid] <= v) lo = mid; else hi = mid; }
        counts[v === edges[last] ? last - 1 : lo] += 1;
    }
    return counts;
}

export const cumsumFrom0 = (a) => { const out = [0]; for (const x of a) out.push(out[out.length - 1] + x); return out; };

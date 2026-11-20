// Subspaces, singular directions, and local stability of the Jacobian
// (port of shape_servoing/analysis.py).
//
//     J = Phi Sigma Psi^T = sum_l sigma_l phi_l psi_l^T     (singular value decomposition)
//     w = prod_l sigma_l,   cond = sigma_max / sigma_min    (responsiveness, anisotropy)
//     dL/dt = -alpha e^T J J_hat^+ e                        (Lyapunov rate, L = ||e||^2 / 2)
import { cols, column, columnStack, dot, eigvalsh, eye, inv, matmul, matvec, norm, pinv, prod,
    sub, subM, svd, transpose } from './linalg.js';

// Gains sigma_l (descending), feature directions phi_l (columns of Phi), and
// robot directions psi_l (columns of Psi): [sigma, Phi, Psi]. The sign of each
// pair (phi_l, psi_l) is a free choice of the SVD; here, the largest entry of
// phi_l is positive (numpy leaves the sign to LAPACK).
export function singularTriplets(J) {
    const { U, s, V } = svd(J);
    const sign = s.map((_, l) => {
        let best = 0;
        for (let i = 0; i < U.length; i++) if (Math.abs(U[i][l]) > Math.abs(best) + 1e-12) best = U[i][l];
        return best < 0 ? -1 : 1;
    });
    return [s, U.map((row) => row.map((x, l) => sign[l] * x)), V.map((row) => s.map((_, l) => sign[l] * row[l]))];
}

export const manipulability = (J) => prod(svd(J).s);

export function conditionNumber(J) {
    const s = svd(J).s;
    return s[s.length - 1] > 0 ? s[0] / s[s.length - 1] : Infinity;
}

export function numericalRank(J, rtol = 1e-3) {
    const s = svd(J).s;
    return s.filter((x) => x > rtol * s[0]).length;
}

// Orthonormal basis (columns) of the robot velocities that J maps to ~zero.
export function nullSpace(J, rtol = 1e-3) {
    const { s, V } = svd(J);
    const rank = s.filter((x) => x > rtol * s[0]).length;
    return V.map((row) => row.slice(rank));
}

// Split e into J J^+ e, its component in Range(J), and the orthogonal component.
export function rangeSplit(J, e) {
    const eRange = matvec(J, matvec(pinv(J), e));
    return [eRange, sub(e, eRange)];
}

export const nullSpaceProjector = (J) => subM(eye(cols(J)), matmul(pinv(J), J));

// Damped least-squares inverse J^T (J J^T + mu^2 I)^{-1}.
export function dampedInverse(J, mu) {
    const A = matmul(J, transpose(J)).map((row, i) => row.map((x, j) => (i === j ? x + mu * mu : x)));
    return matmul(transpose(J), inv(A));
}

// W_s J W_q^{-1} for diagonal scalings of the features and robot coordinates.
export const scaled = (J, featureScale, robotScale) => J.map((row, i) => row.map((x, j) => featureScale[i] * x / robotScale[j]));

// dL/dt = -alpha e^T J J_hat^+ e for L = ||e||^2 / 2.
export const lyapunovRate = (J, J_hat, e, alpha) => -alpha * dot(e, matvec(J, matvec(pinv(J_hat), e)));

// Smallest restricted eigenvalue in the paper's stability condition: for n <= m,
// Sym(J J_hat^+) on Range(J); for n > m, Sym(J_hat^+ J) on Range(J^T).
// Returns [smallest eigenvalue, all eigenvalues].
export function descentMargin(J, J_hat, rtol = 1e-6) {
    const [sigma, Phi, Psi] = singularTriplets(J);
    const keep = sigma.map((x) => x > rtol * sigma[0]);
    let basis, M;
    if (J.length <= cols(J)) {
        basis = columnStack(sigma.map((_, l) => column(Phi, l)).filter((_, l) => keep[l]));
        M = matmul(J, pinv(J_hat));
    } else {
        basis = columnStack(sigma.map((_, l) => column(Psi, l)).filter((_, l) => keep[l]));
        M = matmul(pinv(J_hat), J);
    }
    const sym = M.map((row, i) => row.map((x, j) => 0.5 * (x + M[j][i])));
    const reduced = matmul(transpose(basis), matmul(sym, basis));
    const values = eigvalsh(reduced.map((row, i) => row.map((x, j) => 0.5 * (x + reduced[j][i]))));
    return [values[0], values];
}

// Angle [rad] between each error e (columns) and the feature velocity J J_hat^+ e.
export function descentAngles(J, J_hat, errors) {
    const velocity = matmul(J, matmul(pinv(J_hat), errors));
    const out = [];
    for (let c = 0; c < errors[0].length; c++) {
        const e = column(errors, c), v = column(velocity, c);
        const cosine = dot(e, v) / (norm(e) * norm(v) + 1e-300);
        out.push(Math.acos(Math.min(1, Math.max(-1, cosine))));
    }
    return out;
}

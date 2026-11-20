// The deformation Jacobian J = ds/dq and its estimation (port of shape_servoing/jacobian.py).
//
//     numericalJacobian  J(q) by central differences of any map q -> s (the exact
//                        Jacobian of the simulated rod, unavailable on a real system)
//     offlineEstimate    regression over exploratory motions, J0 = dS dQ^+
//     broydenUpdate      online secant correction from the latest pair (dq, ds)
import { columnStack, matmul, normF, pinv, subM, zeros } from './linalg.js';

// Column j is the feature velocity produced by a unit velocity of q_j.
export function numericalJacobian(measure, q, step = 1e-3) {
    const columns = q.map((_, j) => {
        const dq = zeros(q.length);
        dq[j] = step;
        const plus = measure(q.map((x, i) => x + dq[i]));
        const minus = measure(q.map((x, i) => x - dq[i]));
        return plus.map((x, i) => (x - minus[i]) / (2 * step));
    });
    return columnStack(columns);
}

// Least-squares Jacobian from the exploratory actions dQ (m x C).
// Returns [J0, dS (n x C), residual ||dS - J0 dQ||_F].
export function offlineEstimate(measure, q0, dQ) {
    const s0 = measure(q0);
    const C = dQ[0].length;
    const responses = [];
    for (let c = 0; c < C; c++) {
        const s = measure(q0.map((x, i) => x + dQ[i][c]));
        responses.push(s.map((x, i) => x - s0[i]));
    }
    const dS = columnStack(responses);
    const J0 = matmul(dS, pinv(dQ));
    return [J0, dS, normF(subM(dS, matmul(J0, dQ)))];
}

// Rank-one correction enforcing the secant relation J dq = ds. Returns the updated
// estimate and the prediction error r = ds - J_hat dq. `relaxation` < 1 applies
// only part of the correction (1 is Broyden's rule as in the tutorial).
export function broydenUpdate(J, dq, ds, relaxation = 1.0) {
    const r = ds.map((x, i) => x - J[i].reduce((acc, v, j) => acc + v * dq[j], 0));
    const dq2 = dq.reduce((acc, x) => acc + x * x, 0);
    const J1 = J.map((row, i) => row.map((v, j) => v + relaxation * r[i] * dq[j] / dq2));
    return [J1, r];
}

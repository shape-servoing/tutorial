// Three ways the loop fails (port of shape_servoing/failures.py).
//
//     loss of controllability   dx/dq loses rank: sigma_min -> 0, w -> 0
//     loss of observability     ds/dx does not register a motion of the object
//     mechanical instability    lambda_min(Pi_xx) -> 0 (buckling)
import { conditionNumber, manipulability, singularTriplets } from './analysis.js';
import { numericalJacobian } from './jacobian.js';
import { eigvalsh, interp, norm } from './linalg.js';

// Horizontal grasp with the grippers `separation` apart and tangents aligned.
export const stretchedGrasp = (separation, height = 1.0) => [-separation / 2, height, 0.0, separation / 2, height, 0.0];

// Exact Jacobian diagnostics as the grippers pull the rod apart.
export function tautSweep(plant, separations) {
    const out = { sigma_min: [], manipulability: [], condition: [], wrist_share: [] };
    for (const separation of separations) {
        const q = stretchedGrasp(separation);
        plant.moveTo(q);
        const J = numericalJacobian((x) => plant.peek(x), q);
        const [sigma, , Psi] = singularTriplets(J);
        const last = sigma.length - 1;
        out.sigma_min.push(sigma[last]);
        out.manipulability.push(manipulability(J));
        out.condition.push(conditionNumber(J));
        out.wrist_share.push(Psi[2][last] ** 2 + Psi[5][last] ** 2);
    }
    return out;
}

// Smallest eigenvalue of the energy Hessian Pi_xx on the straight rod.
export function straightRodStiffness(rod, separation, height = 1.0) {
    const nodes = rod.straightNodes(stretchedGrasp(separation, height));
    return eigvalsh(rod.freeHessian(nodes))[0];
}

// Separation at which lambda_min(Pi_xx) of the straight rod crosses zero.
export function bucklingOnset(rod, separations) {
    const lam = separations.map((d) => straightRodStiffness(rod, d));
    const order = lam.map((_, i) => i).sort((a, b) => lam[a] - lam[b]);
    return interp(0, order.map((i) => lam[i]), order.map((i) => separations[i]));
}

// ||ds|| / ||dq||: near zero for a lost direction, large at an instability.
export const responseRatio = (dq, ds) => norm(ds) / Math.max(norm(dq), 1e-12);

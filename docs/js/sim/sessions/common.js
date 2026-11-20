// Shared pieces of the demo sessions (port of demos/sessions/common.py).
//
// A session is the state of one open demo page: its act_<name> methods are the
// actions the page can request, each with one object of named arguments, and
// `state()` describes what the page draws. The helpers keep the robot inside a
// safe workspace and package scenes for drawing.
import { norm } from '../linalg.js';
import { Q_ARCH, targetFromGrasp } from '../running_example.js';

// Workspace of the grippers: x and y in simulation units, theta in radians.
export const Q_LOWER = [-4.2, -1.2, -1.45, 0.6, -1.2, -1.45];
export const Q_UPPER = [-0.6, 3.8, 1.45, 4.2, 3.8, 1.45];
export const MIN_SEPARATION = 1.2, MAX_SEPARATION = 7.2;      // at most 20 % stretch

export class Session {
    dispatch(action, payload = {}) {
        const method = this[`act_${action}`];
        if (typeof method !== 'function') throw new Error(`unknown action: ${action}`);
        return method.call(this, payload || {});
    }

    state() { throw new Error('not implemented'); }
}

const clip = (x, lo, hi) => Math.min(hi, Math.max(lo, x));

// Clip a grasp to the workspace and to the allowed gripper separation.
export function admissible(q) {
    q = q.map((x, i) => clip(Number(x), Q_LOWER[i], Q_UPPER[i]));
    const left = [q[0], q[1]], right = [q[3], q[4]];
    const gap = norm([right[0] - left[0], right[1] - left[1]]);
    if (gap > MAX_SEPARATION || gap < MIN_SEPARATION) {
        const middle = [0.5 * (left[0] + right[0]), 0.5 * (left[1] + right[1])];
        const unit = [(right[0] - left[0]) / Math.max(gap, 1e-9), (right[1] - left[1]) / Math.max(gap, 1e-9)];
        const half = 0.5 * clip(gap, MIN_SEPARATION, MAX_SEPARATION);
        q[0] = middle[0] - half * unit[0]; q[1] = middle[1] - half * unit[1];
        q[3] = middle[0] + half * unit[0]; q[4] = middle[1] + half * unit[1];
    }
    return q;
}

// What the scene canvas draws for the current object.
export const scene = (plant) => ({ q: plant.q, nodes: plant.nodes, points: plant.points });

export const targetScene = (nodes, points) => ({ nodes, points });

// A feasible target recorded at a random grasp, away from the current shape.
// Returns [q, nodes, points, s*].
export function randomTarget(rng, plant, minDistance = 1.2) {
    for (let attempt = 0; attempt < 200; attempt++) {
        const separation = rng.uniform(3.4, 5.6);
        const middle = [rng.uniform(-0.7, 0.7), rng.uniform(0.4, 2.0)];
        const tilt = rng.uniform(-0.25, 0.25);
        const axis = [Math.cos(tilt), Math.sin(tilt)];
        const left = [middle[0] - 0.5 * separation * axis[0], middle[1] - 0.5 * separation * axis[1]];
        const right = [middle[0] + 0.5 * separation * axis[0], middle[1] + 0.5 * separation * axis[1]];
        const bend = rng.uniform(-1.2, 1.3);
        const q = admissible([...left, tilt + bend + rng.uniform(-0.3, 0.3), ...right, tilt - bend + rng.uniform(-0.3, 0.3)]);
        const [nodes, points, sStar] = targetFromGrasp(q, plant.rod, plant.featureMap, plant.nPoints);
        const current = plant.points;
        let d2 = 0;
        points.forEach((p, i) => { d2 += (p[0] - current[i][0]) ** 2 + (p[1] - current[i][1]) ** 2; });
        if (Math.sqrt(d2) >= minDistance) return [q, nodes, points, sStar];
    }
    return [Array.from(Q_ARCH), ...targetFromGrasp(Q_ARCH, plant.rod, plant.featureMap, plant.nPoints)];
}

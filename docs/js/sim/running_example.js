// The running example: two robot arms deform an elastic rod seen by a camera
// (port of shape_servoing/running_example.py).
//
// q = [x_L, y_L, theta_L, x_R, y_R, theta_R] (m = 6); perception tracks P = 5
// material points (n = 10 for point coordinates). Lengths in simulation units
// (rest length 6), angles in radians, control period 1. `Plant` groups the
// robot, the rod, the camera, and the feature map: command q, measure s.
import { featureVector } from './features.js';
import { linspace } from './linalg.js';
import { Rod } from './rod.js';

export const DOF_NAMES = ['x_L', 'y_L', 'theta_L', 'x_R', 'y_R', 'theta_R'];
export const N_POINTS = 5;

// Grasps used throughout the tutorial figures.
export const Q_NOMINAL = [-3.0, 1.00, 0.00, 3.0, 1.00, 0.00];     // initial configuration q_0
export const Q_ARCH = [-1.7, 1.15, 1.25, 1.7, 1.15, -1.25];       // the arch target
export const Q_REACHABLE = [-2.6, 1.25, 0.55, 2.6, 1.15, -0.55];  // feasible target
// Synthetic displacements beyond the rod's admissible deformation limits.
export const UNFEASIBLE_SHIFTS = { 1: [0.0, 0.6375], 2: [0.0, -0.525], 3: [0.0, 0.60] };

// Offline exploration: four signed perturbations of each gripper coordinate.
export const PROBE_AMPLITUDES = [0.35, 0.35, 0.18, 0.35, 0.35, 0.18];
export const PROBE_FRACTIONS = [-1.0, -0.5, 0.5, 1.0];

// Exploratory actions Delta Q (m x C): one coordinate perturbed at a time.
export function probeMotions(amplitudes = PROBE_AMPLITUDES, fractions = PROBE_FRACTIONS) {
    const columns = [];
    amplitudes.forEach((a, j) => fractions.forEach((f) => {
        const c = new Array(amplitudes.length).fill(0);
        c[j] = f * a;
        columns.push(c);
    }));
    return amplitudes.map((_, i) => columns.map((c) => c[i]));
}

// Robot + rod + camera + feature map: command q, measure s. Each new equilibrium
// is computed from the previous one; `peek` returns the features that would be
// measured at q without moving the robot.
export class Plant {
    constructor({ rod = null, featureMap = 'points', nPoints = N_POINTS, q = Q_NOMINAL } = {}) {
        this.rod = rod || new Rod();
        this.featureMap = featureMap;
        this.nPoints = nPoints;
        this.reset(q);
    }

    // Start an independent experiment at q, without a previous equilibrium.
    reset(q) {
        this.q = Array.from(q);
        this.nodes = this.rod.equilibrium(this.q);
        return this.s;
    }

    // Move the grippers to q, let the rod settle, and measure s.
    moveTo(q) {
        this.q = Array.from(q);
        this.nodes = this.rod.equilibrium(this.q, this.nodes);
        return this.s;
    }

    peek(q) { return this.features(this.rod.equilibrium(q, this.nodes)); }
    pointsOf(nodes) { return Rod.samplePoints(nodes, this.nPoints); }
    features(nodes) { return featureVector(this.pointsOf(nodes), this.featureMap); }
    get points() { return this.pointsOf(this.nodes); }
    get s() { return this.features(this.nodes); }
}

// Target recorded by demonstration: the equilibrium at a grasp q*.
// Returns [nodes, points, s*].
export function targetFromGrasp(qStar, rod = null, featureMap = 'points', nPoints = N_POINTS) {
    const plant = new Plant({ rod, featureMap, nPoints, q: qStar });
    return [plant.nodes, plant.points, plant.s];
}

// A synthetic target that exceeds the rod's deformation limits.
export function unfeasiblePoints(points, shifts = UNFEASIBLE_SHIFTS) {
    const out = points.map((p) => [p[0], p[1]]);
    for (const [i, shift] of Object.entries(shifts)) {
        out[Number(i)][0] += shift[0];
        out[Number(i)][1] += shift[1];
    }
    return out;
}

// ------------------------------------------------------------------ the looped rod of Section VI
// Bringing the grippers close together and turning the wrists by -pi and +pi winds
// the rod into a hanging ring: the tangent turns by 2 pi along the rod.
export const Q_RING = [-0.4, 2.5, 0.0, 0.4, 2.5, 0.0];        // grasp before winding
export const Q_STRAIGHT = [-2.7, 2.5, 0.0, 2.7, 2.5, 0.0];    // the same rod, unlooped

// Plant holding the looped rod, wound continuously from Q_RING.
export function ringPlant(featureMap = 'points', nPoints = N_POINTS, steps = 200) {
    const plant = new Plant({ featureMap, nPoints, q: Q_RING });
    for (const t of linspace(0, 1, steps + 1).slice(1)) {
        const q = Array.from(Q_RING);
        q[2] -= t * Math.PI;
        q[5] += t * Math.PI;
        plant.moveTo(q);
    }
    return plant;
}

// The grasp q moved rigidly: rotated by `angle` about `center` (default: the
// midpoint of the grippers), then translated by `shift`.
export function rigidGrasp(q, angle = 0, shift = [0, 0], center = null) {
    const out = Array.from(q);
    const c0 = center || [0.5 * (q[0] + q[3]), 0.5 * (q[1] + q[4])];
    const c = Math.cos(angle), s = Math.sin(angle);
    for (const i of [0, 3]) {
        const x = q[i] - c0[0], y = q[i + 1] - c0[1];
        out[i] = c * x - s * y + c0[0] + shift[0];
        out[i + 1] = s * x + c * y + c0[1] + shift[1];
        out[i + 2] = q[i + 2] + angle;
    }
    return out;
}

// Target of the combined example: the unlooped rod at another pose.
export const Q_STRAIGHT_MOVED = rigidGrasp(Q_STRAIGHT, 20 * Math.PI / 180, [1.2, 0.6]);

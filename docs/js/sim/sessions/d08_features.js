// Demo 09 - Choosing the shape features (port of demos/sessions/d08_features.py).
//
// Point coordinates, edge vectors, and curvature-based features (turning angles)
// computed from the same tracked points:
//     gestures      rigid translation, rigid rotation, and a non-rigid deformation
//                   of the tracked points: which features change?
//     closed loop   the loop with each representation from a common start to the
//                   arch target, while a temporary disturbance pulls the grippers
//                   during the first 20 iterations
//     null space    robot motions in Null(J) of the chosen representation
//     looped rod    the loop with each representation, from a looped rod to the
//                   same rod unlooped: shape error and largest axial strain
import { nullSpace, numericalRank } from '../analysis.js';
import { ShapeServoingLoop } from '../control.js';
import { FEATURE_NAMES, featureDimension, featureVector, rigidMotion } from '../features.js';
import { numericalJacobian, offlineEstimate } from '../jacobian.js';
import { column, cumsumFrom0, interp, linspace, maxAbs, mean, norm, scale, sub, zeros } from '../linalg.js';
import { Plant, Q_ARCH, Q_STRAIGHT, probeMotions, ringPlant, targetFromGrasp } from '../running_example.js';
import { Session, scene } from './common.js';

const START = [-2.45, 1.10, 0.55, 2.45, 1.10, -0.55];     // a shallower arch

// The grasp q moved rigidly: rotated by `angle` about `center`, then shifted.
export function transformedGrasp(q, angle, shift, center = [0.0, 1.15]) {
    const c = Math.cos(angle), s = Math.sin(angle);
    const out = Array.from(q);
    for (const i of [0, 3]) {
        const x = q[i] - center[0], y = q[i + 1] - center[1];
        out[i] = c * x - s * y + center[0] + shift[0];
        out[i + 1] = s * x + c * y + center[1] + shift[1];
        out[i + 2] = q[i + 2] + angle;
    }
    return out;
}

export const GAINS = { points: 0.06, edges: 0.08, angles: 0.12 };

// The temporary disturbance of the tutorial's feature figure, the same in every
// run: for the first DISTURBANCE_STEPS iterations it pulls the grippers toward a
// shifted and rotated copy of the target grasp at DISTURBANCE_RATE, then stops.
const DISTURBANCE_TARGET = transformedGrasp(Q_ARCH, -24 * Math.PI / 180, [2.0, 0.15]);
const DISTURBANCE_RATE = 0.03;
const DISTURBANCE_STEPS = 20;
const COMPARE_HORIZON = 80;

// Sum of a_k sin(k pi t): zero at t = 0 and t = 1, oscillating in between.
const wiggle = (t, terms) => terms.reduce((acc, [k, a]) => acc + a * Math.sin(k * Math.PI * t), 0);

const LOOP_HORIZON = 200;

export class FeaturesSession extends Session {
    constructor() {
        super();
        [this.arch_nodes, this.arch_points] = targetFromGrasp(Q_ARCH);
        [this.straight_nodes, this.straight_points] = targetFromGrasp(Q_STRAIGHT);
        this.rep = 'points';
        this.ring_rep = 'points';
        this._setup();
        this._setupRing();
    }

    _makeLoop(rep) {
        const plant = new Plant({ featureMap: rep, q: START });
        const [J0] = offlineEstimate((q) => plant.peek(q), START, probeMotions());
        const loop = new ShapeServoingLoop(plant, featureVector(this.arch_points, rep), J0, {
            alpha: GAINS[rep], damping: 0.05, maxResponse: 25.0, relaxation: 0.4 });
        // The disturbance acts on the robot motion, outside the controller: added, not projected.
        loop.secondary = (q) => (loop.k < DISTURBANCE_STEPS
            ? q.map((x, i) => -DISTURBANCE_RATE * (x - DISTURBANCE_TARGET[i])) : zeros(q.length));
        loop.projectSecondary = false;
        return [plant, J0, loop];
    }

    _setup() {
        [this.plant, this.J0, this.loop] = this._makeLoop(this.rep);
        this.initial_error = norm(this.loop.error);
    }

    state() {
        const rank = numericalRank(this.J0);
        return { scene: scene(this.plant), target: { nodes: this.arch_nodes, points: this.arch_points },
            rep: this.rep, n: featureDimension(5, this.rep), rank, null_dim: 6 - rank,
            k: this.loop.k, error: norm(this.loop.error), initial_error: this.initial_error,
            disturbance_steps: DISTURBANCE_STEPS, horizon: COMPARE_HORIZON };
    }

    // ------------------------------------------------------------- closed loop
    act_set_rep({ rep }) {
        this.rep = rep;
        this._setup();
        return this.state();
    }

    act_reset() {
        this._setup();
        return this.state();
    }

    act_step({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.max(1, Math.trunc(n)); i++) {
            this.loop.step();
            samples.push({ k: this.loop.k, error: norm(this.loop.error) });
        }
        return { ...this.state(), samples };
    }

    // Run every representation from the common start; return the final shapes.
    act_compare({ iterations = COMPARE_HORIZON } = {}) {
        const out = {};
        for (const rep of FEATURE_NAMES) {
            const [plant, , loop] = this._makeLoop(rep);
            const e0 = norm(loop.error);
            for (let i = 0; i < iterations; i++) loop.step();
            out[rep] = { nodes: plant.nodes, points: plant.points, q: plant.q, ratio: norm(loop.error) / e0 };
        }
        return { runs: out, start: new Plant({ q: START }).nodes };
    }

    // ------------------------------------------------------------- looped rod
    _makeRingLoop(rep) {
        const plant = ringPlant(rep);
        const [J0] = offlineEstimate((q) => plant.peek(q), plant.q, probeMotions());
        const loop = new ShapeServoingLoop(plant, featureVector(this.straight_points, rep), J0, {
            alpha: GAINS[rep], damping: 0.05, maxResponse: 25.0, relaxation: 0.4 });
        return [plant, loop];
    }

    _setupRing() {
        [this.ring, this.ring_loop] = this._makeRingLoop(this.ring_rep);
        this.ring_initial_error = norm(this.ring_loop.error);
        this.ring_peak = Math.max(...this.ring.rod.axialStrain(this.ring.nodes));
    }

    _ringState() {
        const strain = Math.max(...this.ring.rod.axialStrain(this.ring.nodes));
        return { scene: scene(this.ring), rep: this.ring_rep, k: this.ring_loop.k,
            target: { nodes: this.straight_nodes, points: this.straight_points },
            error: norm(this.ring_loop.error) / this.ring_initial_error,
            strain, peak_strain: this.ring_peak, energy: this.ring.rod.elasticEnergy(this.ring.nodes) };
    }

    act_ring_state() { return { ring: this._ringState() }; }

    act_set_ring_rep({ rep }) {
        this.ring_rep = rep;
        this._setupRing();
        return { ring: this._ringState() };
    }

    act_reset_ring() {
        this._setupRing();
        return { ring: this._ringState() };
    }

    act_step_ring({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.max(1, Math.trunc(n)); i++) {
            this.ring_loop.step();
            const state = this._ringState();
            this.ring_peak = Math.max(this.ring_peak, state.strain);
            samples.push({ k: state.k, error: state.error, strain: state.strain });
        }
        return { ring: this._ringState(), samples };
    }

    // Run every representation from the looped rod; return error and strain histories.
    act_compare_ring({ iterations = LOOP_HORIZON } = {}) {
        const out = {};
        for (const rep of FEATURE_NAMES) {
            const [plant, loop] = this._makeRingLoop(rep);
            const e0 = norm(loop.error);
            const error = [1.0], strain = [Math.max(...plant.rod.axialStrain(plant.nodes))];
            let peakNodes = plant.nodes, peak = strain[0];
            for (let i = 0; i < Math.trunc(iterations); i++) {
                loop.step();
                error.push(norm(loop.error) / e0);
                strain.push(Math.max(...plant.rod.axialStrain(plant.nodes)));
                if (strain[strain.length - 1] > peak) { peak = strain[strain.length - 1]; peakNodes = plant.nodes; }
            }
            out[rep] = { error, strain, nodes: plant.nodes, points: plant.points, peak_nodes: peakNodes, peak };
        }
        return { runs: out, start: ringPlant().nodes,
            target: { nodes: this.straight_nodes, points: this.straight_points } };
    }

    // ------------------------------------------------------------- gestures
    // Features of all three maps while the tracked points translate, rotate, or deform.
    act_gesture({ kind, frames = 121 }) {
        const points = this.arch_points, nodes = this.arch_nodes;
        const xi = linspace(-1, 1, points.length);
        const bow = xi.map((x) => [0.10 * Math.sin(Math.PI * x), Math.cos(0.5 * Math.PI * x)]);
        const ess = xi.map((x) => [0, Math.sin(Math.PI * x)]);
        const base = Object.fromEntries(FEATURE_NAMES.map((rep) => [rep, featureVector(points, rep)]));
        const arc = cumsumFrom0(nodes.slice(1).map((p, i) => Math.hypot(p[0] - nodes[i][0], p[1] - nodes[i][1])));
        const pointArc = linspace(0.12, 0.88, points.length).map((f) => f * arc[arc.length - 1]);
        const out = [];
        for (const t of linspace(0, 1, frames)) {
            let moved, movedNodes;
            if (kind === 'translate') {
                const shift = [wiggle(t, [[2, 0.45], [5, 0.18]]), wiggle(t, [[3, 0.30], [6, -0.14]])];
                moved = rigidMotion(points, 0, shift);
                movedNodes = nodes.map((p) => [p[0] + shift[0], p[1] + shift[1]]);
            } else if (kind === 'rotate') {
                const angle = wiggle(t, [[2, 22.0], [5, 9.0]]) * Math.PI / 180;
                moved = rigidMotion(points, angle);
                const c = Math.cos(angle), s = Math.sin(angle);
                const cx = mean(points.map((p) => p[0])), cy = mean(points.map((p) => p[1]));
                movedNodes = nodes.map(([x, y]) => [c * (x - cx) - s * (y - cy) + cx, s * (x - cx) + c * (y - cy) + cy]);
            } else {
                const w1 = wiggle(t, [[3, 0.55]]), w2 = wiggle(t, [[4, 0.32], [7, 0.10]]);
                moved = points.map((p, i) => [p[0] + w1 * bow[i][0] + w2 * ess[i][0], p[1] + w1 * bow[i][1] + w2 * ess[i][1]]);
                const shift = moved.map((p, i) => [p[0] - points[i][0], p[1] - points[i][1]]);
                // drawn on the rod by interpolating along its length
                const sx = shift.map((d) => d[0]), sy = shift.map((d) => d[1]);
                movedNodes = nodes.map((p, i) => [p[0] + interp(arc[i], pointArc, sx), p[1] + interp(arc[i], pointArc, sy)]);
            }
            out.push({ nodes: movedNodes, points: moved,
                features: Object.fromEntries(FEATURE_NAMES.map((rep) => [rep, sub(featureVector(moved, rep), base[rep])])) });
        }
        return { kind, frames: out };
    }

    // ------------------------------------------------------------- null space
    // Robot motion along the first direction of Null(J) for the chosen features.
    act_null_motion({ frames = 48, amplitude = 0.5 } = {}) {
        const J = numericalJacobian((q) => this.plant.peek(q), this.plant.q);
        const basis = nullSpace(J);
        if (basis[0].length === 0) return { frames: [], null_dim: 0 };
        const first = column(basis, 0);
        const direction = scale(first, amplitude / maxAbs(first));
        const s0 = this.plant.s, p0 = this.plant.points;
        let nodes = this.plant.nodes;
        const out = [];
        for (const t of linspace(0, 1, frames + 1)) {
            const q = this.plant.q.map((x, i) => x + Math.sin(2 * Math.PI * t) * direction[i]);
            nodes = this.plant.rod.equilibrium(q, nodes);
            const points = this.plant.pointsOf(nodes);
            let d2 = 0;
            points.forEach((p, i) => { d2 += (p[0] - p0[i][0]) ** 2 + (p[1] - p0[i][1]) ** 2; });
            out.push({ q, nodes, points, feature_change: norm(sub(this.plant.features(nodes), s0)), point_change: Math.sqrt(d2) });
        }
        return { frames: out, null_dim: basis[0].length, direction };
    }
}

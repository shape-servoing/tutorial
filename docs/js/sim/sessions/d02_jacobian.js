// Demo 02 - The deformation Jacobian (port of demos/sessions/d02_jacobian.py).
//
// At the current grasp, the exact Jacobian J = ds/dq (finite differences of the
// equilibrium, available in simulation) predicts the feature change of a small
// robot motion, ds ~ J dq. The page compares this prediction with the rod's
// actual response, and shows how the agreement degrades with step size.
import { stretchedGrasp } from '../failures.js';
import { numericalJacobian } from '../jacobian.js';
import { add, linspace, matvec, norm, reshape, scale, sub } from '../linalg.js';
import { Plant, Q_ARCH, Q_NOMINAL } from '../running_example.js';
import { Session, admissible, scene } from './common.js';

const PRESETS = {
    nominal: Q_NOMINAL,
    slack: [-2.3, 1.0, 0.0, 2.3, 1.0, 0.0],
    taut: stretchedGrasp(6.6),
    arch: Q_ARCH,
};
const STEP_SCALES = linspace(0.1, 2.0, 12);

export class JacobianSession extends Session {
    constructor() {
        super();
        this.plant = new Plant({ q: Q_NOMINAL });
        this.dq = [0.0, 0.5, 0.0, 0.0, 0.0, 0.0];
        this.J = this._exact();
    }

    _exact() { return numericalJacobian((q) => this.plant.peek(q), this.plant.q); }

    // Actual feature change of the motion dq from the current equilibrium.
    _response(dq) {
        const nodes = this.plant.rod.equilibrium(add(this.plant.q, dq), this.plant.nodes);
        return [nodes, sub(this.plant.features(nodes), this.plant.s)];
    }

    state() {
        const [nodesAfter, actual] = this._response(this.dq);
        const predicted = matvec(this.J, this.dq);
        const steps = [], errors = [];
        if (norm(this.dq) > 1e-9) {
            for (const beta of STEP_SCALES) {
                const ds = this._response(scale(this.dq, beta))[1];
                steps.push(beta * norm(this.dq));
                errors.push(norm(sub(ds, scale(predicted, beta))) / Math.max(norm(ds), 1e-12));
            }
        }
        return { scene: scene(this.plant), J: this.J, dq: this.dq, predicted, actual,
            after: { nodes: nodesAfter, points: this.plant.pointsOf(nodesAfter) },
            relative_error: norm(sub(actual, predicted)) / Math.max(norm(actual), 1e-12),
            curve: { steps, errors } };
    }

    act_move({ q }) {
        this.plant.moveTo(admissible(q));
        this.J = this._exact();
        return this.state();
    }

    act_set_dq({ dq }) {
        this.dq = dq.map(Number);
        return this.state();
    }

    act_preset({ name }) {
        this.plant.reset(PRESETS[name]);
        this.J = this._exact();
        return this.state();
    }

    // The robot oscillates along dq: q(t) = q + sin(2 pi t) dq, rod re-solved at each frame.
    act_play({ frames = 48 } = {}) {
        let nodes = this.plant.nodes;
        const out = [];
        const base = this.plant.points;
        const predictedPoints = reshape(matvec(this.J, this.dq), 2);
        for (const t of linspace(0, 1, frames + 1)) {
            const w = Math.sin(2 * Math.PI * t);
            const q = add(this.plant.q, scale(this.dq, w));
            nodes = this.plant.rod.equilibrium(q, nodes);
            out.push({ q, nodes, points: this.plant.pointsOf(nodes),
                predicted: base.map((p, i) => [p[0] + w * predictedPoints[i][0], p[1] + w * predictedPoints[i][1]]) });
        }
        return { frames: out };
    }
}

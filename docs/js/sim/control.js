// The control law and the complete shape-servoing loop, Algorithm 1
// (port of shape_servoing/control.py).
//
//     u = alpha J_hat^+ e                                 (pseudoinverse law)
//     u = alpha J_hat^T (J_hat J_hat^T + mu^2 I)^{-1} e   (damped least squares)
//
// `ShapeServoingLoop.step` runs one iteration: error, command, motion,
// measurement, and the gated Broyden update. Extensions (Appendix E): a feedforward
// term, a secondary objective (optionally projected onto Null(J_hat)), and
// `FeatureTask`, a secondary objective defined by a second feature map.
import { featureMapJacobian, featureVector } from './features.js';
import { broydenUpdate, offlineEstimate } from './jacobian.js';
import { add, cholesky, choleskySolve, matmul, matvec, norm, pinv, scale, sub, transpose } from './linalg.js';
import { probeMotions } from './running_example.js';

// Robot velocity u for the error e (pseudoinverse, or damped if damping > 0).
export function controlLaw(J, e, alpha, damping = 0) {
    if (damping <= 0) return scale(matvec(pinv(J), e), alpha);
    const A = matmul(J, transpose(J)).map((row, i) => row.map((x, j) => (i === j ? x + damping * damping : x)));
    return scale(matvec(transpose(J), choleskySolve(cholesky(A), e)), alpha);
}

// (I - J^+ J) v: the component of v in Null(J).
function projectNull(J, v) {
    const Jp = pinv(J);
    return sub(v, matvec(Jp, matvec(J, v)));
}

export class ShapeServoingLoop {
    constructor(plant, sStar, J0, { alpha = 0.05, dt = 1.0, onlineUpdate = true, damping = 0.0,
        minStep = 1e-6, maxResponse = Infinity, relaxation = 1.0 } = {}) {
        this.plant = plant;
        this.sStar = Array.from(sStar);
        this.J_hat = J0.map((row) => Array.from(row));
        Object.assign(this, { alpha, dt, onlineUpdate, damping });
        // Remark on gating: skip updates from tiny steps, and reject pairs whose
        // response ratio ||ds|| / ||dq|| is implausibly large.
        Object.assign(this, { minStep, maxResponse, relaxation });
        this.secondary = null;              // q -> robot velocity, or a FeatureTask
        this.projectSecondary = true;       // pursue it within Null(J_hat)
        this.jacobianFrom = null;           // FeatureTask giving J_hat by the chain rule
        this.s = plant.s;                   // line 1: measure s_0
        this.k = 0;
    }

    get error() { return sub(this.sStar, this.s); }

    // Obtain J_hat from a point-coordinate FeatureTask by the chain rule,
    // J_hat = (ds/ds_p) J_p_hat, at every iteration, instead of Broyden's rule.
    deriveJacobianFrom(task) {
        this.jacobianFrom = task;
        this.J_hat = task.chainedJacobian(this.plant.featureMap);
    }

    // u_k = alpha J_hat^+ e_k, plus the optional extensions of Appendix E.
    command(e, sStarRate = null) {
        let u = controlLaw(this.J_hat, e, this.alpha, this.damping);
        if (sStarRate) u = add(u, matvec(pinv(this.J_hat), sStarRate));            // moving target
        if (this.secondary) {                                                       // secondary task
            let u2 = typeof this.secondary === 'function' ? this.secondary(this.plant.q) : this.secondary.call(this.plant.q);
            if (this.projectSecondary) u2 = projectNull(this.J_hat, u2);
            u = add(u, u2);
        }
        return u;
    }

    step(sStarRate = null) {
        if (this.jacobianFrom) this.J_hat = this.jacobianFrom.chainedJacobian(this.plant.featureMap);
        const e = sub(this.sStar, this.s);                                         // line 3
        const u = this.command(e, sStarRate);                                       // line 4
        const dq = scale(u, this.dt);                                               // line 5
        const sNext = this.plant.moveTo(add(this.plant.q, dq));                    // line 6
        const ds = sub(sNext, this.s);                                              // line 7
        let r = sub(ds, matvec(this.J_hat, dq));                                    // line 8
        const step = norm(dq);
        const updated = this.onlineUpdate && !this.jacobianFrom && step > this.minStep
            && norm(ds) <= this.maxResponse * step;
        if (updated) [this.J_hat, r] = broydenUpdate(this.J_hat, dq, ds, this.relaxation);
        const tasks = this.secondary instanceof FeatureTask ? [this.secondary] : [];
        if (this.jacobianFrom && this.jacobianFrom !== this.secondary) tasks.push(this.jacobianFrom);
        for (const task of tasks) task.observe(dq);
        const record = { k: this.k, e, u, dq, ds, r, updated };
        this.s = sNext;                                                             // line 9
        this.k += 1;
        return record;
    }
}

// Secondary objective defined by a second feature map: u_2 = beta J2_hat^+ (s2* - s2)
// (damped inverse), with its own estimate J2_hat, initialized by offline regression
// around the current grasp and updated with Broyden's rule from the same robot motions.
export class FeatureTask {
    constructor(plant, featureMap, sStar, { gain = 0.05, damping = 0.05, relaxation = 1.0,
        maxResponse = 25.0, probes = null } = {}) {
        Object.assign(this, { plant, featureMap, gain, damping, relaxation, maxResponse });
        this.sStar = Array.from(sStar);
        const measure = (q) => this._features(plant.rod.equilibrium(q, plant.nodes));
        [this.J_hat] = offlineEstimate(measure, plant.q, probes || probeMotions());
        this.s = this._features(plant.nodes);
    }

    _features(nodes) { return featureVector(this.plant.pointsOf(nodes), this.featureMap); }

    get error() { return sub(this.sStar, this.s); }

    call() { return controlLaw(this.J_hat, this.error, this.gain, this.damping); }

    // (ds/ds_p) J_p_hat: Jacobian of another feature map of the tracked points.
    chainedJacobian(featureMap) {
        if (this.featureMap !== 'points') throw new Error('the chain rule needs a point-coordinate task');
        return matmul(featureMapJacobian(this.plant.points, featureMap), this.J_hat);
    }

    // Measure s2 after the robot moved by dq and update J2_hat (gated).
    observe(dq) {
        const sNext = this._features(this.plant.nodes);
        const ds = sub(sNext, this.s), step = norm(dq);
        if (step > 1e-6 && norm(ds) <= this.maxResponse * step) [this.J_hat] = broydenUpdate(this.J_hat, dq, ds, this.relaxation);
        this.s = sNext;
    }
}

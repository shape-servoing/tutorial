// Demo 04 - Closing the loop, Algorithm 1 (port of demos/sessions/d04_loop.py).
//
// The shape-servoing loop runs from q0 toward a target, starting from the
// offline estimate J0. Each `step` executes Algorithm 1 iterations; a single
// step also returns the intermediate signals (e_k, u_k, dq_k, ds_{k+1}, the
// prediction error, and the rank-one Broyden correction) for step-by-step display.
import { conditionNumber } from '../analysis.js';
import { ShapeServoingLoop } from '../control.js';
import { offlineEstimate } from '../jacobian.js';
import { copyM, norm, subM } from '../linalg.js';
import { Random } from '../random.js';
import { Plant, Q_ARCH, Q_NOMINAL, probeMotions, targetFromGrasp } from '../running_example.js';
import { Session, admissible, randomTarget, scene, targetScene } from './common.js';

const GATE = 25.0;      // default largest accepted response ratio ||ds|| / ||dq|| when gating is on

export class LoopSession extends Session {
    constructor() {
        super();
        this.rng = new Random(4);
        this.plant = new Plant({ q: Q_NOMINAL });
        [this.J0] = offlineEstimate((q) => this.plant.peek(q), Q_NOMINAL, probeMotions());
        Object.assign(this, { alpha: 0.05, online: true, gate: false, gate_ratio: GATE });
        this._setTarget(...targetFromGrasp(Q_ARCH));
    }

    _setTarget(nodes, points, sStar) {
        this.target = targetScene(nodes, points);
        this.s_star = sStar;
        this._restart();
    }

    // Back to q0 with the offline estimate J0: a new run of Algorithm 1.
    _restart() {
        this.plant.reset(Q_NOMINAL);
        this.loop = new ShapeServoingLoop(this.plant, this.s_star, this.J0, {
            alpha: this.alpha, onlineUpdate: this.online, maxResponse: this.gate ? this.gate_ratio : Infinity });
        this.initial_error = norm(this.loop.error);
    }

    state() {
        return { scene: scene(this.plant), target: this.target, k: this.loop.k,
            e: this.loop.error, error: norm(this.loop.error), initial_error: this.initial_error,
            J_hat: this.loop.J_hat, condition: conditionNumber(this.loop.J_hat),
            params: { alpha: this.alpha, online: this.online, gate: this.gate, gate_ratio: this.gate_ratio } };
    }

    // ------------------------------------------------------------- actions
    // Run n iterations. Returns the new samples, and the details of the last one.
    act_step({ n = 1 } = {}) {
        const before = { nodes: copyM(this.plant.nodes), points: this.plant.points, q: Array.from(this.plant.q) };
        const J_before = copyM(this.loop.J_hat);
        const samples = [];
        let it = null;
        for (let i = 0; i < Math.trunc(n); i++) {
            it = this.loop.step();
            const dqNorm = norm(it.dq);
            samples.push({ k: this.loop.k, error: norm(this.loop.error), u: it.u, updated: it.updated,
                correction: norm(it.r) / Math.max(dqNorm, 1e-12),
                ratio: norm(it.ds) / Math.max(dqNorm, 1e-12),
                rejected: Boolean(this.online && dqNorm > this.loop.minStep && !it.updated) });
        }
        const detail = { before, e: it.e, u: it.u, dq: it.dq, ds: it.ds, r: it.r, updated: it.updated,
            J_before, correction: subM(this.loop.J_hat, J_before) };
        return { ...this.state(), samples, detail };
    }

    act_set_params({ alpha = null, online = null, gate = null, gate_ratio = null } = {}) {
        if (alpha !== null) this.alpha = Number(alpha);
        if (online !== null) this.online = Boolean(online);
        if (gate !== null) this.gate = Boolean(gate);
        if (gate_ratio !== null) this.gate_ratio = Number(gate_ratio);
        this.loop.alpha = this.alpha;
        this.loop.onlineUpdate = this.online;
        this.loop.maxResponse = this.gate ? this.gate_ratio : Infinity;
        return this.state();
    }

    act_reset() {
        this._restart();
        return this.state();
    }

    // 'arch', 'random', or 'current': the shape the rod has now (by demonstration).
    act_set_target({ kind }) {
        if (kind === 'arch') {
            this._setTarget(...targetFromGrasp(Q_ARCH));
        } else if (kind === 'random') {
            this.plant.reset(Q_NOMINAL);
            const [, nodes, points, sStar] = randomTarget(this.rng, this.plant);
            this._setTarget(nodes, points, sStar);
        } else {
            this._setTarget(copyM(this.plant.nodes), this.plant.points, this.plant.s);
        }
        return this.state();
    }

    // Move the grippers by hand (paused): no robot command, so no Broyden pair.
    act_move({ q }) {
        this.loop.s = this.plant.moveTo(admissible(q));
        return this.state();
    }
}

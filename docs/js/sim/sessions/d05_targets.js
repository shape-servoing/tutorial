// Demo 05 - Target feasibility and reachability (port of demos/sessions/d05_targets.py).
//
// A feasible and reached target (the equilibrium at another grasp) and a
// synthetic infeasible one (three points exceed the deformation limits). The
// page splits the error into the part the controller acts on, J J^+ e in
// Range(J), and the rest, which no small robot motion changes.
import { rangeSplit } from '../analysis.js';
import { ShapeServoingLoop } from '../control.js';
import { numericalJacobian, offlineEstimate } from '../jacobian.js';
import { copyM, flatten, norm } from '../linalg.js';
import { Plant, Q_NOMINAL, Q_REACHABLE, probeMotions, targetFromGrasp, unfeasiblePoints } from '../running_example.js';
import { Session, scene } from './common.js';

export class TargetsSession extends Session {
    constructor() {
        super();
        this.plant = new Plant({ q: Q_NOMINAL });
        [this.J0] = offlineEstimate((q) => this.plant.peek(q), Q_NOMINAL, probeMotions());
        [this.feasible_nodes, this.feasible_points] = targetFromGrasp(Q_REACHABLE);
        Object.assign(this, { exact: false, alpha: 0.06 });
        this.kind = 'reachable';
        this.points = copyM(this.feasible_points);
        this._restart();
    }

    _restart() {
        this.plant.reset(Q_NOMINAL);
        this.loop = new ShapeServoingLoop(this.plant, flatten(this.points), this.J0, {
            alpha: this.alpha, damping: 0.02, maxResponse: 25.0, onlineUpdate: !this.exact });
        this.initial_error = norm(this.loop.error);
    }

    _exactJacobian() { return numericalJacobian((q) => this.plant.peek(q), this.plant.q); }

    // Split e into its component in Range(J_hat) and the orthogonal component.
    _split() {
        const J = this._exactJacobian();
        if (this.exact) this.loop.J_hat = J;
        const e = this.loop.error;
        const [eRange, eOrth] = rangeSplit(this.loop.J_hat, e);
        const [exactRange, exactOrth] = rangeSplit(J, e);
        const n = Math.max(norm(e), 1e-12);
        return { e, e_range: eRange, e_orth: eOrth, range_fraction: norm(eRange) / n,
            orth_fraction: norm(eOrth) / n, exact_range_fraction: norm(exactRange) / n,
            exact_orth_fraction: norm(exactOrth) / n };
    }

    state() {
        const nodes = this.kind === 'reachable' ? this.feasible_nodes : null;
        return { scene: scene(this.plant), target: { nodes, points: this.points }, kind: this.kind,
            k: this.loop.k, error: norm(this.loop.error), initial_error: this.initial_error,
            split: this._split(), params: { exact: this.exact, alpha: this.alpha } };
    }

    // ------------------------------------------------------------- actions
    act_step({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.max(1, Math.trunc(n)); i++) {
            if (this.exact) this.loop.J_hat = this._exactJacobian();
            const it = this.loop.step();
            samples.push({ k: this.loop.k, error: norm(this.loop.error), command: norm(it.u) });
        }
        return { ...this.state(), samples };
    }

    act_set_target({ kind }) {
        if (kind !== 'reachable' && kind !== 'unfeasible') throw new Error(`unknown target class: ${kind}`);
        this.kind = kind;
        this.points = kind === 'reachable' ? copyM(this.feasible_points) : unfeasiblePoints(this.feasible_points);
        this._restart();
        return this.state();
    }

    // Drag one target point: the target is no longer a recorded shape.
    act_move_point({ index, xy }) {
        this.kind = 'custom';
        this.points[Math.trunc(index)] = xy.map(Number);
        this.loop.sStar = flatten(this.points);
        if (this.loop.k === 0) this.initial_error = norm(this.loop.error);
        return this.state();
    }

    act_set_params({ exact = null, alpha = null } = {}) {
        if (exact !== null) this.exact = Boolean(exact);
        if (alpha !== null) this.alpha = Number(alpha);
        this._restart();
        return this.state();
    }

    act_reset() {
        this._restart();
        return this.state();
    }
}

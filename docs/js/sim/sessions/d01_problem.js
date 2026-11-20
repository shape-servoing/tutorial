// Demo 01 - The shape-servoing problem (port of demos/sessions/d01_problem.py).
//
// The user moves the grippers by hand. The rod settles to its quasistatic
// equilibrium, perception returns the tracked points, and the page shows the
// feature vector s, the target s*, and the shape error e = s* - s.
import { norm, sub } from '../linalg.js';
import { Random } from '../random.js';
import { Plant, Q_ARCH, Q_NOMINAL, targetFromGrasp } from '../running_example.js';
import { Session, admissible, randomTarget, scene, targetScene } from './common.js';

export class ProblemSession extends Session {
    constructor() {
        super();
        this.rng = new Random(1);
        this.plant = new Plant({ q: Q_NOMINAL });
        this._setTarget(...targetFromGrasp(Q_ARCH));
    }

    _setTarget(nodes, points, sStar) {
        this.target = targetScene(nodes, points);
        this.s_star = sStar;
        this.initial_error = norm(sub(sStar, this.plant.s));
    }

    state() {
        const s = this.plant.s;
        const e = sub(this.s_star, s);
        return { scene: scene(this.plant), target: this.target, s, s_star: this.s_star, e,
            error_norm: norm(e), initial_error: this.initial_error };
    }

    // Move the grippers to q (a drag or a slider) and let the rod settle.
    act_move({ q }) {
        this.plant.moveTo(admissible(q));
        return this.state();
    }

    act_new_target() {
        const [, nodes, points, sStar] = randomTarget(this.rng, this.plant);
        this._setTarget(nodes, points, sStar);
        return this.state();
    }

    act_reset() {
        this.plant.reset(Q_NOMINAL);
        this._setTarget(...targetFromGrasp(Q_ARCH));
        return this.state();
    }
}

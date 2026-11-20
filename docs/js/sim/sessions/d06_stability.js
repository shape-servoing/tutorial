// Demo 07 - Local stability: the descent condition (port of demos/sessions/d06_stability.py).
//
// For the running example n > m, the paper's sufficient condition is positivity
// of Sym(J_hat^+ J) on Range(J^T). The page builds J_hat by corrupting the exact
// J (scaling, rotating two robot-coordinate columns into each other, a random
// perturbation, a flipped column), evaluates the restricted eigenvalues and the
// descent angles, and runs the loop with J_hat held fixed.
import { descentAngles, descentMargin, singularTriplets } from '../analysis.js';
import { ShapeServoingLoop } from '../control.js';
import { numericalJacobian } from '../jacobian.js';
import { add, copyV, dot, eye, histogram, linspace, matmul, matvec, mean, norm, normF, pinv, scale,
    scaleM, subM } from '../linalg.js';
import { Random } from '../random.js';
import { Plant, Q_NOMINAL } from '../running_example.js';
import { Session, scene } from './common.js';

// Robot-coordinate planes in which the estimate rotates the columns of J.
const PLANES = { 'xL-xR': [0, 3], 'yL-thetaL': [1, 2], 'thetaL-thetaR': [2, 5] };
const ANGLE_BINS = linspace(0.0, 180.0, 37);
const MAX_GROWTH = 6.0;       // stop a diverging run when ||e_k|| exceeds this multiple of ||e_0||

export class StabilitySession extends Session {
    constructor() {
        super();
        const rng = new Random(7);
        this.plant = new Plant({ q: Q_NOMINAL });
        this.q_c = copyV(Q_NOMINAL);
        this.J = numericalJacobian((q) => this.plant.peek(q), this.q_c);
        this.perturbation = rng.normalMatrix(this.J.length, this.J[0].length);
        // Random unit errors in Range(J), for the descent-angle histogram.
        const [, Phi] = singularTriplets(this.J);
        const z = rng.normalMatrix(Phi[0].length, 600);
        const zn = z[0].map((_, c) => Math.sqrt(z.reduce((s, row) => s + row[c] * row[c], 0)));
        this.samples = matmul(Phi, z.map((row) => row.map((x, c) => x / zn[c])));
        this.rng = new Random(11);
        this.params = { scale: 1.0, angle: 0.0, plane: 'yL-thetaL', noise: 0.0, flip: -1 };
        this.alpha = 0.3;
        this.distance = 0.4;
        this.direction = [0.5, 0.6, 0.3, -0.4, 0.2, -0.3];
        this._estimate();
        this._newTarget();
    }

    // ------------------------------------------------------------- model
    _estimate() {
        const p = this.params;
        const [i, j] = PLANES[p.plane];
        const R = eye(6);
        const c = Math.cos(p.angle * Math.PI / 180), s = Math.sin(p.angle * Math.PI / 180);
        R[i][i] = c; R[i][j] = -s; R[j][i] = s; R[j][j] = c;
        let J_hat = scaleM(matmul(this.J, R), p.scale);
        const pert = scaleM(this.perturbation, p.noise * normF(this.J) / normF(this.perturbation));
        J_hat = J_hat.map((row, r) => row.map((x, k) => x + pert[r][k]));
        if (p.flip >= 0) J_hat = J_hat.map((row) => row.map((x, k) => (k === p.flip ? -x : x)));
        this.J_hat = J_hat;
    }

    // s* = s(q_c + delta): locally reachable by construction.
    _newTarget() {
        const dq = scale(this.direction, this.distance / norm(this.direction));
        const nodes = this.plant.rod.equilibrium(add(this.q_c, dq), this.plant.nodes);
        this.target = { nodes, points: this.plant.pointsOf(nodes) };
        this.s_star = this.plant.features(nodes);
        this._restart();
    }

    _restart() {
        this.plant.reset(this.q_c);
        this.loop = new ShapeServoingLoop(this.plant, this.s_star, this.J_hat, { alpha: this.alpha, onlineUpdate: false });
        this.initial_error = norm(this.loop.error);
    }

    _rate(e) {
        const velocity = matvec(this.J, matvec(pinv(this.J_hat), e));
        return dot(e, velocity) / Math.max(dot(e, e), 1e-300);
    }

    state() {
        const [margin, eigenvalues] = descentMargin(this.J, this.J_hat);
        const angles = descentAngles(this.J, this.J_hat, this.samples).map((a) => a * 180 / Math.PI);
        const e = this.loop.error;
        const velocity = matvec(this.J, matvec(pinv(this.J_hat), e));      // feature velocity the command produces
        return { scene: scene(this.plant), target: this.target, k: this.loop.k,
            error: norm(e), initial_error: this.initial_error, e, velocity,
            J: this.J, J_hat: this.J_hat,
            matrix_error: normF(subM(this.J, this.J_hat)) / normF(this.J),
            margin, eigenvalues,
            angle_counts: histogram(angles, ANGLE_BINS), angle_bins: ANGLE_BINS,
            descent_fraction: mean(angles.map((a) => (a < 90 ? 1 : 0))), rate: this._rate(e),
            params: { ...this.params, alpha: this.alpha, distance: this.distance } };
    }

    // ------------------------------------------------------------- actions
    act_set_estimate(params = {}) {
        for (const [key, value] of Object.entries(params)) if (key in this.params) this.params[key] = value;
        this._estimate();
        this._restart();
        return this.state();
    }

    act_set_alpha({ alpha }) {
        this.alpha = Number(alpha);
        this._restart();
        return this.state();
    }

    act_new_target({ distance = null } = {}) {
        if (distance !== null) this.distance = Number(distance);
        this.direction = this.rng.normals(6).map((x, i) => x * [1, 1, 0.5, 1, 1, 0.5][i]);
        this._newTarget();
        return this.state();
    }

    act_reset() {
        this._restart();
        return this.state();
    }

    act_step({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.max(1, Math.trunc(n)); i++) {
            if (norm(this.loop.error) > MAX_GROWTH * this.initial_error) break;
            this.loop.step();
            const e = this.loop.error;
            samples.push({ k: this.loop.k, error: norm(e), rate: this._rate(e) });
        }
        const diverged = norm(this.loop.error) > MAX_GROWTH * this.initial_error;
        return { ...this.state(), samples, diverged };
    }
}

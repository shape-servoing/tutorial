// Demo 10 - Three ways the loop fails (port of demos/sessions/d09_failures.py).
//
//     controllability   pull the rod taut: sigma_min and w of the exact Jacobian
//                       fall, and the pseudoinverse turns errors along the weak
//                       direction into large commands (damped inverse: bounded)
//     observability     a motion the features do not register gives the same
//                       measured pair (dq != 0, ds ~ 0) as a lost direction
//     instability       compress the rod, or rotate the wrists of an arch until
//                       it snaps through: small dq, large ds
import { singularTriplets } from '../analysis.js';
import { bucklingOnset, responseRatio, straightRodStiffness, stretchedGrasp, tautSweep } from '../failures.js';
import { numericalJacobian } from '../jacobian.js';
import { column, eigvalsh, linspace, maxAbs, norm, scale, sub } from '../linalg.js';
import { Plant } from '../running_example.js';
import { Session, scene } from './common.js';

const REST = 6.0;

// Grippers 4.8 apart, tangents tilted by +/- theta (an arch for theta > 0).
export const archGrasp = (theta) => [-2.4, 1.0, theta, 2.4, 1.0, -theta];

const exactJacobian = (plant) => numericalJacobian((q) => plant.peek(q), plant.q);

export class FailuresSession extends Session {
    constructor() {
        super();
        const extensions = linspace(0.0, 20.0, 25);
        this.sweep = { extension: extensions,
            ...tautSweep(new Plant({ q: stretchedGrasp(REST) }), extensions.map((x) => REST * (1 + x / 100))) };
        const compressions = linspace(-10.0, 20.0, 31);
        this.taut = new Plant({ q: stretchedGrasp(REST) });
        const separations = compressions.map((c) => REST * (1 - c / 100));
        this.buckling = { compression: compressions,
            lambda: separations.map((d) => straightRodStiffness(this.taut.rod, d)),
            onset: 100 * (REST - bucklingOnset(this.taut.rod, separations)) / REST };
        this.mu = 0.05;
        this.extension = 0.0;
        this._resetCompression();
        this._resetSnap();
    }

    state() {
        return { taut: this._tautState(), sweep: this.sweep, buckling: this.buckling,
            compress: this._compressState(), snap: this._snapState() };
    }

    // ------------------------------------------------------------- controllability
    _tautState() {
        const J = exactJacobian(this.taut);
        const [sigma, Phi, Psi] = singularTriplets(J);
        const last = sigma.length - 1;
        const psiMin = column(Psi, last);
        const sign = Math.sign(psiMin[2] || 1.0);
        return { scene: scene(this.taut), extension: this.extension, sigma,
            psi_min: scale(psiMin, sign), phi_min: column(Phi, last),
            wrist_share: psiMin[2] ** 2 + psiMin[5] ** 2, mu: this.mu,
            pinv_gain: sigma.map((s) => 1 / s), damped_gain: sigma.map((s) => s / (s * s + this.mu * this.mu)) };
    }

    act_taut({ extension }) {
        this.extension = Number(extension);
        this.taut.moveTo(stretchedGrasp(REST * (1 + this.extension / 100)));
        return { taut: this._tautState() };
    }

    act_set_mu({ mu }) {
        this.mu = Number(mu);
        return { taut: this._tautState() };
    }

    // ------------------------------------------------------------- observability
    // Robot motion and measured response for three cases with the same question:
    // does ds follow dq?
    act_observe({ scenario, frames = 40 }) {
        let plant, direction;
        if (scenario === 'controllability') {          // taut rod, motion along the weakest direction
            plant = new Plant({ q: stretchedGrasp(REST * 1.2) });
            const [, , Psi] = singularTriplets(exactJacobian(plant));
            direction = column(Psi, Psi[0].length - 1);
        } else if (scenario === 'observability') {     // rigid translation, turning-angle features
            plant = new Plant({ featureMap: 'angles', q: stretchedGrasp(5.4) });
            direction = [1.0, 0.6, 0.0, 1.0, 0.6, 0.0];
        } else {                                        // a healthy case: lift the left gripper
            plant = new Plant({ q: stretchedGrasp(REST) });
            direction = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0];
        }
        direction = scale(direction, 0.5 / maxAbs(direction));
        const s0 = plant.s, q0 = Array.from(plant.q);
        let nodes = plant.nodes;
        const out = [];
        for (const t of linspace(0, 1, frames + 1)) {
            const q = q0.map((x, i) => x + Math.sin(2 * Math.PI * t) * direction[i]);
            nodes = plant.rod.equilibrium(q, nodes);
            const ds = sub(plant.features(nodes), s0);
            out.push({ q, nodes, points: plant.pointsOf(nodes), dq: norm(sub(q, q0)), ds: norm(ds) });
        }
        const peak = out[Math.floor(frames / 4)];
        return { scenario, feature_map: plant.featureMap, frames: out, ratio: peak.ds / Math.max(peak.dq, 1e-12) };
    }

    // ------------------------------------------------------------- instability
    _resetCompression() {
        this.compress = new Plant({ q: stretchedGrasp(REST * 1.05) });
        this.compression = -5.0;
        this.compress_ratio = null;
    }

    _compressState() {
        const q = this.compress.q;
        const straight = this.compress.rod.straightNodes(q);
        return { scene: scene(this.compress), straight, compression: this.compression,
            lambda_straight: straightRodStiffness(this.compress.rod, REST * (1 - this.compression / 100)),
            lambda_equilibrium: eigvalsh(this.compress.rod.freeHessian(this.compress.nodes))[0],
            ratio: this.compress_ratio };
    }

    act_compress({ compression }) {
        const sBefore = this.compress.s, qBefore = Array.from(this.compress.q);
        this.compression = Number(compression);
        const s = this.compress.moveTo(stretchedGrasp(REST * (1 - this.compression / 100)));
        if (norm(sub(this.compress.q, qBefore)) > 1e-9) {
            this.compress_ratio = responseRatio(sub(this.compress.q, qBefore), sub(s, sBefore));
        }
        return { compress: this._compressState() };
    }

    _resetSnap() {
        this.snap = new Plant({ q: archGrasp(0.8) });
        this.snap_path = { theta: [0.8], lambda: [this._snapLambda()], ratio: [null] };
    }

    _snapLambda() { return eigvalsh(this.snap.rod.freeHessian(this.snap.nodes))[0]; }

    _snapState() { return { scene: scene(this.snap), path: this.snap_path }; }

    act_wrist({ theta }) {
        const sBefore = this.snap.s, qBefore = Array.from(this.snap.q);
        const s = this.snap.moveTo(archGrasp(Number(theta)));
        if (norm(sub(this.snap.q, qBefore)) > 1e-9) {
            this.snap_path.theta.push(Number(theta));
            this.snap_path.lambda.push(this._snapLambda());
            this.snap_path.ratio.push(responseRatio(sub(this.snap.q, qBefore), sub(s, sBefore)));
        }
        return { snap: this._snapState() };
    }

    act_reset_instability() {
        this._resetCompression();
        this._resetSnap();
        return { compress: this._compressState(), snap: this._snapState() };
    }
}

// Demo 03 - Offline estimation by regression (port of demos/sessions/d03_offline.py).
//
// The robot probes the rod with C small motions around q0 and fits
// J0 = dS dQ^+. The page varies the probing amplitude, the number of probes,
// and the measurement noise, and compares the estimate with the exact Jacobian
// (simulation only) and with a model-based estimate from a mismatched rod model.
import { conditionNumber, singularTriplets } from '../analysis.js';
import { numericalJacobian, offlineEstimate } from '../jacobian.js';
import { add, column, geomspace, mean, normF, scale, subM } from '../linalg.js';
import { Random } from '../random.js';
import { Rod } from '../rod.js';
import { PROBE_AMPLITUDES, Plant, Q_NOMINAL, probeMotions } from '../running_example.js';
import { Session, admissible, scene } from './common.js';

const FRACTIONS = { 1: [1.0], 2: [-1.0, 1.0], 4: [-1.0, -0.5, 0.5, 1.0] };
const SWEEP_SCALES = geomspace(0.02, 4.0, 14);

export class OfflineSession extends Session {
    constructor() {
        super();
        this.plant = new Plant({ q: Q_NOMINAL });
        Object.assign(this, { scale: 1.0, noise: 0.0, per_dof: 4, seed: 0 });
        this.model = { bend: 1.0, gravity: 1.0 };
        this._configurationChanged();
    }

    _configurationChanged() {
        this.J = numericalJacobian((q) => this.plant.peek(q), this.plant.q);
        this.estimate = null;
    }

    // Perception with additive Gaussian noise on every feature coordinate.
    _noisyMeasure(rng) {
        return (q) => this.plant.peek(q).map((x) => x + this.noise * rng.standardNormal());
    }

    _estimate(amplitude, seed) {
        const dQ = probeMotions(scale(PROBE_AMPLITUDES, amplitude), FRACTIONS[this.per_dof]);
        const [J0, dS, residual] = offlineEstimate(this._noisyMeasure(new Random(seed)), this.plant.q, dQ);
        return [J0, dQ, dS, residual];
    }

    _relativeError(J_hat) { return normF(subM(J_hat, this.J)) / normF(this.J); }

    state() {
        return { scene: scene(this.plant), J: this.J, estimate: this.estimate,
            params: { scale: this.scale, noise: this.noise, per_dof: this.per_dof },
            model: this._modelEstimate() };
    }

    // J_g = dg/dq of a rod model whose bending stiffness and weight may be wrong.
    _modelEstimate() {
        const rod = new Rod({ kBend: 4.0 * this.model.bend, gravity: 0.45 * this.model.gravity });
        const model = new Plant({ rod, q: this.plant.q });
        const J_g = numericalJacobian((q) => model.peek(q), this.plant.q);
        return { ...this.model, J: J_g, error: this._relativeError(J_g) };
    }

    // ------------------------------------------------------------- actions
    act_move({ q }) {
        this.plant.moveTo(admissible(q));
        this._configurationChanged();
        return this.state();
    }

    act_set_params({ scale: amplitude = null, noise = null, per_dof = null } = {}) {
        if (amplitude !== null) this.scale = Number(amplitude);
        if (noise !== null) this.noise = Number(noise);
        if (per_dof !== null) this.per_dof = Math.trunc(Number(per_dof));
        this.estimate = null;
        return this.state();
    }

    act_set_model({ bend = null, gravity = null } = {}) {
        if (bend !== null) this.model.bend = Number(bend);
        if (gravity !== null) this.model.gravity = Number(gravity);
        return { model: this._modelEstimate() };
    }

    // Run the C exploratory motions and fit J0. Returns each probe for animation.
    act_probe() {
        this.seed += 1;
        const [J0, dQ, dS, residual] = this._estimate(this.scale, this.seed);
        const [sigma] = singularTriplets(J0);
        this.estimate = { J0, dQ, dS, residual, dS_norm: normF(dS), error: this._relativeError(J0),
            condition: conditionNumber(J0), sigma };
        const probes = [];
        for (let c = 0; c < dQ[0].length; c++) {
            const q = add(this.plant.q, column(dQ, c));
            const nodes = this.plant.rod.equilibrium(q, this.plant.nodes);
            probes.push({ q, nodes, points: this.plant.pointsOf(nodes) });
        }
        return { ...this.state(), probes };
    }

    // Residual and estimation error as the probing amplitude varies.
    act_sweep() {
        const ratio = [], error = [];
        for (const amplitude of SWEEP_SCALES) {
            const runs = [101, 202].map((seed) => this._estimate(amplitude, seed));
            ratio.push(mean(runs.map(([, , dS, r]) => r / normF(dS))));
            error.push(mean(runs.map(([J0]) => this._relativeError(J0))));
        }
        return { scales: SWEEP_SCALES, residual_ratio: ratio, error };
    }
}

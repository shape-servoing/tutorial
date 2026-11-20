// Demo 08 - Singular directions of the Jacobian (port of demos/sessions/d07_svd.py).
//
// J_hat = Phi Sigma Psi^T: a unit robot velocity along psi_l produces the
// feature velocity sigma_l phi_l. The page lists the singular triplets of the
// offline estimate (or of the exact Jacobian), plays the robot motion along a
// chosen psi_l on the rod, and shows how the singular values change with the
// units of the robot coordinates and with the configuration.
import { conditionNumber, manipulability, numericalRank, singularTriplets } from '../analysis.js';
import { numericalJacobian, offlineEstimate } from '../jacobian.js';
import { column, linspace, maxAbs } from '../linalg.js';
import { Plant, Q_NOMINAL, probeMotions } from '../running_example.js';
import { Session, admissible, scene } from './common.js';

const DEGREES = [1.0, 1.0, 180.0 / Math.PI, 1.0, 1.0, 180.0 / Math.PI];   // W_q for angles in degrees

export class SvdSession extends Session {
    constructor() {
        super();
        this.plant = new Plant({ q: Q_NOMINAL });
        Object.assign(this, { source: 'estimate', units: 'rad', selected: 5 });
        this._configurationChanged();
    }

    _configurationChanged() {
        const measure = (q) => this.plant.peek(q);
        this.J_rad = this.source === 'estimate'
            ? offlineEstimate(measure, this.plant.q, probeMotions())[0]
            : numericalJacobian(measure, this.plant.q);
    }

    get W_q() { return this.units === 'deg' ? DEGREES : [1, 1, 1, 1, 1, 1]; }

    _scaled() { const W = this.W_q; return this.J_rad.map((row) => row.map((x, j) => x / W[j])); }  // W_s J W_q^{-1}, W_s = I

    state() {
        const J = this._scaled();
        const [sigma, Phi, Psi] = singularTriplets(J);
        return { scene: scene(this.plant), J, sigma, Phi, Psi,
            manipulability: manipulability(J), condition: conditionNumber(J),
            rank: numericalRank(J), selected: this.selected,
            params: { source: this.source, units: this.units } };
    }

    // ------------------------------------------------------------- actions
    act_move({ q }) {
        this.plant.moveTo(admissible(q));
        this._configurationChanged();
        return this.state();
    }

    act_set({ source = null, units = null, selected = null } = {}) {
        if (source !== null && source !== this.source) {
            this.source = source;
            this._configurationChanged();
        }
        if (units !== null) this.units = units;
        if (selected !== null) this.selected = Math.trunc(Number(selected));
        return this.state();
    }

    // Back to the nominal grasp, the offline estimate, radians, and l = 6.
    act_reset() {
        this.plant.reset(Q_NOMINAL);
        Object.assign(this, { source: 'estimate', units: 'rad', selected: 5 });
        this._configurationChanged();
        return this.state();
    }

    // Robot oscillates along psi_l (in physical units); the rod is re-solved each frame.
    act_play({ index, frames = 48, amplitude = 0.3 }) {
        const [sigma, Phi, Psi] = singularTriplets(this._scaled());
        const W = this.W_q;
        const direction = column(Psi, index).map((x, j) => x / W[j]);     // physical motion for unit psi_l
        const scaleF = amplitude / maxAbs(direction);
        const base = this.plant.points;
        const phi = column(Phi, index);
        let nodes = this.plant.nodes;
        const out = [];
        for (const t of linspace(0, 1, frames + 1)) {
            const w = scaleF * Math.sin(2 * Math.PI * t);
            const q = this.plant.q.map((x, j) => x + w * direction[j]);
            nodes = this.plant.rod.equilibrium(q, nodes);
            out.push({ q, nodes, points: this.plant.pointsOf(nodes),
                predicted: base.map((p, i) => [p[0] + w * sigma[index] * phi[2 * i], p[1] + w * sigma[index] * phi[2 * i + 1]]) });
        }
        return { frames: out, index };
    }
}

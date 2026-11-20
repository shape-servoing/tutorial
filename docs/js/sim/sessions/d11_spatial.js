// Demo 12 - Shape control in 3-D (port of demos/sessions/d11_spatial.py).
//
// The same Jacobian/pseudoinverse loop controls a spatial rod, an ARAP surface
// mesh, and an ARAP volumetric lattice. In each case q has three generalized
// actuation coordinates and s stacks selected 3-D vertex positions.
import { EPS, matvec, norm, pinv, scale, sub, svdvals } from '../linalg.js';
import { SpatialModel } from '../spatial.js';
import { Session } from './common.js';

export class SpatialSession extends Session {
    constructor() {
        super();
        this.gain = 0.22;
        this.severity = 1.0;
        this._select('rod');
    }

    _select(kind) {
        this.model = new SpatialModel(kind);
        this.kind = kind;
        this._restart();
    }

    _restart() {
        this.q = [0, 0, 0];
        this.k = 0;
        this.vertices = this.model.deform(this.q);
        this.target_q = scale(this.model.spec.q_target, this.severity);
        this.target = this.model.deform(this.target_q);
        this.initial_error = Math.max(this._error(), 1e-12);
        this.command_norm = 0.0;
        this.command_vector = [0, 0, 0];
        this.J = this.model.jacobian(this.q);
        this._updateJacobianDiagnostics();
    }

    _updateJacobianDiagnostics() {
        this.singular_values = svdvals(this.J);
        const threshold = Math.max(this.J.length, this.J[0].length) * EPS * this.singular_values[0];
        this.rank = this.singular_values.filter((s) => s > threshold).length;
        this.condition = this.singular_values[0] / Math.max(this.singular_values[this.singular_values.length - 1], 1e-12);
    }

    _errorVector() { return sub(this.model.features(this.target), this.model.features(this.vertices)); }

    _error() { return norm(this._errorVector()); }

    state() {
        const spec = this.model.spec;
        return {
            kind: this.kind, label: spec.label, deformation: spec.deformation,
            vertices: this.vertices, target: this.target, rest: spec.rest, faces: spec.faces,
            edges: spec.display_edges, model_edges: spec.model_edges, fixed: spec.fixed, driven: spec.driven,
            fixed_anchor: spec.fixed_anchor, driven_anchor: spec.driven_anchor, grasp_radius: spec.grasp_radius,
            features: spec.features, q: this.q, q_target: this.target_q, q_labels: spec.q_labels,
            jacobian: this.J, singular_values: this.singular_values, rank: this.rank, condition: this.condition,
            k: this.k, error: this._error(), initial_error: this.initial_error,
            command: this.command_norm, command_vector: this.command_vector,
            params: { gain: this.gain, severity: this.severity },
        };
    }

    act_step({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.min(3, Math.max(1, Math.trunc(n))); i++) {
            const error = this._errorVector();
            const J = this.model.jacobian(this.q);
            let command = scale(matvec(pinv(J, 1e-4), error), this.gain);
            const c = norm(command);
            if (c > 0.30) command = scale(command, 0.30 / c);
            this.q = this.q.map((x, j) => x + command[j]);
            this.vertices = this.model.deform(this.q);
            this.J = J;
            this._updateJacobianDiagnostics();
            this.command_vector = Array.from(command);
            this.command_norm = norm(command);
            this.k += 1;
            samples.push({ k: this.k, error: this._error() / this.initial_error, command: this.command_norm,
                command_vector: this.command_vector, q: Array.from(this.q),
                singular_values: Array.from(this.singular_values), condition: this.condition });
        }
        return { ...this.state(), samples };
    }

    act_set_object({ kind }) {
        this._select(kind);
        return this.state();
    }

    act_set_params({ gain = null, severity = null } = {}) {
        if (gain !== null) this.gain = Number(gain);
        if (severity !== null && Math.abs(Number(severity) - this.severity) > 1e-8 + 1e-5 * Math.abs(this.severity)) {
            this.severity = Number(severity);
            this._restart();
        }
        return this.state();
    }

    act_reset() {
        this._restart();
        return this.state();
    }
}

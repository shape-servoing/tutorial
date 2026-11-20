// Demo 06 - Range, null spaces, and residual error (port of demos/sessions/d06_subspaces.py).
//
// Two complementary local decompositions of the deformation Jacobian:
// * point features (n > m): e = e_parallel + e_perp, with the reducible part in
//   Range(J) and the local residual in Null(J^T);
// * two tracked points (n < m): dq = dq_task + dq_null, with task motion in
//   Range(J^T) and robot motion that preserves their positions in Null(J).
import { nullSpace, numericalRank, rangeSplit } from '../analysis.js';
import { numericalJacobian } from '../jacobian.js';
import { add, column, flatten, matvec, maxAbs, norm, pinv, scale, sub, svd } from '../linalg.js';
import { Plant, Q_NOMINAL, Q_REACHABLE, targetFromGrasp, unfeasiblePoints } from '../running_example.js';
import { Session, scene } from './common.js';

const DEFAULTS = { range_weight: 1.0, residual_weight: 1.0, task_weight: 0.75, null_weight: 0.75 };

const scaledStep = (direction, amplitude) => scale(direction, amplitude / Math.max(maxAbs(direction), 1e-12));

export class SubspacesSession extends Session {
    constructor() {
        super();
        this.case = 'error';
        this.params = { ...DEFAULTS };
        this._prepareExamples();
    }

    _prepareExamples() {
        // Overdetermined point-coordinate example: the beyond-limit error of the
        // feasibility demo, split exactly.
        this.pointPlant = new Plant({ q: Q_NOMINAL });
        this.J_points = numericalJacobian((q) => this.pointPlant.peek(q), Q_NOMINAL);
        const [, recorded] = targetFromGrasp(Q_REACHABLE);
        const rawError = sub(flatten(unfeasiblePoints(recorded)), this.pointPlant.s);
        [this.errorRangeBase, this.errorOrthBase] = rangeSplit(this.J_points, rawError);

        // Underdetermined example: two tracked (x, y) points give a full-row-rank
        // 4x6 Jacobian, hence two robot null directions.
        this.motionPlant = new Plant({ nPoints: 2, q: Q_NOMINAL });
        this.J_motion = numericalJacobian((q) => this.motionPlant.peek(q), Q_NOMINAL);
        // The SVD leaves the sign of the task direction and the basis of the
        // two-dimensional null space free: the task motion is the one that lowers
        // the grippers, and the null motion is the part of a left-wrist rotation
        // in Null(J), N N^T e_thetaL (the Python session keeps numpy's choices).
        const { V } = svd(this.J_motion);
        if (numericalRank(this.J_motion) !== 4) throw new Error('expected rank 4');
        const task = column(V, 0);
        this.taskDirection = scaledStep(task[1] + task[4] > 0 ? scale(task, -1) : task, 0.38);
        const N = nullSpace(this.J_motion);
        this.nullDirection = scaledStep(scale(matvec(N, N[2]), -1), 0.25);
    }

    _dimensions(J) {
        const n = J.length, m = J[0].length, rank = numericalRank(J);
        return { n, m, rank, range_dim: rank, null_jt_dim: n - rank, range_jt_dim: rank, null_j_dim: m - rank };
    }

    _errorState() {
        const eRange = scale(this.errorRangeBase, this.params.range_weight);
        const eOrth = scale(this.errorOrthBase, this.params.residual_weight);
        const e = add(eRange, eOrth);
        const targetPoints = this.pointPlant.points.map((p, i) => [p[0] + e[2 * i], p[1] + e[2 * i + 1]]);
        const command = matvec(pinv(this.J_points), e);
        const response = matvec(this.J_points, command);
        const n = Math.max(norm(e), 1e-12);
        return {
            case: 'error', scene: scene(this.pointPlant), target: { nodes: null, points: targetPoints },
            J: this.J_points, e, e_range: eRange, e_orth: eOrth, command, response,
            range_fraction: norm(eRange) / n, orth_fraction: norm(eOrth) / n,
            dimensions: this._dimensions(this.J_points), params: this.params,
        };
    }

    _shapeAt(plant, q) {
        const nodes = plant.rod.equilibrium(q, plant.nodes);
        return { q, nodes, points: plant.pointsOf(nodes) };
    }

    _motionState() {
        const dqTask = scale(this.taskDirection, this.params.task_weight);
        const dqNull = scale(this.nullDirection, this.params.null_weight);
        const task = this._shapeAt(this.motionPlant, add(Q_NOMINAL, dqTask));
        const nullScene = this._shapeAt(this.motionPlant, add(Q_NOMINAL, dqNull));
        const combined = this._shapeAt(this.motionPlant, add(add(Q_NOMINAL, dqTask), dqNull));
        const s0 = this.motionPlant.s;
        return {
            case: 'motion', scene: scene(this.motionPlant), task_scene: task, null_scene: nullScene,
            combined_scene: combined, J: this.J_motion, dq_task: dqTask, dq_null: dqNull,
            linear_task_change: matvec(this.J_motion, dqTask), linear_null_change: matvec(this.J_motion, dqNull),
            task_change: sub(flatten(task.points), s0), null_change: sub(flatten(nullScene.points), s0),
            dimensions: this._dimensions(this.J_motion), params: this.params,
        };
    }

    state() { return this.case === 'error' ? this._errorState() : this._motionState(); }

    act_set_case({ case: value }) {
        if (value !== 'error' && value !== 'motion') throw new Error(`unknown subspace case: ${value}`);
        this.case = value;
        return this.state();
    }

    act_set_params(params = {}) {
        for (const [key, value] of Object.entries(params)) if (key in this.params) this.params[key] = Number(value);
        return this.state();
    }

    act_reset() {
        Object.assign(this.params, DEFAULTS);
        return this.state();
    }
}

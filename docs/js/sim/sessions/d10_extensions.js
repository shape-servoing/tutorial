// Demo 11 - Extensions of the shape-servoing loop (port of demos/sessions/d10_extensions.py).
//
//     tracking    a time-varying target s*(t), recorded from a moving grasp; the
//                 feedforward term J_hat^+ ds*/dt is added to the control law
//     secondary   with turning angles (n = 3 < m = 6) the shape task leaves robot
//                 directions unused: a secondary task (left gripper to a marker) is
//                 pursued within Null(J_hat), or added unprojected for comparison
//     combined    turning angles unloop a looped rod, and a point-coordinate task,
//                 projected onto Null(J_hat) of the turning angles, transports it to
//                 the target pose. Only the point Jacobian is estimated; the
//                 turning-angle Jacobian follows from it by the chain rule.
import { FeatureTask, ShapeServoingLoop } from '../control.js';
import { featureVector } from '../features.js';
import { offlineEstimate } from '../jacobian.js';
import { norm, sub, zeros } from '../linalg.js';
import { Plant, Q_ARCH, Q_STRAIGHT_MOVED, probeMotions, ringPlant, targetFromGrasp } from '../running_example.js';
import { Session, scene } from './common.js';

const PERIOD = 120;                       // iterations per target oscillation
export const SECONDARY_GAIN = 0.05;

// Turning angles regulate the shape, and a point-coordinate task the pose. The
// point Jacobian J_p_hat is estimated offline and updated with Broyden's rule;
// the turning-angle Jacobian is J_hat = (d kappa / d s_p) J_p_hat. Returns
// [loop, task] (assign the task to loop.secondary to transport the rod).
export function combinedLoop(plant, angleTarget, pointTarget) {
    const task = new FeatureTask(plant, 'points', pointTarget, { gain: 0.10 });
    const loop = new ShapeServoingLoop(plant, angleTarget, task.chainedJacobian('angles'), { alpha: 0.12, damping: 0.05 });
    loop.deriveJacobianFrom(task);
    return [loop, task];
}

// Grasp that records the moving target: the arch sways and tilts.
export function movingGrasp(k) {
    const phase = 2 * Math.PI * k / PERIOD;
    const dx = 0.8 * Math.sin(phase), dtheta = 0.25 * Math.sin(phase);
    return Q_ARCH.map((x, i) => x + [dx, 0, dtheta, dx, 0, dtheta][i]);
}

export class ExtensionsSession extends Session {
    constructor({ secondaryGain = SECONDARY_GAIN } = {}) {
        super();
        Object.assign(this, { alpha: 0.15, feedforward: true, project: true, transport: true, secondaryGain });
        this._resetTracking();
        this._resetSecondary();
        this._resetCombined();
    }

    state() {
        return { tracking: this._trackingState(), secondary: this._secondaryState(), combined: this._combinedState() };
    }

    // ------------------------------------------------------------- tracking
    _resetTracking() {
        this.k = 0;
        this.target_plant = new Plant({ q: movingGrasp(0) });
        this.plant = new Plant({ q: movingGrasp(0) });
        const [J0] = offlineEstimate((q) => this.plant.peek(q), this.plant.q, probeMotions());
        this.loop = new ShapeServoingLoop(this.plant, this.target_plant.s, J0, { alpha: this.alpha });
    }

    _trackingState() {
        return { scene: scene(this.plant), k: this.k,
            target: { nodes: this.target_plant.nodes, points: this.target_plant.points },
            error: norm(this.loop.error), params: { alpha: this.alpha, feedforward: this.feedforward } };
    }

    act_track({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.max(1, Math.trunc(n)); i++) {
            const sNow = this.target_plant.s;
            const sNext = this.target_plant.moveTo(movingGrasp(this.k + 1));
            this.loop.sStar = sNow;                                                   // e_k = s*_k - s_k
            const rate = this.feedforward ? sub(sNext, sNow).map((x) => x / this.loop.dt) : null;
            this.loop.step(rate);
            this.loop.sStar = sNext;
            this.k += 1;
            samples.push({ k: this.k, error: norm(this.loop.error) });
        }
        return { tracking: this._trackingState(), samples };
    }

    act_set_tracking({ alpha = null, feedforward = null } = {}) {
        if (alpha !== null) this.alpha = Number(alpha);
        if (feedforward !== null) this.feedforward = Boolean(feedforward);
        this.loop.alpha = this.alpha;
        return { tracking: this._trackingState() };
    }

    act_reset_tracking() {
        this._resetTracking();
        return { tracking: this._trackingState() };
    }

    // ------------------------------------------------------------- secondary task
    _resetSecondary() {
        this.shape_plant = new Plant({ featureMap: 'angles', q: Q_ARCH });
        const [J0] = offlineEstimate((q) => this.shape_plant.peek(q), Q_ARCH, probeMotions());
        this.shape_loop = new ShapeServoingLoop(this.shape_plant, this.shape_plant.s, J0, { alpha: 0.1 });
        this.shape_loop.secondary = (q) => this._secondaryCommand(q);
        this.shape_loop.projectSecondary = this.project;
        this.marker = [-2.6, 2.4];
        this.secondary_k = 0;
    }

    // u2 moves the left gripper toward the marker (positions only).
    _secondaryCommand(q) {
        const u2 = zeros(6);
        u2[0] = this.secondaryGain * (this.marker[0] - q[0]);
        u2[1] = this.secondaryGain * (this.marker[1] - q[1]);
        return u2;
    }

    _distance() { return norm([this.marker[0] - this.shape_plant.q[0], this.marker[1] - this.shape_plant.q[1]]); }

    _secondaryState() {
        return { scene: scene(this.shape_plant), k: this.secondary_k, marker: this.marker,
            shape_error: norm(this.shape_loop.error), distance: this._distance(), project: this.project };
    }

    act_secondary({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.max(1, Math.trunc(n)); i++) {
            this.shape_loop.step();
            this.secondary_k += 1;
            samples.push({ k: this.secondary_k, shape_error: norm(this.shape_loop.error), distance: this._distance() });
        }
        return { secondary: this._secondaryState(), samples };
    }

    act_set_marker({ xy }) {
        this.marker = [Math.min(-0.4, Math.max(-4.4, Number(xy[0]))), Math.min(3.8, Math.max(-1.0, Number(xy[1])))];
        return { secondary: this._secondaryState() };
    }

    act_set_projection({ project }) {
        this.project = Boolean(project);
        this.shape_loop.projectSecondary = this.project;
        return { secondary: this._secondaryState() };
    }

    act_reset_secondary() {
        this._resetSecondary();
        return { secondary: this._secondaryState() };
    }

    // ------------------------------------------------------------- combined feature maps
    _resetCombined() {
        [this.target_nodes, this.target_points] = targetFromGrasp(Q_STRAIGHT_MOVED);
        this.angle_target = featureVector(this.target_points, 'angles');
        this.point_target = featureVector(this.target_points, 'points');
        this.ring = ringPlant('angles');
        [this.ring_loop, this.transport_task] = combinedLoop(this.ring, this.angle_target, this.point_target);
        this.ring_loop.secondary = this.transport ? this.transport_task : null;
        this.combined_k = 0;
    }

    _combinedState() {
        const points = featureVector(this.ring.points, 'points');
        return { scene: scene(this.ring), k: this.combined_k, transport: this.transport,
            target: { nodes: this.target_nodes, points: this.target_points },
            angle_error: norm(this.ring_loop.error), point_error: norm(sub(this.point_target, points)),
            strain: Math.max(...this.ring.rod.axialStrain(this.ring.nodes)) };
    }

    act_combined({ n = 1 } = {}) {
        const samples = [];
        for (let i = 0; i < Math.max(1, Math.trunc(n)); i++) {
            this.ring_loop.step();             // updates the point estimate, also without transport
            this.combined_k += 1;
            const state = this._combinedState();
            samples.push({ k: this.combined_k, angle_error: state.angle_error,
                point_error: state.point_error, strain: state.strain });
        }
        return { combined: this._combinedState(), samples };
    }

    act_set_transport({ transport }) {
        this.transport = Boolean(transport);
        this.ring_loop.secondary = this.transport ? this.transport_task : null;
        return { combined: this._combinedState() };
    }

    act_reset_combined() {
        this._resetCombined();
        return { combined: this._combinedState() };
    }
}

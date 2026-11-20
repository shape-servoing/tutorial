// The demos, in the order of the tutorial (port of demos/catalog.py): each
// entry names its session class (sessions/), its page files (lessons/<id>.html,
// js/demos/<id>.js), and a small preview drawn on the menu card.
import { rangeSplit, singularTriplets } from './analysis.js';
import { ShapeServoingLoop } from './control.js';
import { stretchedGrasp } from './failures.js';
import { numericalJacobian, offlineEstimate } from './jacobian.js';
import { add, column, copyM, linspace, sub } from './linalg.js';
import { Plant, Q_ARCH, Q_NOMINAL, Q_REACHABLE, probeMotions, targetFromGrasp, unfeasiblePoints } from './running_example.js';
import { ProblemSession } from './sessions/d01_problem.js';
import { JacobianSession } from './sessions/d02_jacobian.js';
import { OfflineSession } from './sessions/d03_offline.js';
import { LoopSession } from './sessions/d04_loop.js';
import { TargetsSession } from './sessions/d05_targets.js';
import { StabilitySession } from './sessions/d06_stability.js';
import { SubspacesSession } from './sessions/d06_subspaces.js';
import { SvdSession } from './sessions/d07_svd.js';
import { FeaturesSession } from './sessions/d08_features.js';
import { FailuresSession } from './sessions/d09_failures.js';
import { ExtensionsSession } from './sessions/d10_extensions.js';
import { SpatialSession } from './sessions/d11_spatial.js';

const entry = (id, part, number, title, summary, concepts, session) => ({ id, part, number, title, summary, concepts, session });

export const ENTRIES = [
    entry('problem', 1, 1, 'The shape-servoing problem',
        'Move the grippers by hand and observe the features $\\vs$, the target $\\vs^{*}$, '
        + 'and the shape error $\\ve=\\vs^{*}-\\vs$.',
        ['feature map', 'shape error', 'quasistatic equilibrium'], ProblemSession),
    entry('jacobian', 1, 2, 'The deformation Jacobian',
        'Apply a small robot motion and compare the feature change predicted '
        + 'by $\\vJ$ with the measured one.',
        ['ds/dt = J u', 'columns of J', 'locality'], JacobianSession),
    entry('offline', 1, 3, 'Offline estimation by regression',
        'Probe the rod with 24 small motions and fit the initial estimate '
        + 'from the action-response pairs.',
        ['probing', 'least squares', 'residual'], OfflineSession),
    entry('loop', 1, 4, 'Closing the loop',
        'Run Algorithm 1 step by step: control law, motion, measurement, '
        + 'and the online Broyden update.',
        ['pseudoinverse law', 'Broyden update', 'gain'], LoopSession),
    entry('targets', 1, 5, 'Target feasibility and reachability',
        'Separate global feasibility and reachability, then compare a recorded '
        + 'reachable target with a synthetic target beyond deformation limits.',
        ['feasible set', 'admissible path', 'infeasible target'], TargetsSession),
    entry('subspaces', 1, 6, 'Range, null spaces, and residual error',
        'Using only point coordinates, decompose feature error and expose '
        + 'gripper motions that preserve two tracked point positions.',
        ['Range(J)', 'Null(J^T)', 'Null(J)'], SubspacesSession),
    entry('stability', 1, 7, 'Local stability and convergence',
        'Perturb the Jacobian estimate and check whether its commands still '
        + 'reduce the shape error.',
        ['Lyapunov candidate', 'Sym(J_hat^+ J)', 'corrective directions'], StabilitySession),
    entry('svd', 2, 8, 'Singular value analysis of the deformation Jacobian',
        'Move the robot along each direction $\\vpsi_l$ and compare the feature '
        + 'change with its prediction $\\sigma_l\\vphi_l$.',
        ['SVD', 'manipulability', 'condition number'], SvdSession),
    entry('features', 2, 9, 'Choosing the shape features',
        'Compare point coordinates, edge vectors, and curvature-based features: which '
        + 'motions change them, what the controller can regulate, and how much '
        + 'each stretches a looped rod.',
        ['invariance', 'null space', 'deformation path'], FeaturesSession),
    entry('failures', 2, 10, 'Failure modes of the shape-servoing loop',
        'Pull the rod taut, move it in ways the features do not register, '
        + 'and compress it until it buckles.',
        ['controllability', 'observability', 'mechanical instability'], FailuresSession),
    entry('extensions', 2, 11, 'Extensions of the shape-servoing loop',
        'Track a moving target with feedforward, pursue a secondary task '
        + 'in the null space of the estimate, and combine two feature '
        + 'representations to unloop a rod and move it to a target pose.',
        ['feedforward', 'null-space projection', 'two feature representations'], ExtensionsSession),
    entry('spatial', 2, 12, 'Three-dimensional objects',
        'Apply the same shape-servoing loop to a spatial rod and to a sheet '
        + 'and a solid that deform as-rigid-as-possible (ARAP).',
        ['3-D features', 'ARAP', 'surface and volume meshes'], SpatialSession),
];

export const CATALOG = Object.fromEntries(ENTRIES.map((e) => [e.id, e]));

const pairs = (a, b) => a.map((p, i) => [p, b[i]]);

// Menu previews: a few rods, points, a target, and arrows.
export function preview(demoId) {
    const plant = new Plant({ q: Q_NOMINAL });
    const [archNodes, archPoints, archS] = targetFromGrasp(Q_ARCH);
    const view = { rods: [plant.nodes], points: plant.points, target: null, arrows: [] };
    if (['problem', 'loop', 'stability'].includes(demoId)) {
        view.target = { nodes: archNodes, points: archPoints };
        view.arrows = pairs(plant.points, archPoints);
    }
    if (demoId === 'loop') {
        const [J0] = offlineEstimate((q) => plant.peek(q), Q_NOMINAL, probeMotions());
        const loop = new ShapeServoingLoop(plant, archS, J0);
        view.rods = [];
        for (let k = 0; k < 141; k++) {
            if ([0, 6, 16, 140].includes(k)) view.rods.push(copyM(plant.nodes));
            loop.step();
        }
        view.points = plant.points;
        view.arrows = [];
    } else if (demoId === 'jacobian' || demoId === 'svd') {
        const J = numericalJacobian((q) => plant.peek(q), plant.q);
        const [, Phi] = singularTriplets(J);
        const c = demoId === 'jacobian' ? column(J, 1) : column(Phi, Phi[0].length - 1);
        view.arrows = plant.points.map((p, i) => [p, [p[0] + 1.2 * c[2 * i], p[1] + 1.2 * c[2 * i + 1]]]);
    } else if (demoId === 'offline') {
        view.rods = [plant.nodes, ...[[0, 0.35, 0, 0, 0, 0], [0, -0.35, 0, 0, 0, 0]].map((d) => new Plant({ q: add(Q_NOMINAL, d) }).nodes)];
    } else if (demoId === 'targets') {
        const [nodes, points] = targetFromGrasp(Q_REACHABLE);
        view.target = { nodes: null, points: unfeasiblePoints(points) };
        view.rods = [nodes];
        view.points = points;
        view.arrows = pairs(points, unfeasiblePoints(points));
    } else if (demoId === 'subspaces') {
        const [, points] = targetFromGrasp(Q_REACHABLE);
        const target = unfeasiblePoints(points);
        const J = numericalJacobian((q) => plant.peek(q), plant.q);
        const [eRange] = rangeSplit(J, sub(target.flat(), plant.s));
        const projected = plant.points.map((p, i) => [p[0] + eRange[2 * i], p[1] + eRange[2 * i + 1]]);
        view.target = { nodes: null, points: target };
        view.arrows = pairs(plant.points, projected);
    } else if (demoId === 'features') {
        view.rods = [archNodes];
        view.points = archPoints;
        view.arrows = pairs(archPoints.slice(0, -1), archPoints.slice(1));
    } else if (demoId === 'failures') {
        view.rods = [6.6, 5.7, 5.0].map((d) => new Plant({ q: stretchedGrasp(d) }).nodes);
        view.points = [];
    } else if (demoId === 'extensions') {
        view.rods = [-0.8, 0.0, 0.8].map((dx) => new Plant({ q: add(Q_ARCH, [dx, 0, 0, dx, 0, 0]) }).nodes);
        view.points = [];
    } else if (demoId === 'spatial') {
        const t = linspace(0, 1, 17);
        const rest = t.map((x) => [6.0 * (x - 0.5), 0.4]);
        const target = t.map((x) => [6.0 * (x - 0.5), 0.4 + 1.55 * Math.sin(Math.PI * x) + 0.225 * Math.sin(2 * Math.PI * x)]);
        const every2 = (a) => a.slice(1, -1).filter((_, i) => i % 2 === 0);
        view.rods = [rest, target];
        view.points = every2(target);
        view.target = { nodes: target, points: null };
        view.arrows = pairs(every2(rest), every2(target));
    }
    return view;
}

// The menu entries without their session classes, with previews.
export const catalogWithPreviews = () => ENTRIES.map((e) => ({
    id: e.id, part: e.part, number: e.number, title: e.title, summary: e.summary,
    concepts: e.concepts, preview: preview(e.id),
}));


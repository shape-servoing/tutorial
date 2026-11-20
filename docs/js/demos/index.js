// The page module of every demo, by id (the shell imports them statically, so
// that each page bundles into one script).
import * as extensions from './extensions.js';
import * as failures from './failures.js';
import * as features from './features.js';
import * as jacobian from './jacobian.js';
import * as loop from './loop.js';
import * as offline from './offline.js';
import * as problem from './problem.js';
import * as spatial from './spatial.js';
import * as stability from './stability.js';
import * as subspaces from './subspaces.js';
import * as svd from './svd.js';
import * as targets from './targets.js';

export const DEMOS = { problem, jacobian, offline, loop, targets, subspaces, stability, svd, features, failures, extensions, spatial };

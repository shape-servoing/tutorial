// Shape features: the feature map s = f(p) (port of shape_servoing/features.py).
//
//     point coordinates  s = [p_1, ..., p_P]           n = 2P      extrinsic
//     edge vectors       d_i = p_{i+1} - p_i           n = 2(P-1)  translation-invariant
//     turning angles     kappa_i = angle(d_i, d_{i+1}) n = P-2     rigid-motion-invariant
// Points are arrays of [x, y] pairs.
import { mean, zerosM } from './linalg.js';

export const pointCoordinates = (points) => points.flatMap((p) => [p[0], p[1]]);
export const edgeVectors = (points) => points.slice(1).flatMap((p, i) => [p[0] - points[i][0], p[1] - points[i][1]]);
export function turningAngles(points) {
    const d = points.slice(1).map((p, i) => [p[0] - points[i][0], p[1] - points[i][1]]);
    return d.slice(0, -1).map((a, i) => {
        const b = d[i + 1];
        return Math.atan2(a[0] * b[1] - a[1] * b[0], a[0] * b[0] + a[1] * b[1]);
    });
}

export const FEATURE_MAPS = { points: pointCoordinates, edges: edgeVectors, angles: turningAngles };
export const FEATURE_NAMES = Object.keys(FEATURE_MAPS);

export const featureVector = (points, featureMap = 'points') => FEATURE_MAPS[featureMap](points);

// Derivative ds/ds_p of a feature map with respect to the point coordinates s_p.
export function featureMapJacobian(points, featureMap = 'points') {
    const P = points.length;
    if (featureMap === 'points') return zerosM(2 * P, 2 * P).map((row, i) => { row[i] = 1; return row; });
    if (featureMap === 'edges') {
        const D = zerosM(2 * (P - 1), 2 * P);
        for (let i = 0; i < P - 1; i++) {
            for (let d = 0; d < 2; d++) { D[2 * i + d][2 * i + d] = -1; D[2 * i + d][2 * i + 2 + d] = 1; }
        }
        return D;
    }
    // kappa_i = angle(d_{i+1}) - angle(d_i), with d(angle(d))/dd = perp(d) / |d|^2
    const g = points.slice(1).map((p, i) => {
        const d = [p[0] - points[i][0], p[1] - points[i][1]];
        const d2 = d[0] * d[0] + d[1] * d[1];
        return [-d[1] / d2, d[0] / d2];
    });
    const D = zerosM(P - 2, 2 * P);
    for (let i = 0; i < P - 2; i++) {
        for (let k = 0; k < 2; k++) {
            D[i][2 * i + k] = g[i][k];
            D[i][2 * i + 2 + k] = -g[i][k] - g[i + 1][k];
            D[i][2 * i + 4 + k] = g[i + 1][k];
        }
    }
    return D;
}

export const featureDimension = (nPoints, featureMap) => ({ points: 2 * nPoints, edges: 2 * (nPoints - 1), angles: nPoints - 2 })[featureMap];

export const shapeError = (sStar, s) => sStar.map((x, i) => x - s[i]);

// Rotate the points about their centroid by `angle` [rad], then translate.
export function rigidMotion(points, angle = 0, translation = [0, 0]) {
    const cx = mean(points.map((p) => p[0])), cy = mean(points.map((p) => p[1]));
    const c = Math.cos(angle), s = Math.sin(angle);
    return points.map(([x, y]) => [c * (x - cx) - s * (y - cy) + cx + translation[0],
        s * (x - cx) + c * (y - cy) + cy + translation[1]]);
}

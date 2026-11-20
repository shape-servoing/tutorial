// Level medals. The medal body is the "dignity" silhouette, the target shape
// of a clay-sculpting shape-comparison demo, with the same path data: a
// head-and-neck contour that the sculptor has to
// carve out of a block. Level 1 (fundamentals) is silver, level 2 (advanced
// concepts) gold.

// Path data of the sculpting target, in its original SVG coordinates.
export const DIGNITY_PATH = 'm 760.7495,2617.5699 c 0.38246,3.9549 5.10303,31.8314 10.64711,41.2593 '
    + '5.54386,9.4279 18.93547,21.0188 25.06522,21.5951 6.12975,0.5762 31.22287,-7.0403 40.34127,-13.3083 '
    + '9.1184,-6.2682 16.19732,-12.3718 15.11094,-18.5306 -1.08638,-6.1587 -18.28883,-24.1643 '
    + '-14.36233,-38.7585 3.92628,-14.5942 20.67115,-39.3838 19.50855,-52.6751 -1.16261,-13.2913 '
    + '-14.26695,-29.8428 -33.73125,-36.6606 -19.46408,-6.8176 -28.33331,-9.0766 -39.55608,-8.7065 '
    + '-11.22256,0.3701 -36.90156,16.8381 -39.55349,23.3579 -2.65193,6.5198 -14.26589,33.9295 '
    + '-12.47558,42.2921 1.79052,8.3624 20.07543,32.1702 24.54446,36.5093 4.46881,4.3391 4.46118,3.6259 '
    + '4.46118,3.6259 z';

// Geometry of the path (its bounding box starts at X0, Y0; size W x H; area
// centroid at CX, CY in the shifted frame).
const X0 = 731.573, Y0 = 2511.749, W = 125.53, H = 168.70, CX = 66.0, CY = 78.2;

const METAL = {
    1: { name: 'silver', stops: ['#ffffff', '#dfe4e8', '#9ea8b1', '#5f6973'], rim: '#4f5861', ink: '#3f474f' },
    2: { name: 'gold', stops: ['#fff1b8', '#f2c230', '#c98a12', '#87570a'], rim: '#6e4507', ink: '#5a3804' },
};
const RIBBON = {
    1: { band: '#527563', stripe: '#eaf1ed', edge: '#3e624f' },     // level 1 accent
    2: { band: '#8d6b43', stripe: '#f2eadc', edge: '#6f5232' },     // level 2 accent
};

let counter = 0;

// SVG markup of the medal for `level` (1 or 2). `size` is the rendered height in px.
export function medalSVG(level, { size = 220, title = true } = {}) {
    const m = METAL[level] || METAL[1], r = RIBBON[level] || RIBBON[1];
    const id = `medal${level}-${++counter}`;
    const shape = `<path d="${DIGNITY_PATH}" transform="translate(${-X0} ${-Y0})"/>`;
    const digit = String(level), name = level === 2 ? 'ADVANCED' : 'FUNDAMENTALS';
    // The longer name gets a smaller type, so that it stays inside the silhouette.
    const nameStyle = name.length > 9 ? 'font-size="7.6" letter-spacing="0.7"' : 'font-size="9.5" letter-spacing="1.5"';
    // View box: the shape (0..W, 0..H) plus margins and the ribbon above it.
    const vb = { x: -18, y: -70, w: W + 36, h: H + 84 };
    const width = Math.round(size * vb.w / vb.h);
    // Ribbon: two crossed straps that meet at a ring on top of the head.
    const top = 9;
    const strap = (x1, x2, x3, x4, fill) =>
        `<polygon points="${x1},${vb.y + 4} ${x2},${vb.y + 4} ${x3},${top} ${x4},${top + 4}" fill="${fill}"/>`;
    return `
<svg class="medal-svg" xmlns="http://www.w3.org/2000/svg" viewBox="${vb.x} ${vb.y} ${vb.w} ${vb.h}"
     width="${width}" height="${size}" role="img" aria-label="Level ${level} medal">
  ${title ? `<title>Level ${level} medal</title>` : ''}
  <defs>
    <radialGradient id="${id}-metal" cx="0.32" cy="0.24" r="0.82">
      <stop offset="0" stop-color="${m.stops[0]}"/>
      <stop offset="0.28" stop-color="${m.stops[1]}"/>
      <stop offset="0.72" stop-color="${m.stops[2]}"/>
      <stop offset="1" stop-color="${m.stops[3]}"/>
    </radialGradient>
    <linearGradient id="${id}-inner" x1="0" y1="0" x2="1" y2="1">
      <stop offset="0" stop-color="${m.stops[2]}" stop-opacity="0.55"/>
      <stop offset="1" stop-color="${m.stops[0]}" stop-opacity="0.35"/>
    </linearGradient>
    <clipPath id="${id}-clip">${shape}</clipPath>
    <filter id="${id}-shadow" x="-30%" y="-30%" width="160%" height="160%">
      <feDropShadow dx="0" dy="4" stdDeviation="4" flood-color="#000" flood-opacity="0.28"/>
    </filter>
  </defs>
  <g class="medal-ribbon">
    ${strap(CX - 44, CX - 18, CX + 8, CX - 10, r.band)}
    ${strap(CX + 44, CX + 18, CX - 8, CX + 10, r.edge)}
    <polygon points="${CX - 36},${vb.y + 4} ${CX - 28},${vb.y + 4} ${CX + 2},${top} ${CX - 4},${top + 1}" fill="${r.stripe}" opacity="0.85"/>
    <polygon points="${CX + 36},${vb.y + 4} ${CX + 28},${vb.y + 4} ${CX - 2},${top} ${CX + 4},${top + 1}" fill="${r.stripe}" opacity="0.6"/>
    <circle cx="${CX}" cy="${top + 1}" r="7.5" fill="url(#${id}-metal)" stroke="${m.rim}" stroke-width="2"/>
  </g>
  <g filter="url(#${id}-shadow)">
    <g fill="url(#${id}-metal)" stroke="${m.rim}" stroke-width="3.2" stroke-linejoin="round">${shape}</g>
  </g>
  <g clip-path="url(#${id}-clip)">
    <g transform="translate(${CX} ${CY}) scale(0.84) translate(${-CX} ${-CY})"
       fill="none" stroke="${m.rim}" stroke-width="1.8" stroke-opacity="0.6">${shape}</g>
    <g transform="translate(${CX} ${CY}) scale(0.86) translate(${-CX} ${-CY})"
       fill="none" stroke="#ffffff" stroke-width="1.2" stroke-opacity="0.55">${shape}</g>
    <ellipse cx="38" cy="34" rx="34" ry="15" transform="rotate(-28 38 34)" fill="#ffffff" opacity="0.32"/>
  </g>
  <g fill="${m.ink}" text-anchor="middle" font-family="Georgia, 'Times New Roman', serif">
    <text x="${CX}" y="56" font-size="12" font-weight="700" letter-spacing="3">LEVEL</text>
    <text x="${CX + 2}" y="104" font-size="44" font-weight="700">${digit}</text>
    <text x="${CX + 4}" y="124" ${nameStyle} font-weight="700">${name}</text>
  </g>
</svg>`;
}

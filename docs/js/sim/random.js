// Seeded random numbers for the demos (uniform and standard normal). The Python
// sessions use numpy's generator with the same seeds, so random targets, probe
// noise, and random perturbations differ between the two; every other number
// of the demos is the same.

export class Random {
    constructor(seed = 0) {
        // sfc32, seeded by splitmix32 of the integer seed.
        let x = (seed >>> 0) + 0x9e3779b9;
        const next = () => {
            x = (x + 0x9e3779b9) >>> 0;
            let z = x;
            z = Math.imul(z ^ (z >>> 16), 0x85ebca6b) >>> 0;
            z = Math.imul(z ^ (z >>> 13), 0xc2b2ae35) >>> 0;
            return (z ^ (z >>> 16)) >>> 0;
        };
        this.state = [next(), next(), next(), next()];
        for (let i = 0; i < 12; i++) this.random();
        this.spare = null;
    }

    // Uniform in [0, 1).
    random() {
        let [a, b, c, d] = this.state;
        const t = (((a + b) >>> 0) + d) >>> 0;
        d = (d + 1) >>> 0;
        a = b ^ (b >>> 9);
        b = (c + (c << 3)) >>> 0;
        c = ((c << 21) | (c >>> 11)) >>> 0;
        c = (c + t) >>> 0;
        this.state = [a >>> 0, b, c, d];
        return t / 4294967296;
    }

    uniform(low = 0, high = 1) { return low + (high - low) * this.random(); }

    standardNormal() {
        if (this.spare !== null) { const z = this.spare; this.spare = null; return z; }
        let u = 0;
        while (u === 0) u = this.random();
        const v = this.random();
        const r = Math.sqrt(-2 * Math.log(u));
        this.spare = r * Math.sin(2 * Math.PI * v);
        return r * Math.cos(2 * Math.PI * v);
    }

    normals(n) { return Array.from({ length: n }, () => this.standardNormal()); }
    normalMatrix(rows, cols) { return Array.from({ length: rows }, () => this.normals(cols)); }
}

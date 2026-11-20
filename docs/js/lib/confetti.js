// A short confetti burst on a full-window canvas (no dependencies). Pieces use
// the entity colours of the paper figures: rod blue, tracked-point yellow,
// gripper violet and orange, aqua, and red.

const COLORS = ['#2a78d6', '#eda100', '#4a3aa7', '#eb6834', '#1baf7a', '#e34948', '#f2c230'];

export function confetti({ count = 190, duration = 3600 } = {}) {
    if (window.matchMedia('(prefers-reduced-motion: reduce)').matches) return Promise.resolve();
    const canvas = document.createElement('canvas');
    canvas.className = 'confetti-canvas';
    canvas.setAttribute('aria-hidden', 'true');
    document.body.append(canvas);
    const ctx = canvas.getContext('2d');
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const resize = () => {
        canvas.width = Math.round(window.innerWidth * dpr);
        canvas.height = Math.round(window.innerHeight * dpr);
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    };
    resize();
    window.addEventListener('resize', resize);

    const w = window.innerWidth, h = window.innerHeight;
    const pieces = [];
    // Two "party poppers" from the lower corners, plus a light rain from the top.
    for (let i = 0; i < count; i += 1) {
        const kind = i % 5;
        let x, y, vx, vy;
        if (kind < 2) {
            const left = kind === 0;
            x = left ? -10 : w + 10;
            y = h * (0.75 + 0.2 * Math.random());
            const tilt = 0.25 + 0.55 * Math.random();          // from the vertical, toward the centre
            const speed = 13 + 9 * Math.random();
            vx = (left ? 1 : -1) * Math.sin(tilt) * speed;
            vy = -Math.cos(tilt) * speed - 4;
        } else {
            x = Math.random() * w;
            y = -20 - Math.random() * h * 0.4;
            vx = (Math.random() - 0.5) * 3;
            vy = 1 + Math.random() * 3;
        }
        pieces.push({
            x, y, vx, vy,
            size: 6 + Math.random() * 7,
            ratio: 0.35 + Math.random() * 0.5,
            color: COLORS[Math.floor(Math.random() * COLORS.length)],
            angle: Math.random() * Math.PI * 2,
            spin: (Math.random() - 0.5) * 0.35,
            wobble: Math.random() * Math.PI * 2,
            round: Math.random() < 0.25,
        });
    }

    return new Promise((resolve) => {
        const start = performance.now();
        let last = start;
        function frame(now) {
            const dt = Math.min(2.5, (now - last) / 16.67);
            last = now;
            const t = now - start;
            const fade = t > duration - 700 ? Math.max(0, (duration - t) / 700) : 1;
            ctx.clearRect(0, 0, w, h);
            for (const p of pieces) {
                p.vy += 0.32 * dt;
                p.vx *= Math.pow(0.985, dt);
                p.vy *= Math.pow(0.985, dt);
                p.wobble += 0.12 * dt;
                p.x += (p.vx + Math.sin(p.wobble) * 0.8) * dt;
                p.y += p.vy * dt;
                p.angle += p.spin * dt;
                ctx.save();
                ctx.globalAlpha = fade;
                ctx.translate(p.x, p.y);
                ctx.rotate(p.angle);
                ctx.scale(1, Math.abs(Math.cos(p.wobble)) * 0.8 + 0.2);
                ctx.fillStyle = p.color;
                if (p.round) { ctx.beginPath(); ctx.arc(0, 0, p.size * 0.4, 0, Math.PI * 2); ctx.fill(); }
                else ctx.fillRect(-p.size / 2, -p.size * p.ratio / 2, p.size, p.size * p.ratio);
                ctx.restore();
            }
            if (t < duration) requestAnimationFrame(frame);
            else {
                window.removeEventListener('resize', resize);
                canvas.remove();
                resolve();
            }
        }
        requestAnimationFrame(frame);
    });
}

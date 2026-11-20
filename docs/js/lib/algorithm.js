// Algorithm 1 of the tutorial, line by line, with the active line highlighted.
import { h, label } from './ui.js';

export const ALGORITHM_1 = {
    caption: '<b>Algorithm 1</b> Online Jacobian-based shape servoing',
    require: 'Input: target $\\vs^{*}$, initial estimate $\\vJh_0$, gain $\\alpha$, period $\\Delta t$',
    lines: [
        { text: 'measure $\\vy_0$ and compute $\\vs_0=f(\\vy_0)$; $k \\gets 0$' },
        { text: '<b>while</b> true <b>do</b>' },
        { text: '$\\ve_k \\gets \\vs^{*}-\\vs_k$', indent: 1 },
        { text: '$\\vu_k \\gets \\alpha\\,\\vJh_k^{+}\\ve_k$', indent: 1, comment: 'control law' },
        { text: 'apply $\\vu_k$ for $\\Delta t$; $\\dq_k \\gets \\vu_k\\Delta t$', indent: 1 },
        { text: 'measure $\\vy_{k+1}$ and compute $\\vs_{k+1}=f(\\vy_{k+1})$', indent: 1 },
        { text: '$\\ds_{k+1} \\gets \\vs_{k+1}-\\vs_k$', indent: 1 },
        { text: '$\\vJh_{k+1} \\gets$ update of $\\vJh_k$ from $(\\dq_k,\\ds_{k+1})$', indent: 1, comment: 'Broyden' },
        { text: '$k \\gets k+1$', indent: 1 },
        { text: '<b>end while</b>' },
    ],
};

export class AlgorithmView {
    constructor(spec = ALGORITHM_1) {
        this.rows = [];
        this.el = h('div', { class: 'algorithm' },
            h('div', { class: 'algorithm-caption', html: spec.caption }),
            h('div', { class: 'algorithm-line' }, h('span', { class: 'num' }, ''), rich(spec.require)));
        spec.lines.forEach((line, i) => {
            const body = h('span', { class: line.indent ? `indent-${line.indent}` : '' }, rich(line.text),
                line.comment ? h('span', { class: 'comment' }, `  ▷ ${line.comment}`) : null);
            const row = h('div', { class: 'algorithm-line' }, h('span', { class: 'num' }, `${i + 1}:`), body);
            this.rows.push(row);
            this.el.append(row);
        });
    }

    // Highlight line n (1-based); 0 clears.
    highlight(n) {
        this.rows.forEach((row, i) => row.classList.toggle('active', i + 1 === n));
    }
}

// Text with <b> tags and $maths$.
function rich(text) {
    const span = h('span');
    text.split(/(<b>.*?<\/b>)/).forEach((part) => {
        if (part.startsWith('<b>')) span.append(h('b', {}, part.slice(3, -4)));
        else if (part) span.append(label(part));
    });
    return span;
}

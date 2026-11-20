// Equations with KaTeX (vendored, offline), using the notation macros of the
// tutorial's LaTeX preamble so the lessons can quote it verbatim.

export const MACROS = {
    '\\R': '\\mathbb{R}',
    '\\norm': '\\lVert #1 \\rVert',
    '\\vs': '\\mathbf{s}', '\\ve': '\\mathbf{e}', '\\vx': '\\mathbf{x}', '\\vy': '\\mathbf{y}',
    '\\vp': '\\mathbf{p}', '\\vv': '\\mathbf{v}', '\\vJ': '\\mathbf{J}', '\\vJh': '\\hat{\\mathbf{J}}',
    '\\vq': '\\mathbf{q}', '\\vr': '\\mathbf{r}', '\\vu': '\\mathbf{u}', '\\vd': '\\mathbf{d}',
    '\\dq': '\\Delta\\mathbf{q}', '\\dQ': '\\Delta\\mathbf{Q}', '\\dS': '\\Delta\\mathbf{S}',
    '\\ds': '\\Delta\\mathbf{s}', '\\vphi': '\\boldsymbol{\\phi}', '\\vpsi': '\\boldsymbol{\\psi}',
    '\\vg': '\\mathbf{g}', '\\vW': '\\mathbf{W}', '\\vI': '\\mathbf{I}',
    '\\Range': '\\operatorname{Range}', '\\Null': '\\operatorname{Null}',
    '\\Sym': '\\operatorname{Sym}', '\\cond': '\\operatorname{cond}',
};

export function tex(source, display = false) {
    return window.katex.renderToString(source, {
        displayMode: display, macros: { ...MACROS }, throwOnError: false, strict: 'ignore',
    });
}

// Render every <span class="tex"> (inline) and <div class="tex-block"> (display)
// below root, once.
export function renderMath(root = document) {
    for (const el of root.querySelectorAll('.tex, .tex-block')) {
        if (el.dataset.rendered) continue;
        el.innerHTML = tex(el.textContent, el.classList.contains('tex-block'));
        el.dataset.rendered = '1';
    }
}

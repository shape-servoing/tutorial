// The session of a demo page: the JavaScript port of the Python sessions
// (js/sim/sessions, checked against demos/sessions by tests/test_web_parity.py)
// runs in the page itself, so the demos need no server. A call returns a promise,
// as a request to a server would, and runs after the browser has handled pending
// input, so that drags and sliders stay responsive.
import { CATALOG, ENTRIES, preview } from '../sim/catalog.js';
import { toast } from './ui.js';

// Results are copied, as a server's JSON answers would be: the page may keep them
// while the session changes its own arrays, and non-finite numbers become null.
function plain(value) {
    if (typeof value === 'number') return Number.isFinite(value) ? value : null;
    if (Array.isArray(value) || ArrayBuffer.isView(value)) return Array.from(value, plain);
    if (value && typeof value === 'object') {
        const out = {};
        for (const [key, v] of Object.entries(value)) if (v !== undefined && typeof v !== 'function') out[key] = plain(v);
        return out;
    }
    return value === undefined ? null : value;
}

const nextTask = () => new Promise((resolve) => { setTimeout(resolve, 0); });

function report(error) {
    console.error(error);
    toast(`${error.name || 'Error'}: ${error.message}`);
}

function run(fn) {
    try {
        return plain(fn());
    } catch (error) {
        report(error);
        throw error;
    }
}

export class Session {
    static async start(demoId) {
        await nextTask();
        let impl;
        try { impl = new CATALOG[demoId].session(); } catch (error) { report(error); throw error; }
        return { session: new Session(impl), state: run(() => impl.state()) };
    }

    constructor(impl) { this.impl = impl; }

    async call(action, payload = {}) {
        await nextTask();
        return run(() => this.impl.dispatch(action, payload ?? {}));
    }

    // For continuous input (drags, sliders): at most one call in flight; while it
    // runs, only the newest payload is kept. onResult gets each answer.
    stream(action, onResult) {
        let busy = false;
        let pending = null;
        const send = async (payload) => {
            busy = true;
            try { onResult(await this.call(action, payload)); } catch { /* reported by run */ }
            busy = false;
            if (pending) { const next = pending; pending = null; send(next); }
        };
        return (payload) => { if (busy) pending = payload; else send(payload); };
    }
}

// The demos in tutorial order; with `previews`, each with the drawing of its menu card.
export async function catalog({ previews = false } = {}) {
    return ENTRIES.map((e) => ({
        id: e.id, part: e.part, number: e.number, title: e.title, summary: e.summary,
        concepts: e.concepts, ...(previews ? { preview: plain(preview(e.id)) } : {}),
    }));
}

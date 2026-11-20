// Demo page: lesson on the left (lessons/<id>.html), interactive workspace on
// the right (js/demos/<id>.js, backed by the session of js/sim/sessions). The
// built page contains every lesson (window.SHAPE_SERVOING_LESSONS, written by
// tools/build_web.py); fetching the lesson file is only the fallback.
import { DEMOS } from './demos/index.js';
import { catalog, Session } from './lib/api.js';
import { syncPlayer } from './lib/frame.js';
import { renderMath } from './lib/math.js';
import { initPage } from './lib/page.js';
import { celebrateCompletedParts, markVisited } from './lib/rewards.js';
import { initTools } from './lib/tools.js';
import { label } from './lib/ui.js';

await initPage({ math: false });

const demos = await catalog();
const requested = new URLSearchParams(location.search).get('id');
const index = Math.max(0, demos.findIndex((d) => d.id === requested));
const demo = demos[index];
const prev = demos[index - 1], next = demos[index + 1];

document.title = `${demo.title} · Shape Servoing Demos`;
// Level 2 pages use their own palette (css: [data-part="2"]; `part` is the level).
document.documentElement.dataset.part = String(demo.part);
const partName = demo.part === 1 ? 'Level 1 · Fundamentals' : 'Level 2 · Advanced concepts';
const partTag = `<span class="part-tag">Level ${demo.part}</span>`;
document.getElementById('topbarTitle').innerHTML = `${partTag}${demo.part === 1 ? 'Fundamentals' : 'Advanced concepts'} · <strong>${String(demo.number).padStart(2, '0')} ${demo.title}</strong>`;
document.getElementById('lessonKicker').textContent = `${partName} · Demo ${String(demo.number).padStart(2, '0')}`;
document.getElementById('lessonTitle').textContent = demo.title;
document.getElementById('lessonIntro').replaceChildren(label(demo.summary));

for (const [id, target, text] of [['prevDemo', prev, '◀ Previous'], ['prevLesson', prev, '◀ Previous'],
    ['nextDemo', next, 'Next ▶'], ['nextLesson', next, 'Next demo ▶']]) {
    const link = document.getElementById(id);
    if (target) { link.href = `demo.html?id=${target.id}`; link.title = target.title; link.textContent = text; }
    else if (id.startsWith('next')) { link.href = 'index.html'; link.textContent = 'Back to all demos'; }
    else link.style.visibility = 'hidden';
}
// After the demo links exist, so that teaching mode (?present) carries over to them.
initTools(document.getElementById('toolsButton'), { demo, workspace: document.getElementById('workspace') });
// The address and the title of the player follow this demo.
syncPlayer();

markVisited(demo.id, demos);
// A part's medal is given once the user has left the demo that completed it;
// it pops up shortly after the page opens, while the demo itself loads.
setTimeout(() => celebrateCompletedParts(demos, demo.id), 500);

const lessonBody = document.getElementById('lessonBody');
const lessons = window.SHAPE_SERVOING_LESSONS;
lessonBody.innerHTML = lessons?.[demo.id] ?? await (await fetch(`lessons/${demo.id}.html`)).text();
renderMath(document);

const { session, state } = await Session.start(demo.id);
await DEMOS[demo.id].mount({ workspace: document.getElementById('workspace'), lesson: lessonBody, session, state, demo });

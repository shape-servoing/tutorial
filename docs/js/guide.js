// The user guide, the glossary, and the useful definitions: static pages with
// the same top bar as the menu.
import { initPage } from './lib/page.js';
import { initTools } from './lib/tools.js';

await initPage();
initTools(document.getElementById('toolsButton'));

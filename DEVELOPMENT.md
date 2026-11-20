# Editing and building the demo pages

## Structure

```
demos/                  Python reference of the demo sessions (catalog.py, sessions/)
docs/                   the interactive demos (one self-contained html file per page) and their sources:
                        css, lessons, js (pages), js/lib, js/demos, js/sim (JavaScript port), img, audio, vendor
tools/build_web.py      writes the style sheets, scripts, lessons, and figures into docs/*.html
run.py                  local server for docs/ (--sandbox, --tests)
```

The pages run a JavaScript port of the Python package (`docs/js/sim`). Each Python module has a counterpart of the same name in `docs/js/sim` (and `docs/js/sim/sessions` for `demos/sessions`), with the functions in camelCase. `demos/` holds the Python reference of the demo sessions (the state and actions behind each page), which the port reproduces; [TESTS.md](TESTS.md) describes how the two are compared.

## How the pages are built

Each page is a single HTML file that contains its style sheets, the KaTeX fonts, KaTeX, its script (the page, the demos, and the simulation), the lessons, and its figures, so that a page view is one request (the music loads its files only when played). The pages therefore also work on hosts that serve every page in a sandboxed origin without CORS headers and limit the number of requests, such as the anonymized pages of anonymous.4open.science (175 requests per 15 minutes).

The sources of the pages are the HTML outside the blocks marked `data-inline`, `docs/css/app.css`, the lessons in `docs/lessons/`, and the ES modules in `docs/js` (page scripts), `docs/js/lib` (drawing, plots, widgets, theme, music, player), `docs/js/demos` (one module per demo), and `docs/js/sim` (the JavaScript port of `shape_servoing` and of the demo sessions). After editing them, fill the `data-inline` blocks again (Node.js is needed; `npx` fetches esbuild):

```bash
python tools/build_web.py     # writes the style sheets, scripts, lessons, and figures into docs/*.html
python run.py --sandbox       # check the pages as anonymous.4open.science serves them
```

`tests/test_web_build.py` fails while a page is out of date with its sources.

## Hosting

The pages work on any static host. On GitHub Pages: *Settings → Pages → Deploy from a branch*, branch `main`, folder `/docs`.

The pages normally open inside `docs/player.html`, which keeps the music playing while the demos change in a frame, and keeps the address bar on the current demo; add `?standalone` to an address to open a page on its own. Hosts with sandboxed origins refuse to show a page inside another one and give the pages no storage. There, the pages open on their own, the links between them carry the theme, the palette, the demos visited, and the music that is playing (`?st=`, `?music=`), and *Save figure* and *Record animation* show the result in a dialog, to be saved with the browser's context menu, because downloads are blocked. `python run.py --sandbox` serves the pages in the same way, to check them locally.

`run.py` options: `--port 9000`, `--no-browser`, `--sandbox`, and `--tests`.

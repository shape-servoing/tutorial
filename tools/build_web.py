"""Make every page in docs/ a single, self-contained HTML file.

The pages are written as ES modules (docs/js: the page scripts, docs/js/lib, the
demos in docs/js/demos, and the simulation in docs/js/sim, which also runs under
Node.js in tests/web_parity.mjs), a style sheet (docs/css/app.css), the lessons
(docs/lessons/*.html), and KaTeX. The anonymized pages of anonymous.4open.science
are served in a sandboxed origin without CORS headers, where ES modules, font
files, and fetch() requests fail, and that host allows each visitor 175 requests
per 15 minutes. The build therefore writes into each page everything it loads. In
docs/*.html, the blocks

    <style data-inline="css/app.css"></style>
    <style data-inline="vendor/katex/katex.min.css"></style>   (with the KaTeX fonts)
    <script data-inline="vendor/katex/katex.min.js"></script>
    <script data-inline="lessons"></script>                    (as window.SHAPE_SERVOING_LESSONS)
    <script data-inline="js/shell.js"></script>                (the page and its imports, esbuild)

and the attributes src="..." data-inline="img/fig1_running_example.png" (a data
URI) are filled again on every run; edit the rest of the HTML as usual. A page
view is then one request (the music loads its files only when played), and the
pages also open directly from the disk.

Run after editing docs/js, docs/css, docs/lessons, or docs/img (needs Node.js;
npx fetches esbuild):

    python tools/build_web.py

tests/test_web_build.py checks that the pages match their sources.
"""
from __future__ import annotations

import base64
import hashlib
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Callable

ROOT = Path(__file__).resolve().parent.parent
DOCS = ROOT / "docs"
ESBUILD = "esbuild@0.24.0"
PAGES = ("index.html", "demo.html", "guide.html", "glossary.html", "definitions.html", "player.html")
ENTRIES = ("js/menu.js", "js/shell.js", "js/guide.js", "js/player.js")
# Every module a page can import (the digest of these files is written in each bundle).
SOURCES = ("js", "js/lib", "js/demos", "js/sim", "js/sim/sessions")
BLOCK = re.compile(r'<(style|script) data-inline="([^"]+)">(.*?)</\1>', re.DOTALL)
ATTRIBUTE = re.compile(r'\b(src|href)="[^"]*"(\s+)data-inline="([^"]+)"')
MIME = {".jpg": "image/jpeg", ".png": "image/png", ".svg": "image/svg+xml"}


def read(path: str) -> str:
    return (DOCS / path).read_text(encoding="utf-8").replace("\r\n", "\n")


def sources_digest() -> str:
    """Digest of the module sources, independent of the line endings of the checkout."""
    digest = hashlib.sha256()
    for folder in SOURCES:
        for path in sorted((DOCS / folder).glob("*.js")):
            digest.update(f"{folder}/{path.name}\n".encode())
            digest.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return digest.hexdigest()[:16]


def bundle_header(entry: str, digest: str) -> str:
    return f"/* docs/{entry} and the modules it imports, bundled by tools/build_web.py (sources {digest}) */\n"


def bundle(entry: str, digest: str) -> str:
    npx = shutil.which("npx")
    if npx is None:
        sys.exit("npx (Node.js) is needed to bundle the pages")
    result = subprocess.run([npx, "--yes", ESBUILD, f"docs/{entry}", "--bundle", "--format=esm", "--minify",
                             "--platform=browser", "--target=es2020", "--supported:top-level-await=true",
                             "--log-level=warning"],
                            cwd=ROOT, capture_output=True, text=True, encoding="utf-8")
    if result.returncode != 0:
        sys.exit(result.stderr)
    code = result.stdout
    if re.search(r"(^|[;}])(import|export)[\s{*]", code, re.MULTILINE):
        sys.exit(f"{entry}: the bundle still contains import/export statements")
    # The pages use top-level await: the bundle runs in an async function, as a classic script.
    return bundle_header(entry, digest) + "(async () => {\n" + code.strip("\n") + "\n})();"


def katex_css() -> str:
    css = read("vendor/katex/katex.min.css")

    def embed(match: re.Match) -> str:
        data = base64.b64encode((DOCS / "vendor" / "katex" / "fonts" / f"{match[1]}.woff2").read_bytes()).decode("ascii")
        return f'url(data:font/woff2;base64,{data}) format("woff2")'

    css = re.sub(r'url\(fonts/(KaTeX_[\w-]+)\.woff2\) format\("woff2"\)'
                 r'(?:,url\(fonts/\1\.woff\) format\("woff"\))?(?:,url\(fonts/\1\.ttf\) format\("truetype"\))?',
                 embed, css)
    if "url(fonts/" in css:
        sys.exit("katex.min.css: a font was not embedded")
    return "/* KaTeX (MIT license), vendor/katex/katex.min.css with the fonts of vendor/katex/fonts */\n" + css


def lessons() -> str:
    """The lessons of all demos, as window.SHAPE_SERVOING_LESSONS = {id: html}."""
    texts = {path.stem: read(f"lessons/{path.name}") for path in sorted((DOCS / "lessons").glob("*.html"))}
    # "<" escaped, so that no lesson can end the script block.
    data = json.dumps(texts, ensure_ascii=False, indent=0).replace("<", "\\u003c")
    return "/* docs/lessons/*.html */\nwindow.SHAPE_SERVOING_LESSONS = " + data + ";"


def block_content(path: str, bundles: Callable[[str], str]) -> str:
    if path == "vendor/katex/katex.min.css":
        return katex_css()
    if path == "lessons":
        return lessons()
    if path in ENTRIES:
        return bundles(path)
    return read(path)


def data_uri(path: str) -> str:
    return f"data:{MIME[Path(path).suffix]};base64," + base64.b64encode((DOCS / path).read_bytes()).decode("ascii")


def render(html: str, bundles: Callable[[str], str]) -> str:
    """The page with its inline blocks and attributes filled from their sources."""
    def block(match: re.Match) -> str:
        tag, path = match[1], match[2]
        content = block_content(path, bundles).strip("\n")
        for closing in ("</script", "</style", "<!--"):
            if closing in content.lower():
                sys.exit(f"{path}: contains {closing}, which would end the inline block")
        return f'<{tag} data-inline="{path}">\n{content}\n</{tag}>'

    html = BLOCK.sub(block, html)
    return ATTRIBUTE.sub(lambda m: f'{m[1]}="{data_uri(m[3])}"{m[2]}data-inline="{m[3]}"', html)


def main() -> None:
    digest = sources_digest()
    cache: dict[str, str] = {}

    def bundles(entry: str) -> str:
        if entry not in cache:
            cache[entry] = bundle(entry, digest)
        return cache[entry]

    for name in PAGES:
        path = DOCS / name
        html = render(path.read_text(encoding="utf-8").replace("\r\n", "\n"), bundles)
        with open(path, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(html)
        print(f"written: docs/{name} ({path.stat().st_size / 1024:.0f} kB)")


if __name__ == "__main__":
    main()

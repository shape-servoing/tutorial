"""
Open the interactive demos locally (Python standard library only).

    python run.py                 serve docs/ at http://127.0.0.1:8765 and open the browser
    python run.py --port 9000     another port (a free port is used if it is taken)
    python run.py --no-browser
    python run.py --sandbox       serve the pages as anonymous.4open.science does: every document
                                  in a sandboxed origin, without CORS headers, not shown in frames
    python run.py --tests         create .venv (first run), install requirements, run the tests

The demos are static files that run in the browser; any web server for the
docs/ folder works as well (e.g., GitHub Pages), and docs/index.html also opens
directly from the disk.
"""
from __future__ import annotations

import argparse
import functools
import hashlib
import http.server
import os
import socket
import subprocess
import sys
import threading
import venv
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DOCS = ROOT / "docs"
VENV = ROOT / ".venv"
REQUIREMENTS = ROOT / "requirements.txt"
STAMP = VENV / ".requirements.sha256"


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map,
                      ".js": "text/javascript", ".mjs": "text/javascript", ".json": "application/json",
                      ".woff2": "font/woff2", ".mp3": "audio/mpeg", ".md": "text/plain; charset=utf-8"}
    sandbox = False

    def end_headers(self) -> None:
        if self.sandbox:
            self.send_header("Content-Security-Policy", "sandbox allow-scripts allow-popups allow-forms allow-modals")
            self.send_header("X-Frame-Options", "SAMEORIGIN")
        super().end_headers()

    def log_message(self, *args) -> None:      # quiet console
        pass


def free_port(preferred: int) -> int:
    with socket.socket() as s:
        try:
            s.bind(("127.0.0.1", preferred))
            return preferred
        except OSError:
            s.bind(("127.0.0.1", 0))
            return s.getsockname()[1]


def venv_python() -> Path:
    return VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def run_tests() -> int:
    """Create .venv if needed, (re)install the requirements when they change, run the tests."""
    python = venv_python()
    if not python.exists():
        print("Creating the virtual environment in .venv ...")
        venv.EnvBuilder(with_pip=True).create(VENV)
    digest = hashlib.sha256(REQUIREMENTS.read_bytes()).hexdigest()
    if not STAMP.exists() or STAMP.read_text().strip() != digest:
        print("Installing requirements ...")
        subprocess.check_call([str(python), "-m", "pip", "install", "--disable-pip-version-check",
                               "-q", "-r", str(REQUIREMENTS)])
        STAMP.write_text(digest)
    return subprocess.call([str(python), "-m", "unittest", "discover", "-s", "tests", "-v"], cwd=ROOT)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--sandbox", action="store_true", help="serve the pages as anonymous.4open.science does")
    parser.add_argument("--tests", action="store_true", help="run the test suite instead of serving the demos")
    args = parser.parse_args()
    if args.tests:
        return run_tests()
    port = free_port(args.port)
    handler = type("SandboxHandler", (Handler,), {"sandbox": True}) if args.sandbox else Handler
    server = http.server.ThreadingHTTPServer(("127.0.0.1", port), functools.partial(handler, directory=str(DOCS)))
    url = f"http://127.0.0.1:{port}/"
    print(f"\n  Shape servoing tutorial - interactive demos\n  Serving {DOCS} at {url}\n  Press Ctrl+C to stop.\n")
    if not args.no_browser:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("  Stopped.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

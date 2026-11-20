#!/usr/bin/env sh
# Linux / macOS launcher: serves the demos (docs/) and opens them in the browser; --tests runs the tests.
cd "$(dirname "$0")" || exit 1
exec python3 run.py "$@"

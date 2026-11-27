#!/bin/sh
# Builds the Python package: dist/motionstudio-<version>-py3-none-any.whl and
# dist/motionstudio-<version>.tar.gz, checked with twine. Upload them with
#   <this script's venv>/bin/twine upload dist/*      (asks for a PyPI API token)
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=${1:-$here/dist}
venv=${MOTIONSTUDIO_BUILD_VENV:-$here/.build-venv}
[ -x "$venv/bin/python" ] || python3 -m venv "$venv"
"$venv/bin/python" -m pip install -q --upgrade build twine
rm -rf "$out"
"$venv/bin/python" -m build --outdir "$out" "$here"
rm -rf "$here/build" "$here/motionstudio.egg-info"
"$venv/bin/twine" check "$out"/*
ls -1 "$out"

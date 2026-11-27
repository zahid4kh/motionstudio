#!/bin/sh
# Makes the source release of the C++ library: dist/motionstudio-cpp-<version>.tar.gz
# (and .zip), with its SHA-256 for FetchContent's URL_HASH. Attach them to a release.
set -e
here=$(cd "$(dirname "$0")" && pwd)
version=$(sed -n 's/^project(motionstudio_cpp VERSION \([0-9.]*\).*/\1/p' "$here/CMakeLists.txt")
name=motionstudio-cpp-$version
out=${1:-$here/dist}
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
mkdir -p "$work/$name" "$out"
cd "$here"
cp -r CMakeLists.txt LICENSE README.md cmake include src examples "$work/$name/"
mkdir -p "$work/$name/tests" && cp tests/live_test.cpp "$work/$name/tests/"
cd "$work"
tar czf "$out/$name.tar.gz" "$name"
rm -f "$out/$name.zip" && zip -qr "$out/$name.zip" "$name"
cd "$out"
sha256sum "$name.tar.gz" "$name.zip" > "$name.sha256"
cat "$name.sha256"

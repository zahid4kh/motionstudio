#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"
MUJOCO_LIB="${HOME}/mujoco-3.10.0/lib"

echo "==> Configuring..."
cmake -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release -S "$SCRIPT_DIR" -DFETCHCONTENT_QUIET=ON

echo "==> Building..."
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "==> Running MotionStudio..."
LD_LIBRARY_PATH="$MUJOCO_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
    "$BUILD_DIR/MotionStudio" "$@"

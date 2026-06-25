#!/bin/bash

# Build MotionStudio and create a Debian package (motionstd)
# Usage: ./build-deb.sh [version] [architecture]

set -e

APP_NAME="motionstd"
VERSION="${1:-0.1.0}"
ARCHITECTURE="${2:-amd64}"
MUJOCO_ROOT="${MUJOCO_ROOT:-$HOME/mujoco-3.10.0}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
STAGING_DIR="$SCRIPT_DIR/staging_${APP_NAME}"
DEB_FILE="$SCRIPT_DIR/${APP_NAME}_${VERSION}_${ARCHITECTURE}.deb"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

die() {
    echo -e "${RED}Error: $1${NC}" >&2
    exit 1
}

echo -e "${BLUE}==> Building ${APP_NAME} v${VERSION} (${ARCHITECTURE})${NC}"

[ -d "$MUJOCO_ROOT/lib" ] || die "MuJoCo not found at $MUJOCO_ROOT (set MUJOCO_ROOT if needed)"
[ -f "$PROJECT_ROOT/icon-1024x1024.png" ] || die "App icon not found at project root"

echo -e "${BLUE}==> Configuring and compiling MotionStudio...${NC}"
cmake -B "$BUILD_DIR" -S "$PROJECT_ROOT" \
    -DCMAKE_BUILD_TYPE=Release \
    -DMUJOCO_ROOT="$MUJOCO_ROOT" \
    -DFETCHCONTENT_QUIET=ON

cmake --build "$BUILD_DIR" -j"$(nproc)"

[ -f "$BUILD_DIR/MotionStudio" ] || die "Build output not found at $BUILD_DIR/MotionStudio"

echo -e "${BLUE}==> Generating Debian staging directory...${NC}"
"$SCRIPT_DIR/debian-staging-script" "$VERSION" "$ARCHITECTURE"

echo -e "${BLUE}==> Installing binaries into staging...${NC}"
install -m 755 "$BUILD_DIR/MotionStudio" "$STAGING_DIR/usr/lib/$APP_NAME/$APP_NAME"
install -m 755 "$MUJOCO_ROOT/lib/libmujoco.so.3.10.0" "$STAGING_DIR/usr/lib/$APP_NAME/"
ln -sf libmujoco.so.3.10.0 "$STAGING_DIR/usr/lib/$APP_NAME/libmujoco.so"

mkdir -p "$STAGING_DIR/usr/share/mime/packages"

cat > "$STAGING_DIR/usr/share/mime/packages/$APP_NAME.xml" << 'EOF'
<?xml version="1.0" encoding="UTF-8"?>
<mime-info xmlns="http://www.freedesktop.org/standards/shared-mime-info">
  <mime-type type="application/x-motionstudio-project">
    <comment>MotionStudio project</comment>
    <glob pattern="*.mstudio"/>
  </mime-type>
</mime-info>
EOF

echo -e "${BLUE}==> Building Debian package...${NC}"
dpkg-deb --build --root-owner-group "$STAGING_DIR" "$DEB_FILE"

echo -e "${GREEN}Package built: $DEB_FILE${NC}"
echo -e "${GREEN}Install with: sudo dpkg -i $DEB_FILE${NC}"
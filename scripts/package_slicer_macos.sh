#!/usr/bin/env bash
# =============================================================================
# package_slicer_macos.sh
#
# Builds 3D Slicer from source on macOS inside a standard prefix (/opt/slicer)
# and packages it into a compressed archive (.tar.zst) for use in macOS CI.
#
# This mirrors the Linux CI Docker workflow (docker/Dockerfile & docker/build-and-push.sh):
#   - Prefix:      /opt/slicer (clean, standard, identical across macOS & Linux)
#   - Source:      /opt/slicer/Slicer
#   - Superbuild:  /opt/slicer/Slicer-SuperBuild
#   - Slicer_DIR:  /opt/slicer/Slicer-SuperBuild/Slicer-build
#
# Prerequisites (Homebrew):
#   brew install cmake ninja zstd qt@6
#
# Usage:
#   # Full workflow (setup /opt/slicer, clone, compile, clean, package):
#   ./scripts/package_slicer_macos.sh
#
#   # Build a specific branch/tag (default: main):
#   ./scripts/package_slicer_macos.sh --slicer-tag main
#   ./scripts/package_slicer_macos.sh --slicer-tag v5.12.0
#
#   # Skip compilation and package an existing /opt/slicer build:
#   ./scripts/package_slicer_macos.sh --package-only
#
#   # Compile only without creating a tarball:
#   ./scripts/package_slicer_macos.sh --build-only
#
#   # Package and upload to GitHub Releases:
#   ./scripts/package_slicer_macos.sh --package-only --upload
#
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# -----------------------------------------------------------------------------
# Defaults
# -----------------------------------------------------------------------------
PREFIX="/opt/slicer"
SLICER_REPO="https://github.com/Slicer/Slicer.git"
SLICER_TAG="main"
RELEASE_TAG="ci-assets"
GH_REPO="rosmed/slicer_ros2_module"
FORMAT="zstd"
DEPLOYMENT_TARGET="14.0"

DO_CLONE=true
DO_BUILD=true
DO_PACKAGE=true
DO_UPLOAD=false
DRY_RUN=false
ASSUME_YES=false

# -----------------------------------------------------------------------------
# Help
# -----------------------------------------------------------------------------
usage() {
  cat <<EOF
Usage: $(basename "$0") [OPTIONS]

Workflow Options:
  --build-only           Clone and build Slicer, but do not create the tarball
  --package-only         Skip clone and build; clean and package existing build
  --upload               Upload generated archive to GitHub Releases via 'gh'
  --slicer-tag <TAG>     Slicer git tag or branch to build (default: ${SLICER_TAG})
  --release-tag <TAG>    GitHub Release tag for upload (default: ${RELEASE_TAG})
  --prefix <DIR>         Base installation prefix (default: ${PREFIX})
  --format <zstd|gzip>   Archive compression format (default: ${FORMAT})
  -o, --output <FILE>    Custom path for output archive
  -j, --jobs <N>         Number of parallel build jobs (default: all CPU cores)
  -y, --yes              Assume yes for confirmation prompts
  -d, --dry-run          Show actions without cloning, compiling, or modifying files
  -h, --help             Show this help message

Examples:
  $(basename "$0")                           # Full clone, build, and package
  $(basename "$0") --slicer-tag v5.12.0      # Build specific tag
  $(basename "$0") --package-only            # Package existing /opt/slicer build
  $(basename "$0") --package-only --upload   # Package and upload to GitHub Releases
EOF
}

# -----------------------------------------------------------------------------
# Parse Arguments
# -----------------------------------------------------------------------------
NPROC="$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"
OUTPUT_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --build-only)
      DO_PACKAGE=false; shift ;;
    --package-only)
      DO_CLONE=false; DO_BUILD=false; shift ;;
    --upload)
      DO_UPLOAD=true; shift ;;
    --slicer-tag)
      SLICER_TAG="$2"; shift 2 ;;
    --release-tag)
      RELEASE_TAG="$2"; shift 2 ;;
    --prefix)
      PREFIX="$2"; shift 2 ;;
    --format)
      FORMAT="$2"; shift 2 ;;
    -o|--output)
      OUTPUT_FILE="$2"; shift 2 ;;
    -j|--jobs)
      NPROC="$2"; shift 2 ;;
    -y|--yes)
      ASSUME_YES=true; shift ;;
    -d|--dry-run)
      DRY_RUN=true; shift ;;
    -h|--help)
      usage; exit 0 ;;
    *)
      echo "Error: Unknown option '$1'" >&2
      usage >&2
      exit 1 ;;
  esac
done

SRC_DIR="${PREFIX}/Slicer"
SUPERBUILD_DIR="${PREFIX}/Slicer-SuperBuild"
SLICER_BUILD_DIR="${SUPERBUILD_DIR}/Slicer-build"
ARCH="$(uname -m)"
OS="$(uname -s)"

if [ "$OS" != "Darwin" ]; then
  echo "Warning: This script is intended for macOS (Darwin). Current OS: $OS" >&2
fi

# Determine archive filename if not specified
if [ "$FORMAT" = "zstd" ]; then
  ARCHIVE_EXT="tar.zst"
elif [ "$FORMAT" = "gzip" ]; then
  ARCHIVE_EXT="tar.gz"
else
  echo "Error: Unsupported compression format '$FORMAT'. Use 'zstd' or 'gzip'." >&2
  exit 1
fi

if [ -z "$OUTPUT_FILE" ]; then
  OUTPUT_FILE="${REPO_ROOT}/slicer-${SLICER_TAG}-macos-${ARCH}.${ARCHIVE_EXT}"
fi

echo "============================================================"
echo "  Slicer macOS CI Builder & Packager"
echo "============================================================"
echo "  Prefix Directory : ${PREFIX}"
echo "  Source Directory : ${SRC_DIR}"
echo "  Build Directory  : ${SUPERBUILD_DIR}"
echo "  Slicer_DIR       : ${SLICER_BUILD_DIR}"
echo "  Target Tag       : ${SLICER_TAG}"
echo "  Architecture     : ${ARCH} (${OS})"
echo "  Parallel Jobs    : ${NPROC}"
echo "  Output Archive   : ${OUTPUT_FILE}"
echo "  Actions          : clone=$DO_CLONE, build=$DO_BUILD, package=$DO_PACKAGE, upload=$DO_UPLOAD"
echo "============================================================"

# -----------------------------------------------------------------------------
# Check Prerequisites
# -----------------------------------------------------------------------------
check_tool() {
  local tool="$1"
  local install_cmd="$2"
  if ! command -v "$tool" >/dev/null 2>&1; then
    echo "Error: Required tool '$tool' not found." >&2
    echo "       Install it using: $install_cmd" >&2
    exit 1
  fi
}

check_tool "cmake" "brew install cmake"
check_tool "ninja" "brew install ninja"
check_tool "git" "xcode-select --install"

if [ "$DO_PACKAGE" = true ]; then
  if [ "$FORMAT" = "zstd" ] && ! command -v zstd >/dev/null 2>&1; then
    echo "Warning: 'zstd' not found. Falling back to gzip." >&2
    echo "         (Run 'brew install zstd' for faster, smaller compression)" >&2
    FORMAT="gzip"
    ARCHIVE_EXT="tar.gz"
    OUTPUT_FILE="${OUTPUT_FILE%.tar.zst}.tar.gz"
  fi
fi

if [ "$DO_UPLOAD" = true ]; then
  check_tool "gh" "brew install gh"
fi

# Locate Homebrew Qt
QT_DIR=""
QT_VERSION=""
for formula in qt qt@6 qt@5; do
  QT_CANDIDATE="$(brew --prefix "$formula" 2>/dev/null || true)"
  if [ -n "$QT_CANDIDATE" ] && [ -d "$QT_CANDIDATE/lib/cmake" ]; then
    if [ -d "$QT_CANDIDATE/lib/cmake/Qt6" ]; then
      QT_DIR="$QT_CANDIDATE/lib/cmake/Qt6"
      QT_VERSION="$("$QT_CANDIDATE/bin/qmake" -query QT_VERSION 2>/dev/null || echo "6.11")"
      break
    elif [ -d "$QT_CANDIDATE/lib/cmake/Qt5" ]; then
      QT_DIR="$QT_CANDIDATE/lib/cmake/Qt5"
      QT_VERSION="$("$QT_CANDIDATE/bin/qmake" -query QT_VERSION 2>/dev/null || echo "5.15")"
      break
    fi
  fi
done

if [ -z "$QT_DIR" ] && [ "$DO_BUILD" = true ]; then
  echo "Error: Could not locate Qt6 or Qt5 CMake directory via Homebrew." >&2
  echo "       Please run: brew install qt" >&2
  exit 1
fi

echo "  Detected Qt      : ${QT_DIR} (${QT_VERSION})"

if [ "$DRY_RUN" = true ]; then
  echo ""
  echo "Dry-run mode enabled. Exiting before modifying any files."
  exit 0
fi

# -----------------------------------------------------------------------------
# 1. Setup /opt/slicer Directory Permissions & Check Disk Space
# -----------------------------------------------------------------------------
if [ ! -d "$PREFIX" ]; then
  echo ""
  echo "==> Creating prefix directory '$PREFIX'..."
  if ! sudo mkdir -p "$PREFIX" 2>/dev/null; then
    echo "Error: Cannot create '$PREFIX' without sudo password." >&2
    echo "       Please run this command once in your terminal:" >&2
    echo "         sudo mkdir -p '$PREFIX' && sudo chown -R \$(whoami) '$PREFIX'" >&2
    exit 1
  fi
  sudo chown -R "$(id -u):$(id -g)" "$PREFIX"
elif [ ! -w "$PREFIX" ]; then
  echo ""
  echo "==> Ensuring write permissions on '$PREFIX'..."
  if ! sudo chown -R "$(id -u):$(id -g)" "$PREFIX" 2>/dev/null; then
    echo "Error: '$PREFIX' is not writable without sudo password." >&2
    echo "       Please run this command once in your terminal:" >&2
    echo "         sudo chown -R \$(whoami) '$PREFIX'" >&2
    exit 1
  fi
fi

# Check available disk space
if [ "$DO_BUILD" = true ]; then
  CHECK_PATH="$PREFIX"
  [ ! -d "$CHECK_PATH" ] && CHECK_PATH="$(dirname "$PREFIX")"
  AVAIL_KB="$(df -k "$CHECK_PATH" | awk 'NR==2 {print $4}')"
  AVAIL_GB=$(( AVAIL_KB / 1024 / 1024 ))
  if [ "$AVAIL_GB" -lt 25 ]; then
    echo "Warning: Only ${AVAIL_GB} GB of free disk space available." >&2
    echo "         A full Slicer Superbuild typically requires ~20-30 GB during compilation." >&2
  fi
fi

# -----------------------------------------------------------------------------
# 2. Clone / Update Slicer Source
# -----------------------------------------------------------------------------
if [ "$DO_CLONE" = true ]; then
  echo ""
  echo "==> Setting up Slicer source tree..."
  if [ ! -d "$SRC_DIR/.git" ]; then
    echo "    Cloning ${SLICER_REPO} (branch/tag: ${SLICER_TAG}) into ${SRC_DIR}..."
    git clone --depth 1 --branch "$SLICER_TAG" "$SLICER_REPO" "$SRC_DIR"
  else
    echo "    Updating existing repository in ${SRC_DIR}..."
    git -C "$SRC_DIR" fetch --depth 1 origin "$SLICER_TAG"
    git -C "$SRC_DIR" checkout "$SLICER_TAG"
  fi
fi

# -----------------------------------------------------------------------------
# 3. Configure & Build Slicer Superbuild
# -----------------------------------------------------------------------------
if [ "$DO_BUILD" = true ]; then
  if [ "$ASSUME_YES" = false ]; then
    echo ""
    echo "------------------------------------------------------------"
    echo " Building 3D Slicer from source can take 2 to 4 hours."
    echo " Prefix: ${PREFIX}"
    echo " Jobs:   ${NPROC}"
    echo "------------------------------------------------------------"
    read -rp "Proceed with compilation? [y/N] " confirm
    if [[ ! "$confirm" =~ ^[Yy]$ ]]; then
      echo "Aborted by user."
      exit 0
    fi
  fi

  echo ""
  echo "==> Configuring Slicer Superbuild with CMake..."
  mkdir -p "$SUPERBUILD_DIR"

  SDK_PATH="$(xcrun --show-sdk-path 2>/dev/null || echo "")"

  CMAKE_ARGS=(
    "-G" "Ninja"
    "-DCMAKE_BUILD_TYPE:STRING=Release"
    "-DBUILD_TESTING:BOOL=OFF"
    "-DSlicer_USE_SYSTEM_QT:BOOL=ON"
    "-DSlicer_VTK_SMP_IMPLEMENTATION_TYPE:STRING=Sequential"
    "-DCMAKE_OSX_DEPLOYMENT_TARGET:STRING=${DEPLOYMENT_TARGET}"
    "-S" "$SRC_DIR"
    "-B" "$SUPERBUILD_DIR"
  )

  if [ -n "$SDK_PATH" ]; then
    CMAKE_ARGS+=("-DCMAKE_OSX_SYSROOT:PATH=${SDK_PATH}")
  fi

  if [[ "$QT_DIR" == *"Qt6"* ]]; then
    CMAKE_ARGS+=("-DQt6_DIR:PATH=${QT_DIR}")
    if [ -n "$QT_VERSION" ]; then
      CMAKE_ARGS+=("-DSlicer_REQUIRED_QT_VERSION:STRING=${QT_VERSION}")
    fi
  else
    CMAKE_ARGS+=("-DQt5_DIR:PATH=${QT_DIR}")
  fi

  cmake "${CMAKE_ARGS[@]}"

  echo ""
  echo "==> Compiling Slicer Superbuild (jobs: ${NPROC})..."
  START_BUILD_TIME=$(date +%s)
  cmake --build "$SUPERBUILD_DIR" -- -j"${NPROC}"
  END_BUILD_TIME=$(date +%s)
  BUILD_DURATION=$(( (END_BUILD_TIME - START_BUILD_TIME) / 60 ))
  echo "==> Slicer build completed in ${BUILD_DURATION} minutes."
fi

# -----------------------------------------------------------------------------
# 4. Clean & Strip Build Tree (Mirrors Linux Dockerfile)
# -----------------------------------------------------------------------------
if [ "$DO_PACKAGE" = true ]; then
  if [ ! -f "${SLICER_BUILD_DIR}/SlicerConfig.cmake" ]; then
    echo "Error: SlicerConfig.cmake not found at ${SLICER_BUILD_DIR}/SlicerConfig.cmake" >&2
    echo "       Please run without --package-only to build first." >&2
    exit 1
  fi

  echo ""
  echo "==> Cleaning unneeded files to minimize archive size..."

  # Strip local symbols from dynamic libraries and executables
  echo "    Stripping debug symbols from .dylib files..."
  find "$SUPERBUILD_DIR" -type f -name "*.dylib" -exec strip -x {} \; 2>/dev/null || true
  find "${SLICER_BUILD_DIR}/bin" -maxdepth 1 -type f -perm +111 -exec strip -x {} \; 2>/dev/null || true

  # Remove object files
  echo "    Deleting object files (*.o)..."
  find "$SUPERBUILD_DIR" -name "*.o" -delete 2>/dev/null || true

  # Remove dependency source trees (*-src)
  echo "    Deleting dependency source checkouts (*-src)..."
  find "$SUPERBUILD_DIR" -maxdepth 1 -name "*-src" -type d -exec rm -rf {} + 2>/dev/null || true

  # Remove dependency .git trees
  echo "    Deleting git directories in external projects..."
  find "$SUPERBUILD_DIR" -name ".git" -type d -prune -exec rm -rf {} + 2>/dev/null || true

  # Clean Python cache and test data
  echo "    Deleting Python bytecode and test data..."
  find "$PREFIX" -name "*.pyc" -delete 2>/dev/null || true
  find "$PREFIX" -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
  find "$SUPERBUILD_DIR" -type d -name "test" -path "*/python*/lib/python*/*" -exec rm -rf {} + 2>/dev/null || true

  # ---------------------------------------------------------------------------
  # 5. Write Manifest & Create Archive
  # ---------------------------------------------------------------------------
  echo ""
  echo "==> Creating archive '${OUTPUT_FILE}'..."

  METADATA_FILE="${PREFIX}/.slicer_ci_metadata.env"
  cat <<EOF > "$METADATA_FILE"
# Slicer CI Pre-built Archive Metadata
SLICER_VERSION="${SLICER_TAG}"
ARCH="${ARCH}"
OS="${OS}"
PREFIX="${PREFIX}"
SLICER_DIR="${SLICER_BUILD_DIR}"
QT_DIR="${QT_DIR}"
QT_VERSION="${QT_VERSION}"
CREATED_AT="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
EOF

  TAR_EXCLUDES=(
    --exclude=".git"
    --exclude="*.o"
    --exclude="*.pyc"
    --exclude="__pycache__"
    --exclude=".DS_Store"
    --exclude="*.dSYM"
  )

  mkdir -p "$(dirname "$OUTPUT_FILE")"
  START_TAR_TIME=$(date +%s)

  if [ "$FORMAT" = "zstd" ]; then
    (
      cd "$PREFIX"
      tar "${TAR_EXCLUDES[@]}" -cf - "Slicer" "Slicer-SuperBuild" ".slicer_ci_metadata.env"
    ) | zstd -T0 -3 -o "$OUTPUT_FILE"
  else
    (
      cd "$PREFIX"
      tar "${TAR_EXCLUDES[@]}" -czf "$OUTPUT_FILE" "Slicer" "Slicer-SuperBuild" ".slicer_ci_metadata.env"
    )
  fi

  rm -f "$METADATA_FILE"

  END_TAR_TIME=$(date +%s)
  TAR_DURATION=$((END_TAR_TIME - START_TAR_TIME))

  ARCHIVE_SIZE="$(du -h "$OUTPUT_FILE" | cut -f1)"
  SHA256="$(shasum -a 256 "$OUTPUT_FILE" | awk '{print $1}')"

  echo ""
  echo "============================================================"
  echo "  Archive Created Successfully! (${TAR_DURATION}s)"
  echo "============================================================"
  echo "  File:    $OUTPUT_FILE"
  echo "  Size:    $ARCHIVE_SIZE"
  echo "  SHA256:  $SHA256"
  echo "============================================================"
fi

# -----------------------------------------------------------------------------
# 6. Upload to GitHub Releases (Optional)
# -----------------------------------------------------------------------------
if [ "$DO_UPLOAD" = true ] && [ "$DO_PACKAGE" = true ]; then
  echo ""
  echo "==> Uploading archive to GitHub Release '${RELEASE_TAG}'..."

  if gh release view "$RELEASE_TAG" --repo "$GH_REPO" >/dev/null 2>&1; then
    echo "    Release '${RELEASE_TAG}' exists. Uploading asset..."
    gh release upload "$RELEASE_TAG" "$OUTPUT_FILE" --repo "$GH_REPO" --clobber
  else
    echo "    Release '${RELEASE_TAG}' does not exist. Creating release and uploading asset..."
    gh release create "$RELEASE_TAG" "$OUTPUT_FILE" \
      --repo "$GH_REPO" \
      --title "macOS CI Prebuilt Slicer Assets" \
      --notes "Prebuilt 3D Slicer macOS (${ARCH}) for CI testing."
  fi
  echo "==> Upload complete!"
elif [ "$DO_PACKAGE" = true ]; then
  echo ""
  echo "To upload this archive manually to GitHub Releases:"
  echo "  gh release upload ${RELEASE_TAG} \"${OUTPUT_FILE}\" --repo ${GH_REPO} --clobber"
  echo ""
  echo "Or create a new release with it:"
  echo "  gh release create ${RELEASE_TAG} \"${OUTPUT_FILE}\" --repo ${GH_REPO} \\"
  echo "    --title \"macOS CI Assets\" --notes \"Prebuilt Slicer for macOS (${ARCH}) CI\""
fi

echo ""
echo "In GitHub Actions (macos-14), download and extract to '${PREFIX}':"
echo "  sudo mkdir -p ${PREFIX}"
echo "  sudo chown -R \$USER ${PREFIX}"
if [ "$FORMAT" = "zstd" ]; then
  echo "  curl -L <RELEASE_URL> | zstd -d | tar -x -C ${PREFIX}"
else
  echo "  curl -L <RELEASE_URL> | tar -xz -C ${PREFIX}"
fi
echo "  # Slicer_DIR is then: ${SLICER_BUILD_DIR}"
echo ""

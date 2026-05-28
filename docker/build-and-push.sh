#!/usr/bin/env bash
# =============================================================================
# build-and-push.sh
#
# Builds the SlicerROS2 CI Docker image locally and optionally pushes it to
# the GitHub Container Registry (ghcr.io).
#
# Prerequisites:
#   - Docker with BuildKit (Docker >= 23 or DOCKER_BUILDKIT=1)
#   - For --push: a GitHub PAT with "write:packages" scope
#     Create one at: https://github.com/settings/tokens/new
#     Then run:  export GITHUB_TOKEN=<your_token>
#
# Usage:
#   ./docker/build-and-push.sh                         # build only, tag=latest
#   ./docker/build-and-push.sh --push                  # build + push with default tag
#   ./docker/build-and-push.sh --org <owner> --push    # push to a specific GHCR owner
#   ./docker/build-and-push.sh --push --tag jazzy-slicer-v5.10.0
#   ./docker/build-and-push.sh --slicer-tag v5.12.0 --push --tag jazzy-slicer-v5.12.0
#
# =============================================================================
set -euo pipefail

# --------------------------------------------------------------------------
# Defaults — edit these if you rename the repo / org
# --------------------------------------------------------------------------
REGISTRY="ghcr.io"
ORG="${GITHUB_REPOSITORY_OWNER:-rosmed}"
IMAGE="slicer_ros2_module/ci"
DEFAULT_TAG="jazzy-slicer-v5.10.0"
SLICER_BUILD_TAG="v5.10.0"   # git tag of Slicer to build inside the image

# --------------------------------------------------------------------------
# Parse arguments
# --------------------------------------------------------------------------
DO_PUSH=false
TAG="${DEFAULT_TAG}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --push)
      DO_PUSH=true; shift ;;
    --org)
      ORG="$2"; shift 2 ;;
    --tag)
      TAG="$2"; shift 2 ;;
    --slicer-tag)
      SLICER_BUILD_TAG="$2"; shift 2 ;;
    -h|--help)
      sed -n '/^# Usage/,/^# ===/p' "$0" | head -n -1 | sed 's/^# //; s/^#//'
      exit 0 ;;
    *)
      echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

FULL_IMAGE="${REGISTRY}/${ORG}/${IMAGE}"
VERSIONED_TAG="${FULL_IMAGE}:${TAG}"
LATEST_TAG="${FULL_IMAGE}:latest"

# --------------------------------------------------------------------------
# Locate the Dockerfile (relative to repo root)
# --------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

echo "============================================================"
echo "  SlicerROS2 CI image builder"
echo "============================================================"
echo "  Registry  : ${REGISTRY}"
echo "  Image     : ${FULL_IMAGE}"
echo "  Tag       : ${TAG}"
echo "  Slicer    : ${SLICER_BUILD_TAG}"
echo "  Dockerfile: ${DOCKERFILE}"
echo "  Build ctx : ${REPO_ROOT}"
echo "  Push      : ${DO_PUSH}"
echo "============================================================"
echo ""

# --------------------------------------------------------------------------
# Sanity checks
# --------------------------------------------------------------------------
if ! command -v docker &>/dev/null; then
  echo "ERROR: docker not found in PATH." >&2
  exit 1
fi

if ${DO_PUSH}; then
  if [[ -z "${GITHUB_TOKEN:-}" ]]; then
    echo "ERROR: GITHUB_TOKEN is not set." >&2
    echo "  Create a PAT at https://github.com/settings/tokens/new" >&2
    echo "  with 'write:packages' scope, then:" >&2
    echo "    export GITHUB_TOKEN=<your_token>" >&2
    exit 1
  fi

  echo "--- Logging in to ${REGISTRY} ---"
  echo "${GITHUB_TOKEN}" | docker login "${REGISTRY}" \
    --username "${GITHUB_ACTOR:-$(git config user.email 2>/dev/null || echo 'user')}" \
    --password-stdin
fi

# --------------------------------------------------------------------------
# Build
# --------------------------------------------------------------------------
echo ""
echo "--- Building image (this will take several hours on first run) ---"
echo ""

docker build \
  --file "${DOCKERFILE}" \
  --build-arg "SLICER_TAG=${SLICER_BUILD_TAG}" \
  --tag "${VERSIONED_TAG}" \
  --tag "${LATEST_TAG}" \
  "${REPO_ROOT}"

echo ""
echo "--- Build complete ---"
echo "  Image: ${VERSIONED_TAG}"
echo ""

# Print the image digest (useful for pinning the exact image in ci.yml)
DIGEST=$(docker inspect --format='{{index .RepoDigests 0}}' "${VERSIONED_TAG}" 2>/dev/null || true)
if [[ -n "${DIGEST}" ]]; then
  echo "  Digest: ${DIGEST}"
fi

# --------------------------------------------------------------------------
# Quick sanity test (no X11 needed)
# --------------------------------------------------------------------------
echo "--- Running quick sanity check ---"
docker run --rm "${VERSIONED_TAG}" \
  bash -c '
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "ROS_DISTRO=${ROS_DISTRO}"
    which ros2
    echo "Slicer_DIR=${Slicer_DIR}"
    ls "${Slicer_DIR}/Slicer" && echo "Slicer binary OK"
  '
echo "--- Sanity check passed ---"
echo ""

# --------------------------------------------------------------------------
# Push
# --------------------------------------------------------------------------
if ${DO_PUSH}; then
  echo "--- Pushing ${VERSIONED_TAG} ---"
  docker push "${VERSIONED_TAG}"
  echo "--- Pushing ${LATEST_TAG} ---"
  docker push "${LATEST_TAG}"
  echo ""
  echo "Done!  Update .github/workflows/ci.yml with:"
  echo "  image: ${VERSIONED_TAG}"
else
  echo "Image built locally only (pass --push to upload to GHCR)."
fi

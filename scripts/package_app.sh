#!/usr/bin/env bash
# ==============================================================================
# package_app.sh
#
# Generates a standalone, self-contained macOS application bundle (SlicerROS2.app)
# and an installable disk image (.dmg) for Apple Silicon (arm64).
#
# The generated bundle includes:
#   - 3D Slicer application runtime (arm64)
#   - SlicerROS2 loadable C++ Qt module, MRML, Logic, Python wrappers
#   - minimal_ros2 C++ core libraries and dependencies
#   - Complete Python ROS 2 runtime (rclpy, standard messages, yaml, lark, etc.)
#   - All required Qt plugins (Cocoa QPA, TLS, SQLite)
#
# Usage:
#   ./scripts/package_app.sh [OPTIONS]
#
# Examples:
#   ./scripts/package_app.sh
#   ./scripts/package_app.sh --no-dmg
#   ./scripts/package_app.sh --slicer-app /path/to/Slicer.app
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "==> SlicerROS2 macOS Application Packager"
python3 "${SCRIPT_DIR}/package_slicer_app_macos.py" \
    --workspace "${WORKSPACE_DIR}" \
    --build-dir "${WORKSPACE_DIR}/build" \
    "$@"

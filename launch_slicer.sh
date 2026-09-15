#!/usr/bin/env bash
#
# launch_slicer.sh
#
# Launch 3D Slicer on macOS or Linux with slicer_ros2_module and minimal_ros2.
# Assumes standalone / minimal ROS environment (no ros2 command-line tools).
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build}"

# 1. Locate Slicer executable
if [ -n "$SLICER_EXECUTABLE" ] && [ -x "$SLICER_EXECUTABLE" ]; then
  SLICER_BIN="$SLICER_EXECUTABLE"
elif [ -n "$SLICER_DIR" ] && [ -x "$SLICER_DIR/Slicer" ]; then
  SLICER_BIN="$SLICER_DIR/Slicer"
elif [ -x "$SCRIPT_DIR/../build/Slicer-build/Slicer" ]; then
  SLICER_BIN="$SCRIPT_DIR/../build/Slicer-build/Slicer"
elif [ -x "/Applications/Slicer.app/Contents/MacOS/Slicer" ]; then
  SLICER_BIN="/Applications/Slicer.app/Contents/MacOS/Slicer"
elif which Slicer >/dev/null 2>&1; then
  SLICER_BIN="$(which Slicer)"
else
  echo "ERROR: Slicer executable not found." >&2
  echo "Please set SLICER_EXECUTABLE or SLICER_DIR, e.g.:" >&2
  echo "  export SLICER_DIR=/path/to/Slicer-build" >&2
  exit 1
fi

# 2. Configure minimal ROS 2 environment
if [ -z "$MINIMAL_ROS_DIR" ]; then
  if [ -d "$BUILD_DIR/minimal_ros2-install" ]; then
    MINIMAL_ROS_DIR="$BUILD_DIR/minimal_ros2-install"
  fi
fi

if [ -n "$MINIMAL_ROS_DIR" ] && [ -d "$MINIMAL_ROS_DIR" ]; then
  if [ -f "$MINIMAL_ROS_DIR/setup.sh" ]; then
    . "$MINIMAL_ROS_DIR/setup.sh"
  elif [ -f "$MINIMAL_ROS_DIR/setup.bash" ]; then
    . "$MINIMAL_ROS_DIR/setup.bash"
  fi
  ROS_LIB_DIR="$MINIMAL_ROS_DIR/lib"
fi

# 3. Locate built module directories
MODULE_LIB_DIR=""
FOUND_MODULE_LIB="$(find "$BUILD_DIR" -type f \( -name "libqSlicerROS2Module.dylib" -o -name "libqSlicerROS2Module.so" \) -exec ls -t {} + 2>/dev/null | head -n 1)"
if [ -n "$FOUND_MODULE_LIB" ]; then
  MODULE_LIB_DIR="$(dirname "$FOUND_MODULE_LIB")"
elif [ -d "$BUILD_DIR/lib" ]; then
  MODULE_LIB_DIR="$BUILD_DIR/lib"
fi

if [ -z "$MODULE_LIB_DIR" ] || [ ! -d "$MODULE_LIB_DIR" ]; then
  echo "WARNING: Could not find built slicer_ros2_module libraries in $BUILD_DIR." >&2
  echo "Run ./build_standalone.sh first." >&2
fi

# Assemble module paths
MODULE_PATHS=()
if [ -n "$MODULE_LIB_DIR" ]; then
  MODULE_PATHS+=("$MODULE_LIB_DIR")
fi

if [ -d "$SCRIPT_DIR/Testing/Python" ]; then
  MODULE_PATHS+=("$SCRIPT_DIR/Testing/Python")
fi

if [ -n "$SLICER_ROS2_MODULE_PATHS" ]; then
  IFS=":" read -ra EXTRA_PATHS <<< "$SLICER_ROS2_MODULE_PATHS"
  for p in "${EXTRA_PATHS[@]}"; do
    if [ -d "$p" ]; then
      MODULE_PATHS+=("$p")
    fi
  done
fi

# 4. Configure dynamic library path
CUSTOM_LIB_PATHS=""
if [ -n "$MODULE_LIB_DIR" ]; then
  CUSTOM_LIB_PATHS="$MODULE_LIB_DIR"
fi
if [ -n "$ROS_LIB_DIR" ]; then
  CUSTOM_LIB_PATHS="${CUSTOM_LIB_PATHS:+$CUSTOM_LIB_PATHS:}$ROS_LIB_DIR"
fi

if [[ "$OSTYPE" == "darwin"* ]]; then
  export DYLD_LIBRARY_PATH="${CUSTOM_LIB_PATHS:+$CUSTOM_LIB_PATHS:}$DYLD_LIBRARY_PATH"
  export DYLD_FALLBACK_LIBRARY_PATH="${CUSTOM_LIB_PATHS:+$CUSTOM_LIB_PATHS:}$DYLD_FALLBACK_LIBRARY_PATH"
else
  export LD_LIBRARY_PATH="${CUSTOM_LIB_PATHS:+$CUSTOM_LIB_PATHS:}$LD_LIBRARY_PATH"
fi

# 5. Launch Slicer
LAUNCH_ARGS=()
if [ ${#MODULE_PATHS[@]} -gt 0 ]; then
  LAUNCH_ARGS+=("--additional-module-paths" "${MODULE_PATHS[@]}")
fi

# If testing is requested and no modules-to-ignore was specified, ignore failing Slicer builtin test modules
ARGS=("$@")
HAS_TESTING=false
HAS_IGNORE=false
for arg in "${ARGS[@]}"; do
  if [ "$arg" = "--testing" ]; then
    HAS_TESTING=true
  elif [ "$arg" = "--modules-to-ignore" ]; then
    HAS_IGNORE=true
  fi
done

if [ "$HAS_TESTING" = true ] && [ "$HAS_IGNORE" = false ]; then
  LAUNCH_ARGS+=("--modules-to-ignore" "RSNAVisTutorialTest,SubjectHierarchyGenericSelfTest,JRC2013VisTest,MultiVolumeImporterPlugin")
fi

exec "$SLICER_BIN" "${LAUNCH_ARGS[@]}" "$@"

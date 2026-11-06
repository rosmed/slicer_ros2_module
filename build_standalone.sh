#!/usr/bin/env bash
#
# build_standalone.sh
# Build slicer_ros2_module as a native Slicer loadable module against ROS 2
# using pure CMake (no colcon required).
#
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${BUILD_DIR:-$SCRIPT_DIR/build}"

# 1. Locate Slicer build tree
if [ -z "$SLICER_DIR" ]; then
  if [ -f "$SCRIPT_DIR/../build/Slicer-build/SlicerConfig.cmake" ]; then
    SLICER_DIR="$SCRIPT_DIR/../build/Slicer-build"
  fi
fi

if [ -z "$SLICER_DIR" ] || [ ! -f "$SLICER_DIR/SlicerConfig.cmake" ]; then
  echo "ERROR: Could not find SlicerConfig.cmake."
  echo "Please specify SLICER_DIR, e.g.:"
  echo "  SLICER_DIR=/path/to/Slicer-build ./build_standalone.sh"
  exit 1
fi

# 2. Locate minimal ROS 2 directory (optional; otherwise CMake selects ROS 2)

if [ -n "$MINIMAL_ROS_DIR" ] && [ -d "$MINIMAL_ROS_DIR" ]; then
  if [ -f "$MINIMAL_ROS_DIR/setup.sh" ]; then
    . "$MINIMAL_ROS_DIR/setup.sh"
  elif [ -f "$MINIMAL_ROS_DIR/setup.bash" ]; then
    . "$MINIMAL_ROS_DIR/setup.bash"
  fi
fi

# 3. Let CMake choose the native ROS Python or the Superbuild's Slicer Python.
# For a pre-built minimal ROS installation, keep preferring Slicer's Python.
if [ -z "$PYTHON_EXE" ] && [ -n "$MINIMAL_ROS_DIR" ] && [ -f "$SLICER_DIR/../python-install/bin/python3" ]; then
  PYTHON_EXE="$SLICER_DIR/../python-install/bin/python3"
fi

echo "==> Using Slicer build:   $SLICER_DIR"
echo "==> Using Python:         ${PYTHON_EXE:-automatic}"
echo "==> Build directory:      $BUILD_DIR"

NPROC="$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

# 4. Configure via CMake
CMAKE_ARGS=(
  "-DSlicer_DIR=$SLICER_DIR"
  "-DCMAKE_BUILD_TYPE=Release"
)

if [ -n "$PYTHON_EXE" ]; then
  CMAKE_ARGS+=("-DPython3_EXECUTABLE=$PYTHON_EXE")
fi

if [ -n "$MINIMAL_ROS_DIR" ] && [ -d "$MINIMAL_ROS_DIR" ]; then
  echo "==> Using pre-built minimal ROS: $MINIMAL_ROS_DIR"
  CMAKE_ARGS+=("-DMINIMAL_ROS_DIR=$MINIMAL_ROS_DIR")
else
  echo "==> CMake will select ROS 2 using the platform default or explicit options."
  if [ -n "$MINIMAL_ROS_SOURCE_DIR" ]; then
    CMAKE_ARGS+=("-DMINIMAL_ROS_SOURCE_DIR=$MINIMAL_ROS_SOURCE_DIR")
  fi
fi

cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR" "${CMAKE_ARGS[@]}" "$@"

# 5. Compile
echo ""
echo "==> Compiling (jobs: $NPROC)..."
cmake --build "$BUILD_DIR" -j "$NPROC"

echo ""
echo "==> Slicer ROS2 module built successfully!"
FOUND_LIB="$(find "$BUILD_DIR" -type f \( -name "libqSlicerROS2Module.dylib" -o -name "libqSlicerROS2Module.so" \) 2>/dev/null | head -n 1)"
if [ -n "$FOUND_LIB" ]; then
  MODULE_BIN_DIR="$(dirname "$FOUND_LIB")"
else
  MODULE_BIN_DIR="$BUILD_DIR/lib"
fi
echo "==> Module binaries located in: $MODULE_BIN_DIR"
echo "==> Launch with: ./launch_slicer.sh or ./scripts/slicer"

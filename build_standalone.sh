#!/usr/bin/env bash
#
# build_standalone.sh
# Build slicer_ros2_module as a native Slicer loadable module against minimal_ros2
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

# 2. Locate minimal ROS 2 directory
if [ -z "$MINIMAL_ROS_DIR" ]; then
  echo "ERROR: MINIMAL_ROS_DIR is not set."
  echo "Please specify MINIMAL_ROS_DIR, e.g.:"
  echo "  MINIMAL_ROS_DIR=/path/to/minimal_ros2/install ./build_standalone.sh"
  exit 1
fi

# Source minimal ROS setup to populate PYTHONPATH and AMENT_PREFIX_PATH
if [ -f "$MINIMAL_ROS_DIR/setup.bash" ]; then
  source "$MINIMAL_ROS_DIR/setup.bash"
elif [ -f "$MINIMAL_ROS_DIR/setup.zsh" ]; then
  source "$MINIMAL_ROS_DIR/setup.zsh"
fi

# 3. Locate Python executable (prefer minimal_ros2 .venv or Slicer python)
if [ -z "$PYTHON_EXE" ]; then
  if [ -f "$MINIMAL_ROS_DIR/../.venv/bin/python3" ]; then
    PYTHON_EXE="$MINIMAL_ROS_DIR/../.venv/bin/python3"
  elif [ -f "$SLICER_DIR/../python-install/bin/python3" ]; then
    PYTHON_EXE="$SLICER_DIR/../python-install/bin/python3"
  else
    PYTHON_EXE="$(which python3)"
  fi
fi

echo "==> Using Slicer build:   $SLICER_DIR"
echo "==> Using minimal ROS:    $MINIMAL_ROS_DIR"
echo "==> Using Python:         $PYTHON_EXE"
echo "==> Build directory:      $BUILD_DIR"

NPROC="$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

# 4. Configure via CMake
cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR" \
  -DSlicer_DIR="$SLICER_DIR" \
  -DMINIMAL_ROS_DIR="$MINIMAL_ROS_DIR" \
  -DPython3_EXECUTABLE="$PYTHON_EXE" \
  -DSlicerROS2_ENABLE_TURTLESIM=OFF \
  -DSlicerROS2_ENABLE_ROSBAG2=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  "$@"

# 4. Compile
echo ""
echo "==> Compiling slicer_ros2_module (jobs: $NPROC)..."
cmake --build "$BUILD_DIR" -j "$NPROC"

echo ""
echo "==> Slicer ROS2 module built successfully!"
echo "==> Module binaries located in: $BUILD_DIR/lib/Slicer-5.13/qt-loadable-modules"

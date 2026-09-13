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
  CANDIDATE_SLICER_DIRS=(
    "$SCRIPT_DIR/../build/Slicer-build"
    "/Users/anton/devel/slicer/build/Slicer-build"
  )
  for dir in "${CANDIDATE_SLICER_DIRS[@]}"; do
    if [ -f "$dir/SlicerConfig.cmake" ]; then
      SLICER_DIR="$dir"
      break
    fi
  done
fi

if [ -z "$SLICER_DIR" ] || [ ! -f "$SLICER_DIR/SlicerConfig.cmake" ]; then
  echo "ERROR: Could not find SlicerConfig.cmake."
  echo "Please specify SLICER_DIR, e.g.:"
  echo "  SLICER_DIR=/path/to/Slicer-build ./build_standalone.sh"
  exit 1
fi

# 2. Locate minimal_ros2 underlay
if [ -z "$ROS2_UNDERLAY_DIR" ]; then
  CANDIDATE_UNDERLAY_DIRS=(
    "$SCRIPT_DIR/../../minimal_ros2/install"
    "/Users/anton/devel/minimal_ros2/install"
  )
  for dir in "${CANDIDATE_UNDERLAY_DIRS[@]}"; do
    if [ -f "$dir/setup.bash" ] || [ -f "$dir/local_setup.bash" ]; then
      ROS2_UNDERLAY_DIR="$dir"
      break
    fi
  done
fi

if [ -z "$ROS2_UNDERLAY_DIR" ]; then
  echo "ERROR: Could not find minimal_ros2 install prefix."
  echo "Please specify ROS2_UNDERLAY_DIR, e.g.:"
  echo "  ROS2_UNDERLAY_DIR=/path/to/minimal_ros2/install ./build_standalone.sh"
  exit 1
fi

# Source underlay setup to populate PYTHONPATH and AMENT_PREFIX_PATH
if [ -f "$ROS2_UNDERLAY_DIR/setup.bash" ]; then
  source "$ROS2_UNDERLAY_DIR/setup.bash"
elif [ -f "$ROS2_UNDERLAY_DIR/setup.zsh" ]; then
  source "$ROS2_UNDERLAY_DIR/setup.zsh"
fi

# 3. Locate Python executable (prefer underlay .venv or Slicer python)
if [ -z "$PYTHON_EXE" ]; then
  if [ -f "$ROS2_UNDERLAY_DIR/../.venv/bin/python3" ]; then
    PYTHON_EXE="$ROS2_UNDERLAY_DIR/../.venv/bin/python3"
  elif [ -f "$SLICER_DIR/../python-install/bin/python3" ]; then
    PYTHON_EXE="$SLICER_DIR/../python-install/bin/python3"
  else
    PYTHON_EXE="$(which python3)"
  fi
fi

echo "==> Using Slicer build:   $SLICER_DIR"
echo "==> Using ROS 2 underlay: $ROS2_UNDERLAY_DIR"
echo "==> Using Python:         $PYTHON_EXE"
echo "==> Build directory:      $BUILD_DIR"

NPROC="$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)"

# 4. Configure via CMake
cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR" \
  -DSlicer_DIR="$SLICER_DIR" \
  -DROS2_UNDERLAY_DIR="$ROS2_UNDERLAY_DIR" \
  -DPython3_EXECUTABLE="$PYTHON_EXE" \
  -DSlicerROS2_ENABLE_MOVEIT=OFF \
  -DSlicerROS2_ENABLE_TURTLESIM=OFF \
  -DSlicerROS2_ENABLE_ROSBAG2=OFF \
  -DSlicerROS2_USE_AMENT=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  "$@"

# 4. Compile
echo ""
echo "==> Compiling slicer_ros2_module (jobs: $NPROC)..."
cmake --build "$BUILD_DIR" -j "$NPROC"

echo ""
echo "==> Slicer ROS2 module built successfully!"
echo "==> Module binaries located in: $BUILD_DIR/lib/Slicer-5.13/qt-loadable-modules"

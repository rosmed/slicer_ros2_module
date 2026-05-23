# =============================================================================
# SlicerROS2 CI Image
#
# Ubuntu 24.04 + ROS2 Jazzy + 3D Slicer 5.10 (built from source)
#
# Build this image locally and push to GHCR — it does NOT need to be rebuilt
# on every CI run.  See docker/build-and-push.sh for instructions.
#
# Image path: ghcr.io/rosmed/slicer_ros2_module/ci:jazzy-slicer-v5.10.0
# =============================================================================

ARG SLICER_TAG=v5.10.0
ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# Stage 1: base dependencies
# ---------------------------------------------------------------------------
FROM ubuntu:24.04 AS base

ARG SLICER_TAG
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
  TZ=UTC \
  LANG=C.UTF-8

# --- Add ROS 2 apt repository FIRST -----------------------------------------
# This must come before the main apt install because colcon and rosdep
# are only available from the ROS apt repo, not the default Ubuntu repos.
RUN apt-get update && apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/ros2.list \
  && rm -rf /var/lib/apt/lists/*

# --- All system + ROS 2 packages in one layer --------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
  # Core build tools
  git \
  git-lfs \
  build-essential \
  cmake \
  ninja-build \
  mold \
  wget \
  patch \
  patchelf \
  software-properties-common \
  # Python + colcon (from ROS repo)
  python3 \
  python3-pip \
  python3-colcon-common-extensions \
  python3-rosdep \
  # Slicer Qt5 system packages (Ubuntu 24.04 ships Qt 5.15.x)
  libqt5x11extras5-dev \
  qtmultimedia5-dev \
  libqt5svg5-dev \
  qtwebengine5-dev \
  libqt5xmlpatterns5-dev \
  qttools5-dev \
  qtbase5-private-dev \
  # Slicer system library dependencies
  libxt-dev \
  libssl-dev \
  libbz2-dev \
  libglu1-mesa-dev \
  libnss3 \
  libpulse-mainloop-glib0 \
  libasound2t64 \
  # ROS 2 Jazzy + all module dependencies
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-ament-cmake \
  ros-${ROS_DISTRO}-rclcpp \
  ros-${ROS_DISTRO}-std-msgs \
  ros-${ROS_DISTRO}-std-srvs \
  ros-${ROS_DISTRO}-sensor-msgs \
  ros-${ROS_DISTRO}-geometry-msgs \
  ros-${ROS_DISTRO}-shape-msgs \
  ros-${ROS_DISTRO}-trajectory-msgs \
  ros-${ROS_DISTRO}-kdl-parser \
  ros-${ROS_DISTRO}-tf2 \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-turtlesim \
  ros-${ROS_DISTRO}-object-recognition-msgs \
  ros-${ROS_DISTRO}-moveit-msgs \
  ros-${ROS_DISTRO}-moveit-core \
  ros-${ROS_DISTRO}-moveit-ros-planning \
  ros-${ROS_DISTRO}-moveit-ros-planning-interface \
  ros-${ROS_DISTRO}-rosbag2-interfaces \
  ros-${ROS_DISTRO}-urdf \
  # Headless display for testing
  xvfb \
  && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# Stage 2: build 3D Slicer from source
# ---------------------------------------------------------------------------
FROM base AS slicer-build

ARG SLICER_TAG
ENV SLICER_TAG=${SLICER_TAG}

WORKDIR /opt/slicer

# Clone Slicer at pinned tag (shallow clone to save disk & time)
RUN git clone --depth 1 --branch ${SLICER_TAG} \
  https://github.com/Slicer/Slicer.git \
  /opt/slicer/Slicer

# Superbuild configuration.
#
# Key flags:
#   -DSlicer_USE_SYSTEM_OpenSSL=ON  - use Ubuntu's OpenSSL (avoids version conflicts with ROS2)
#   -DSlicer_USE_SYSTEM_bzip2=ON    - use Ubuntu's bzip2 (same reason)
#   -DQt5_DIR                       - point to system Qt5 (Ubuntu 24.04 ships Qt 5.15.x)
#   -DCMAKE_BUILD_TYPE=Release      - required for a usable colcon-linked build
#
# Ubuntu 24.04 ships Qt 5.15.x in /usr/lib/x86_64-linux-gnu/cmake/Qt5,
# which meets the Slicer Qt 5.15.2 requirement.
RUN mkdir -p /opt/slicer/Slicer-SuperBuild && \
  cmake \
  -G Ninja \
  -DCMAKE_BUILD_TYPE:STRING=Release \
  -DSlicer_USE_SYSTEM_OpenSSL:BOOL=ON \
  -DSlicer_USE_SYSTEM_bzip2:BOOL=ON \
  -DQt5_DIR:PATH=/usr/lib/x86_64-linux-gnu/cmake/Qt5 \
  -S /opt/slicer/Slicer \
  -B /opt/slicer/Slicer-SuperBuild

# Build — use all CPUs but cap memory via NINJAJOBS if needed.
# The build takes 4–8 hours depending on hardware.
RUN cmake --build /opt/slicer/Slicer-SuperBuild -- -j$(nproc)

# ---------------------------------------------------------------------------
# Stage 3: final image
# ---------------------------------------------------------------------------
FROM base AS final

ARG SLICER_TAG

# SlicerConfig.cmake has absolute paths baked into it that reference CTK, VTK,
# ITK, etc. built alongside Slicer-build in the superbuild.  We must copy the
# entire superbuild tree — copying only Slicer-build is not sufficient.
COPY --from=slicer-build /opt/slicer/Slicer-SuperBuild \
  /opt/slicer/Slicer-SuperBuild

# SlicerConfig.cmake and UseSlicer.cmake also include cmake modules from the
# Slicer *source* tree (e.g. SlicerBlockFindQtAndCheckVersion.cmake,
# UseSlicerMacros.cmake) using absolute paths baked in at build time.
# We only need the CMake/ subdirectory — not the entire ~500 MB source clone.
COPY --from=slicer-build /opt/slicer/Slicer/CMake \
  /opt/slicer/Slicer/CMake

# Space recovery — all steps are safe for a build-only (not runtime) image.
# Headers and *Config.cmake files are intentionally kept; colcon needs them
# to compile and link the module against Slicer's dependencies.
RUN set -e; \
  SB=/opt/slicer/Slicer-SuperBuild; \
  \
  echo "==> Stripping debug symbols from shared libs and executables..."; \
  find "$SB" -type f \( -name "*.so" -o -name "*.so.*" \) \
  -exec strip --strip-unneeded {} \; 2>/dev/null || true; \
  find "$SB/Slicer-build/bin" -type f -executable \
  -exec strip --strip-unneeded {} \; 2>/dev/null || true; \
  \
  echo "==> Deleting object files and static archives..."; \
  find "$SB" \( -name "*.o" -o -name "*.a" \) -delete; \
  \
  echo "==> Deleting dependency source trees (*-src)..."; \
  find "$SB" -maxdepth 1 -name "*-src" -type d -exec rm -rf {} + 2>/dev/null || true; \
  \
  echo "==> Deleting CMakeFiles compiler-detection artifacts from dep builds..."; \
  find "$SB" -mindepth 2 -maxdepth 3 -name "CMakeFiles" -type d \
  -not -path "$SB/Slicer-build/*" \
  -exec rm -rf {} + 2>/dev/null || true; \
  \
  echo "==> Deleting Python bytecode cache..."; \
  find /opt/slicer -name "*.pyc" -delete 2>/dev/null || true; \
  find /opt/slicer -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true; \
  \
  echo "==> Done."

# Make Slicer_DIR available to colcon / cmake without needing -D flags
ENV Slicer_DIR=/opt/slicer/Slicer-SuperBuild/Slicer-build
ENV ROS_DISTRO=jazzy

# Source ROS2 on every interactive shell — CI steps source it explicitly
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc

# Metadata
LABEL org.opencontainers.image.title="SlicerROS2 CI"
LABEL org.opencontainers.image.description="Ubuntu 24.04 + ROS2 Jazzy + Slicer ${SLICER_TAG} (from source)"
LABEL org.opencontainers.image.source="https://github.com/rosmed/slicer_ros2_module"
LABEL slicer.version="${SLICER_TAG}"

WORKDIR /

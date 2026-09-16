# TODO & Future Work

## 1. macOS CI Testing (Apple Silicon / `macos-14`)

### Context
Unlike Linux, where CI testing runs inside a Docker container (`ghcr.io/rosmed/slicer_ros2_module/ci`), Docker on macOS only runs Linux containers. Native macOS CI must run directly on GitHub Actions macOS virtual machine runners (`macos-14` Apple Silicon `arm64`).

Because compiling 3D Slicer from source takes ~45 minutes to an hour, CI jobs cannot rebuild Slicer on every commit. Instead, Slicer is pre-built once inside the standard prefix `/opt/slicer` and distributed as a compressed archive (`.tar.zst`) hosted on GitHub Releases.

---

### Status / Completed Work

- [x] **Packaging Script**: Created [`scripts/package_slicer_macos.sh`](file:///Users/anton/devel/slicer/slicer_ros2_module/scripts/package_slicer_macos.sh) to:
  - Clone Slicer `main` (or a specified tag) into `/opt/slicer/Slicer`.
  - Compile the Superbuild into `/opt/slicer/Slicer-SuperBuild` using Homebrew Qt6 and Ninja.
  - Automatically pass the Xcode SDK path (`xcrun --show-sdk-path`) to resolve OpenSSL sysroot issues.
  - Strip local symbols from `.dylib`s and remove object files, dependency source trees, and Python caches.
  - Package `/opt/slicer` into a ~545 MB `.tar.zst` archive.
- [x] **Portability Fix**: Updated [`MRML/vtkMRMLROS2RobotNode.cxx`](file:///Users/anton/devel/slicer/slicer_ros2_module/MRML/vtkMRMLROS2RobotNode.cxx#L28-L32) with `__has_include(<Eigen/Geometry>)` so it compiles cleanly on both Homebrew and Linux include paths.
- [x] **Path Resolution Fix**: Updated [`launch_slicer.sh`](file:///Users/anton/devel/slicer/slicer_ros2_module/launch_slicer.sh) to resolve `BUILD_DIR` to an absolute path and properly prefer `qt-loadable-modules`.
- [x] **End-to-End Validation**: Built `slicer_ros2_module` standalone against `/opt/slicer` and verified that `./scripts/test_ci` passes all 23 tests headlessly.

---

### Remaining Action Items

#### 1. Upload Prebuilt Slicer Asset to GitHub Releases
Upload the newly generated archive (`slicer-main-macos-arm64.tar.zst`) to the `ci-assets` release:

```bash
gh release create ci-assets slicer-main-macos-arm64.tar.zst \
  --repo rosmed/slicer_ros2_module \
  --title "macOS CI Prebuilt Slicer Assets" \
  --notes "Prebuilt 3D Slicer (main / arm64) for macOS GitHub Actions CI testing."
```

*(If the `ci-assets` release already exists, use `gh release upload ci-assets slicer-main-macos-arm64.tar.zst --repo rosmed/slicer_ros2_module --clobber`)*

---

#### 2. Add GitHub Actions Workflow
Create `.github/workflows/macos-14-slicer-main.yml`:

```yaml
name: macOS Build and Test

on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]
  workflow_dispatch:

jobs:
  build-and-test:
    name: Build & Test (macOS 14 / Slicer main)
    runs-on: macos-14

    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Install dependencies
        run: |
          brew install qt@6 zstd

      - name: Restore Prebuilt Slicer
        run: |
          sudo mkdir -p /opt/slicer
          sudo chown -R $USER /opt/slicer
          curl -L "https://github.com/rosmed/slicer_ros2_module/releases/download/ci-assets/slicer-main-macos-arm64.tar.zst" \
            | zstd -d | tar -x -C /opt/slicer

      - name: Build Module (Standalone with minimal_ros2)
        run: |
          SLICER_DIR=/opt/slicer/Slicer-SuperBuild/Slicer-build ./build_standalone.sh

      - name: Run Tests
        run: |
          ./scripts/test_ci

      - name: Upload logs
        if: always()
        uses: actions/upload-artifact@v4
        with:
          name: macos-colcon-logs
          path: build/log/
          retention-days: 14
```

---

#### 3. Future Enhancements
- [ ] Add support for versioned Slicer tags (e.g. `v5.12.0`) in addition to `main`.
- [ ] Optional: Add a scheduled or manually-triggered workflow (`workflow_dispatch`) to periodically rebuild and update the macOS Slicer archive when Slicer or Homebrew Qt updates.

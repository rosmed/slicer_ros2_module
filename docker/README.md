# SlicerROS2 CI Docker Image

The Docker image in this directory is used by GitHub Actions to build and test
SlicerROS2. It contains Ubuntu, ROS 2 Jazzy, and a source-built 3D Slicer tree,
so CI jobs do not have to rebuild Slicer on every run.

The workflow currently pulls the image from GitHub Container Registry:

```yaml
ghcr.io/rosmed/slicer_ros2_module/ci:jazzy-slicer-v5.10.0
```

Rebuild and push the image when the base operating system, ROS distribution,
Slicer version, or required system dependencies change.

## Build Locally

From the repository root:

```bash
./docker/build-and-push.sh
```

This builds the default tag locally:

```text
ghcr.io/rosmed/slicer_ros2_module/ci:jazzy-slicer-v5.10.0
```

To build a different Slicer tag and image tag:

```bash
./docker/build-and-push.sh \
  --slicer-tag v5.12.0 \
  --tag jazzy-slicer-v5.12.0
```

## Push To GHCR

Create a GitHub personal access token with `write:packages` permission, then
export it before running the script:

```bash
export GITHUB_TOKEN=<your_token>
./docker/build-and-push.sh --push
```

To push a custom tag:

```bash
export GITHUB_TOKEN=<your_token>
./docker/build-and-push.sh \
  --slicer-tag v5.12.0 \
  --tag jazzy-slicer-v5.12.0 \
  --push
```

After pushing a new tag, update the workflow files under `.github/workflows/` (e.g. `ubuntu-24.04-jazzy-slicer-5.10.yml` or `ubuntu-24.04-jazzy-slicer-5.12.yml`) to reference the new versioned image tag.

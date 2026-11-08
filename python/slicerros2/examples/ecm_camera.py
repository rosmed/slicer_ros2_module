"""Render a live ECM endoscope image into an RGB MRML volume."""

_camera = None


def run(*, include_SUJ=False, include_ECM=True, fixed_frame="world", **camera_options):
    """Load the cart and return its ECM VirtualCamera (see .image_node)."""
    from slicerros2.dvrk import ECMCamera, load_patient_cart

    global _camera
    cleanup()
    robots = load_patient_cart(include_SUJ=include_SUJ, include_ECM=include_ECM, fixed_frame=fixed_frame, wait=True)
    _camera = ECMCamera(robots.get("ECM"), fixed_frame=fixed_frame, **camera_options)
    return _camera


def cleanup():
    """Remove the example's camera and image, leaving the cart loaded."""
    global _camera
    if _camera is not None:
        _camera.close(remove_image=True)
        _camera = None

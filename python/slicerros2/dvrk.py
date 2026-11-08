"""dVRK patient-cart visualization and ECM camera utilities."""

from .robots import get_ros_node, load_robot, wait_for_models
from .cameras import VirtualCamera


def _load_patient_cart(*, include_SUJ=False, include_ECM=True, fixed_frame="world", ros_node=None,
                      wait=False, timeout=30.0):
    """Load PSM1, PSM2, PSM3 and ECM from their robot_state_publishers.

    Set ``include_ECM=False`` to load only the PSMs; set ``include_SUJ=True``
    for a Classic/Si cart's supporting structure. Frame names
    are already embedded in dVRK URDFs, so no additional TF prefix is added.
    Returns a dictionary of robot nodes; repeated calls reuse existing nodes.
    """
    node = get_ros_node(ros_node)
    publishers = {arm: f"/{arm}/robot_state_publisher"
                  for arm in ("PSM1", "PSM2", "PSM3")}
    if include_ECM:
        publishers["ECM"] = "/ECM/robot_state_publisher"
    if include_SUJ:
        publishers["SUJ"] = "/SUJ/SUJ_robot_state_publisher"
    robots = {name: load_robot(name, publisher, fixed_frame=fixed_frame, ros_node=node)
              for name, publisher in publishers.items()}
    return wait_for_models(robots, timeout) if wait else robots


def unload_patient_cart(*, ros_node=None):
    """Remove the standard cart robot nodes and their visuals, when present."""
    node = get_ros_node(ros_node)
    _unload_cart_robots(node, ("PSM1", "PSM2", "PSM3", "ECM", "SUJ"))


def _unload_cart_robots(node, names):
    for name in names:
        if node.GetRobotNodeByName(name) is not None:
            node.RemoveAndDeleteRobotNode(name)


class PatientCart:
    """A reusable patient-cart loader; loading is explicit and idempotent."""

    def __init__(self, *, include_SUJ=False, include_ECM=True, fixed_frame="world", ros_node=None):
        self.ros_node = get_ros_node(ros_node)
        self.include_SUJ = include_SUJ
        self.include_ECM = include_ECM
        self.fixed_frame = fixed_frame
        self.robots = {}

    def load(self, *, wait=False, timeout=30.0, fit_view=False):
        self.robots = _load_patient_cart(include_SUJ=self.include_SUJ, include_ECM=self.include_ECM,
                                        fixed_frame=self.fixed_frame, ros_node=self.ros_node)
        # Retain the nodes even if waiting times out, so unload() can clean up.
        if wait:
            wait_for_models(self.robots, timeout)
        if fit_view:
            import slicer
            slicer.util.resetThreeDViews()
        return self.robots

    def unload(self):
        _unload_cart_robots(self.ros_node, tuple(self.robots))
        self.robots.clear()

    def create_camera(self, **options):
        return ECMCamera(self.robots.get("ECM"), ros_node=self.ros_node,
                         fixed_frame=self.fixed_frame, **options)


def _camera_transform(ecm_robot=None, *, ros_node=None, tip_frame=None, fixed_frame="world"):
    frame = tip_frame or "ECM_optical"
    node = None
    if ecm_robot is None:
        node = get_ros_node(ros_node)
        ecm_robot = node.GetRobotNodeByName("ECM")
    if ecm_robot is not None:
        fallback = None
        for index in range(ecm_robot.GetNumberOfNodeReferences("lookup")):
            lookup = ecm_robot.GetNthNodeReference("lookup", index)
            if lookup is None:
                continue
            child_frame = lookup.GetChildID()
            if child_frame == frame:
                return lookup, None
            if "/" not in frame and child_frame.split("/")[-1] == frame and fallback is None:
                fallback = lookup
        if fallback is not None:
            return fallback, None
        raise RuntimeError(f"Camera frame {frame} not found. Wait for ECM robot visuals.")

    lookup = node.CreateAndAddTf2LookupNode(fixed_frame, frame)
    if lookup is None:
        raise RuntimeError(f"Could not create TF lookup {fixed_frame} -> {frame}")
    lookup_id = lookup.GetID()

    def cleanup():
        if lookup.GetScene() is not None and node.GetScene() is not None:
            node.RemoveAndDeleteTf2LookupNode(lookup_id)

    return lookup, cleanup


class ECMCamera(VirtualCamera):
    """Live dVRK camera tracking ECM_optical, with or without ECM visuals.

    ROS axes are X right, Y down, Z forward. The RGB image is available through
    image_node. Call close(remove_image=True) to release the camera and its image.
    """

    def __init__(self, ecm_robot=None, *, ros_node=None, tip_frame=None,
                 fixed_frame="world", name="ECM camera", **options):
        lookup, cleanup = _camera_transform(ecm_robot, ros_node=ros_node,
                                           tip_frame=tip_frame, fixed_frame=fixed_frame)
        try:
            super().__init__(lookup, name=name, **options)
        except Exception:
            if cleanup is not None:
                cleanup()
            raise
        self._on_close_callback = cleanup


def create_ecm_camera(ecm_robot=None, **options):
    """Convenience constructor for ECMCamera."""
    return ECMCamera(ecm_robot, **options)


def load_patient_cart(**options):
    """Load the cart and return its robot-node dictionary."""
    wait = options.pop("wait", False)
    timeout = options.pop("timeout", 30.0)
    return PatientCart(**options).load(wait=wait, timeout=timeout)

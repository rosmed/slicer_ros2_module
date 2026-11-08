"""Small helpers for robot visualization through SlicerROS2."""

import time


def get_ros_node(ros_node=None):
    """Return the supplied ROS node or SlicerROS2's default node."""
    if ros_node is not None:
        return ros_node
    import slicer

    logic = slicer.util.getModuleLogic("ROS2")
    if logic is None:
        raise RuntimeError("Load the SlicerROS2 module first.")
    return logic.GetDefaultROS2Node()


def load_robot(name, parameter_node, *, parameter="robot_description",
               fixed_frame="world", tf_prefix="", ros_node=None):
    """Load a URDF asynchronously, reusing an existing robot with this name.

    Source the robot's ROS workspace before launching Slicer so package:// mesh
    URLs can be resolved. Existing nodes retain their original configuration.
    """
    node = get_ros_node(ros_node)
    robot = node.GetRobotNodeByName(name)
    if robot is None:
        robot = node.CreateAndAddRobotNode(
            name, parameter_node, parameter, fixed_frame, tf_prefix)
    if robot is None:
        raise RuntimeError(f"Could not create robot {name!r}.")
    return robot


def wait_for_models(robots, timeout=30.0):
    """Wait for robot visuals while processing Slicer's Qt/ROS events.

    ``robots`` is a dictionary of names to robot nodes. Call on Slicer's main
    thread. This checks geometry availability, not TF connectivity.
    """
    import slicer

    deadline = time.monotonic() + timeout
    while True:
        missing = [name for name, robot in robots.items()
                   if robot.GetScene() is None or robot.GetNumberOfNodeReferences("model") == 0]
        if not missing:
            return robots
        if time.monotonic() >= deadline:
            raise TimeoutError(
                "Robot visuals not available: " + ", ".join(missing)
                + ". Check robot_description services and mesh package paths.")
        slicer.app.processEvents()
        time.sleep(0.01)

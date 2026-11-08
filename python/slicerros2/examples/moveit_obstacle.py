"""Publish a movable obstacle using the shared MoveIt example helpers."""

# [doc: config]
ROBOT_NAME         = "ur5"                   # name given to the robot node
URDF_NODE_NAME     = "robot_state_publisher" # ROS 2 node that owns robot_description
URDF_PARAM_NAME    = "robot_description"     # parameter holding the URDF
FIXED_FRAME        = "world"                 # MoveIt fixed frame (TF2 parent of obstacle)
TF_PREFIX          = ""                      # tf prefix, leave empty if none
JOINT_STATES_TOPIC = "/joint_states"
PLANNING_GROUP     = "ur_manipulator"        # MoveIt planning group name
MOVE_GROUP_EXISTS  = True                    # set False to skip MoveIt IK setup
# [end: config]

_state = None


def run():
    """Set up MoveIt and return the robot, obstacle, transform and broadcaster."""
    import slicer
    from types import SimpleNamespace
    from slicerros2.examples import _moveit

    setup = _moveit.setup_robot(ROBOT_NAME, URDF_NODE_NAME, URDF_PARAM_NAME,
                                FIXED_FRAME, TF_PREFIX, JOINT_STATES_TOPIC,
                                PLANNING_GROUP, MOVE_GROUP_EXISTS)
    obstacle = _moveit.add_obstacle(setup.motionLogic, setup.robotNode, FIXED_FRAME)
    slicer.util.selectModule("ROS2MotionControl")
    global _state
    _state = SimpleNamespace(**{**vars(setup), **vars(obstacle)})
    return _state


if __name__ == "__main__":
    run()

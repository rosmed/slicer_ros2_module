
""""""""""""""
Motion Control
""""""""""""""

========
Overview
========

The ``ROS2MotionControl`` module provides a high-level interface for robot motion planning and execution using **MoveIt 2**. It allows users to:

*   Perform **Inverse Kinematics (IK)** interactively by dragging a 3D gizmo in the Slicer view.
*   **Plan trajectories** using internal generators or MoveIt's planning services.
*   **Execute trajectories** on real or simulated hardware.

==============
Getting Started
==============

To use motion control, you must have a robot running with a MoveIt ``move_group`` node. 

1.  **Launch ROS 2**: Use one of the provided launch files or your own MoveIt setup. For example, for a Universal Robot:
    
    .. code-block:: bash

       ros2 launch slicer_ros2_module ur_sim_control.launch.py launch_rviz:=true

2.  **Start Slicer**: Load the robot in the **ROS2** module as described in :ref:`load_robot`.

3.  **Open Motion Control**: Switch to the **ROS2 Motion Control** module in Slicer.

======================
Planning and execution
======================

Interactive IK
==============

When a robot is loaded, the Motion Control module allows you to manipulate it via a 3D transformation gizmo (sphere). Dragging this gizmo will trigger an IK request, and the robot model in Slicer will update in real-time to match the solution.

If the solver fails (e.g., the target is out of reach), the robot visualization will turn **red**.

Trajectory Planning
===================

Once a goal pose is reached via IK, you can use the "Plan" button to generate a trajectory. Slicer will display a scrubber allowing you to preview the plan.

*   **Generators**: Supports multiple planning backends, including MoveIt and basic joint interpolation.
*   **Planning Group**: Specify the MoveIt planning group (e.g., ``ur_manipulator``) to use for planning.

Execution
=========

The "Execute" button sends the planned trajectory to the robot's controller via the ``ExecuteTrajectory`` action.

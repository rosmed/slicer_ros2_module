Motion Control
==============

The ``ROS2MotionControl`` module provides a high-level interface for robot motion planning and execution using **MoveIt 2**. It allows users to:

*   Perform **Inverse Kinematics (IK)** interactively by dragging a 3D gizmo in the Slicer view.
*   **Plan trajectories** using internal generators or MoveIt's planning services.
*   **Execute trajectories** on real or simulated hardware.

Motion control builds on the robot loading workflow described in
:doc:`/pages/robot-visualization`.  Slicer models can also be published
to the MoveIt planning scene as collision objects; see the MoveIt
obstacle example in :doc:`/pages/examples`.

Getting Started
---------------

To use motion control, you must have a robot running with a MoveIt ``move_group`` node. 

* **Launch ROS 2**: Use one of the provided launch files or your own MoveIt setup. For example, for a Universal Robot:
    
    .. code-block:: bash

       ros2 launch slicer_ros2_module ur_sim_control.launch.py launch_rviz:=true

* **Start Slicer**: Load the robot in the **ROS2** module as described in :ref:`load_robot`.

* **Open Motion Control**: Switch to the **ROS2 Motion Control** module in Slicer.

Planning and execution
----------------------

Interactive IK
~~~~~~~~~~~~~~

When a robot is loaded, the Motion Control module allows you to manipulate it via a 3D transformation gizmo (sphere). Dragging this gizmo will trigger an IK request, and the robot model in Slicer will update in real-time to match the solution.

If the solver fails (e.g., the target is out of reach), the robot visualization will turn **red**.

Trajectory Planning
~~~~~~~~~~~~~~~~~~~

Once a goal pose is reached via IK, you can use the "Plan" button to generate a trajectory. Slicer will display a scrubber allowing you to preview the plan. Enable "Show trajectory path" to display the planned end-effector path as a ``vtkPolyLine`` model in the scene.

*   **Generators**: Supports multiple planning backends, including MoveIt and basic joint interpolation.
*   **Planning Group**: Specify the MoveIt planning group (e.g., ``ur_manipulator``) to use for planning.

Python scripts can also bypass the GUI and plan a Cartesian path through a list
of Slicer 3D pose markers.  Use
``ROS2MotionControlLogic.CreatePoseMarker`` to create transform-node markers,
then call ``PlanMoveItCartesianTrajectoryFromPoseMarkers``.  See
:doc:`/pages/examples/moveit_pose_markers`.
After loading a planned trajectory into the module widget, scripts can call
``motionWidget.addPlannedTrajectoryPolyline()`` and
``motionWidget.removePlannedTrajectoryPolyline()`` to show or remove the same
end-effector path visualization.

Execution
~~~~~~~~~

The "Execute" button sends the planned trajectory to the robot's controller via the ``ExecuteTrajectory`` action.

MoveIt Scene Synchronization
----------------------------

The **Obstacles** tab allows you to use Slicer models as collision obstacles in MoveIt.

Adding Collision Objects
~~~~~~~~~~~~~~~~~~~~~~~~

* **Select a Model**: Choose a ``vtkMRMLModelNode`` from the dropdown.
* **Set Frame ID**: Specify the ROS reference frame (e.g., ``world`` or ``base_link``).
* **Add to MoveIt**: Click the **Add to MoveIt** button.

Once added, the obstacle is managed in the table below.

Live Synchronization
~~~~~~~~~~~~~~~~~~~~

A powerful feature of this integration is **Live Sync**. Because SlicerROS2 observes the Slicer scene, any movement of the model node (e.g., via a transform gizmo or a tracking system) is immediately pushed to the MoveIt planning scene. MoveIt will then account for these moving obstacles in all future planning requests.

Removing Obstacles
~~~~~~~~~~~~~~~~~~

To remove an obstacle from the MoveIt scene, simply click the **Remove** button in the obstacles table.

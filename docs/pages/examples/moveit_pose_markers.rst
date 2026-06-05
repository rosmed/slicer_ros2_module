MoveIt Cartesian Path from Pose Markers
=======================================

This example shows a Python-only Motion Control workflow:

* create Slicer 3D pose markers using ``vtkMRMLLinearTransformNode``;
* collect those markers in a Python list;
* ask MoveIt's ``/compute_cartesian_path`` service to compute a Cartesian
  trajectory through each pose;
* load the returned trajectory into the Motion Control GUI for preview,
  scrubbing, and execution.

Transform nodes are used as the primary marker type because they carry a full
6-DOF pose.  Slicer Markups control points can also be converted to poses with
``GetPoseMatricesFromMarkups``, but those points only provide position unless
you supply an orientation reference.

Prerequisites
-------------

Start a robot with MoveIt, for example:

.. code-block:: bash

   ros2 launch slicer_ros2_module ur_sim_control.launch.py launch_rviz:=true

Then run the example script:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/moveit_pose_markers.py

Planning Through Markers
------------------------

The relevant script section is:

.. literalinclude:: ../../code/moveit_pose_markers.py
   :language: python
   :start-at: # Build a Python list

The script first waits for a valid live ``/joint_states`` message.  The marker
list then starts with a lead-in segment for feasibility: ``PathPose_0`` is
anchored to the current tip pose, and ``PathPose_1`` is the same pose lowered
by 100 mm.  The remaining markers build the XY path at that lowered height.
Planning uses MoveIt's current state, without forcing an explicit start state.
The script also creates small sphere visuals attached to each marker transform,
so marker positions stay easy to see while editor widgets are hidden. To edit one marker
interactively, enable its editor widget:

.. code-block:: python

   marker = slicer.util.getNode("PathPose_1")
   marker.GetDisplayNode().SetEditorVisibility(True)

After moving a marker, rerun the planning block from the Python console.  The
marker list can also be built entirely by another Python algorithm:

.. code-block:: python

   poseMarkers = [
       slicer.util.getNode("PathPose_0"),
       slicer.util.getNode("PathPose_1"),
       slicer.util.getNode("PathPose_2"),
   ]

   trajectory = motionLogic.PlanMoveItCartesianTrajectoryFromPoseMarkers(
       motionControlNode,
       PLANNING_GROUP,
       poseMarkers,
       relativeToNode=rootTransform,
       robotNode=robotNode,
   )

   fraction = motionControlNode.GetLastCartesianPathFraction()

``fraction`` is the fraction returned by MoveIt's Cartesian path planner.  A
value less than ``1.0`` means MoveIt returned only a partial path.

Execution safety
----------------

The example requires live ``/joint_states``.  If no valid joint-state message
arrives within the timeout, the script raises an error instead of planning from
guessed joint values.

When planning succeeds, the script selects the Motion Control module and loads
the trajectory into the same widget state used by the Plan button.  Use the
GUI Preview button or trajectory scrubber to inspect the result before pressing
Execute.  The Python example intentionally does not execute the trajectory
directly.

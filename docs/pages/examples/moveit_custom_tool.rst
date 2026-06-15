MoveIt Custom Tool with Needle Tip
==================================

This example extends the UR MoveIt workflow from
:doc:`/pages/examples/moveit_obstacle` by attaching a runtime tool to the robot
without editing the URDF or SRDF.  The tool is a 100 mm needle represented as a
MoveIt ``AttachedCollisionObject``.  It is attached to the UR ``tool0`` link and
defines a MoveIt object subframe named ``tip`` at the distal end of the needle.

The complete script also creates a simple world obstacle using the same pattern
as the previous example, then attaches the needle and prints the custom tip
frame name ``SlicerNeedle/tip``.  The script refreshes the existing
**End effector link** selector in Motion Control and selects that tip.

Prerequisites
-------------

Start the UR mock-hardware MoveIt launch:

.. code-block:: bash

   ros2 launch slicer_ros2_module ur_sim_control.launch.py launch_rviz:=true

Then run the example script:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/moveit_custom_tool.py

Needle Tool
-----------

The obstacle setup is intentionally not repeated here; see
:doc:`/pages/examples/moveit_obstacle` for that pattern.  The new part is the
runtime tool attachment:

.. literalinclude:: ../../code/moveit_custom_tool.py
   :language: python
   :start-after: [doc: needle]
   :end-before: [end: needle]

After this block runs, MoveIt collision checking treats the needle as part of
the robot.  Planning requests that support attached-object subframes can use
``needleTipLink`` as the active Cartesian link name.  For example, the scripted
Cartesian helper accepts it through the ``linkName`` argument when you want
waypoints to refer to the needle tip instead of the robot ``tool0`` frame.

The Slicer model is also parented under the robot's ``tool0`` transform, so the
needle appears at the UR tool in the 3D view.

Notes
-----

The needle is not a URDF link.  It is a planning-scene object attached at
runtime.  This is useful for swapping instruments without relaunching MoveIt,
while still allowing the URDF/SRDF path for tools that should be permanent
robot model links.

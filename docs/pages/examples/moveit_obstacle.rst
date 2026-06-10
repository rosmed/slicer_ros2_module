MoveIt Obstacle from a Slicer Model
=====================================

This example demonstrates how to create a geometric obstacle directly in 3D Slicer
using VTK/MRML Python commands and push it to the MoveIt 2 planning scene so that
motion planning (IK and trajectory generation) avoids it.

The obstacle is positioned via a **MRML LinearTransformNode** (the *registration
transform*), so you can move it interactively with the 3D gizmo or update it
programmatically from a registration algorithm — MoveIt's planning scene updates
live without re-publishing the geometry.  This mirrors the clinical workflow of
registering anatomy to a robot coordinate system.

Prerequisites
-------------

You need a running UR5 simulation with MoveIt and the Slicer launcher.
Open **two terminals** and source your workspace in each:

.. code-block:: bash

   # Terminal 1 – UR5 with mock hardware + MoveIt
   source ~/ros2_ws/install/setup.bash
   ros2 launch slicer_ros2_module ur_sim_control.launch.py

.. code-block:: bash

   # Terminal 2 – 3D Slicer
   source ~/ros2_ws/install/setup.bash
   ros2 run slicer_ros2_module slicer

Once Slicer is open, load the robot via the **ROS2** module as described in
:ref:`load_robot`. Wait for the ``[move_group] You can start planning now!``
message in Terminal 1 before proceeding.

.. note::

   All coordinates in the Slicer Python interactor are in **millimetres** (Slicer's
   native unit). The SlicerROS2 module automatically converts them to metres when
   publishing to ROS 2.

You can copy/paste each section of the code below into the Slicer Python
interpreter, or run the complete script from the terminal using the
``--python-script`` argument of the ``slicer`` launcher — the full command
is provided at the end of this example.

Configuration
-------------

All parameters are gathered in a single block at the top of the script.
Edit these values to match your robot and MoveIt setup before running
any of the sections below:

.. literalinclude:: ../../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: config]
   :end-before: [end: config]

Setting Up the Robot and MoveIt
--------------------------------

The following code performs the same steps you would normally carry out
by hand in the **ROS2 Motion Control** module UI. By using a ``ParameterNode``,
this scripted setup synchronizes with the Slicer UI, allowing you
to switch between code and manual interaction:

* Create the robot node from the URDF on the ROS 2 parameter server.
* Wait asynchronously for the URDF to be parsed.
* Delegate the motion-control initialization to the module's logic class.

.. literalinclude:: ../../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: setup]
   :end-before: [end: setup]

Once the code prints *"Setup complete"* you can proceed to create and
publish obstacles.

Creating the Obstacle Geometry
--------------------------------

The following code creates a 200 mm × 200 mm × 200 mm box **centred at the
origin**.  The position is intentionally not baked into the mesh — it will
be controlled entirely by the transform node in the next step:

.. literalinclude:: ../../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: geometry]
   :end-before: [end: geometry]

You should see an orange semi-transparent cube at the world origin.

.. note::

   The node **name** (here ``MoveItObstacle``) becomes the collision-object ``id``
   in MoveIt **and** the ``child_frame_id`` in the TF2 broadcast.  Use a unique,
   descriptive name for each obstacle so that MoveIt can track and remove them
   individually.

Placing the Obstacle with a MRML Transform
-------------------------------------------

Instead of baking a position into the mesh, we create a
``vtkMRMLLinearTransformNode`` (the *registration transform*) and parent the
obstacle under it.  This is equivalent to registering a piece of anatomy to the
robot coordinate system:

.. literalinclude:: ../../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: transform]
   :end-before: [end: transform]

After running this section the cube appears at (400, 0, 300) mm in the 3D view.
A coloured gizmo is overlaid on it — drag it to move the obstacle anywhere in
the scene.

Publishing the Obstacle and Starting the TF2 Broadcast
-------------------------------------------------------

The single call to ``AddMoveItObstacleWithTransform`` does everything needed
to integrate the obstacle with MoveIt:

1. **Publishes a** ``CollisionObject`` whose ``header.frame_id`` is set to the
   obstacle's Slicer node name (``"MoveItObstacle"``).  MoveIt looks up that
   frame in the TF2 tree to find the obstacle's location.
2. **Creates a** ``Tf2BroadcasterNode`` with ``parent = "world"`` and
   ``child = "MoveItObstacle"``.
3. **Observes the MRML transform** so every time you move the gizmo (or a
   registration algorithm writes to the transform) the new pose is broadcast
   over ``/tf`` automatically — no re-publish needed.

.. literalinclude:: ../../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: publish]
   :end-before: [end: publish]

.. note::

   ``/collision_object`` is the ROS 2 / MoveIt 2 standard topic for
   ``moveit_msgs/msg/CollisionObject`` messages.  The ``move_group`` node's
   ``PlanningSceneMonitor`` subscribes to it and updates its internal
   planning scene upon receipt.

   Setting ``frame_id`` to the obstacle name (rather than ``"world"``) lets
   MoveIt track the obstacle through the TF2 tree.  This means dragging the
   gizmo in Slicer's 3D view is enough to move the obstacle in MoveIt — the
   geometry message only needs to be sent once.

   The Motion Control **Obstacles** tab provides the same behavior when
   **Track model transform with TF2** is checked.  In that mode, the
   **Frame ID / TF2 Parent** field is the parent frame for the broadcaster,
   not the final collision-object frame.

Running the Complete Example from the Terminal
-----------------------------------------------

To run all sections as a single unattended script (Slicer starts, loads
the robot, creates and publishes the obstacle, then stays open):

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/moveit_obstacle.py

Verifying the Obstacle in MoveIt
----------------------------------

Verify that MoveIt received the obstacle in a third terminal:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
       "{components: {components: 24}}"

Look for an entry whose ``id`` matches ``MoveItObstacle`` in the response's
``world.collision_objects`` list.

The value ``24`` requests MoveIt's ``WORLD_OBJECT_NAMES`` and
``WORLD_OBJECT_GEOMETRY`` components (``8 + 16``). Requesting only component
``1`` returns scene settings, so the ``world.collision_objects`` field may look
empty even when the object has been received.

To verify the live TF2 broadcast:

.. code-block:: bash

   ros2 topic echo /tf | grep -A5 "MoveItObstacle"

Drag the gizmo in the 3D Slicer's 3D view and watch the translation values change
in the terminal output.

Alternatively, open RViz (``ros2 launch slicer_ros2_module ur_sim_control.launch.py launch_rviz:=true``),
add a **PlanningScene** display, and the orange box will appear in the 3D view
alongside the robot.  Move it with the Slicer gizmo and watch it update live
in RViz.

From this point, any IK or trajectory-planning request issued through the
**ROS2 Motion Control** module (or directly via MoveIt) will treat the box as a
hard collision constraint and plan around it.

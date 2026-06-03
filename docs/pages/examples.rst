Examples
========

This page provides various step-by-step examples demonstrating how to use SlicerROS2 to connect 3D Slicer with ROS 2 workflows.

For background on the APIs used by these examples, see
:doc:`/pages/nodes/topics`, :doc:`/pages/nodes/images_pointclouds`,
:doc:`/pages/robot-visualization`, and :doc:`/pages/motion-control`.

PointCloud subscriber
---------------------

This example demonstrates how to stream a simulated point cloud from a terminal node using a ROS 2 topic, and visualize it dynamically in 3D Slicer.

Running the Point Cloud Publisher
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We have provided a demo script, ``demo_point_cloud.py``, which publishes a 10x20 point cloud simulating a 2D wavelet at 10Hz.

To run the publisher, first make sure you have sourced your ROS 2 environment and built the workspace. Then run:

.. code-block:: bash

   # Source the workspace setup
   source ~/ros2_ws/install/setup.bash

   # Run the demo publisher script
   ros2 run slicer_ros2_module demo_point_cloud.py

The publisher will start generating the simulated point cloud and publishing it over the ROS topic ``/simulated_point_cloud``.

Visualizing in 3D Slicer
~~~~~~~~~~~~~~~~~~~~~~~~

Open the 3D Slicer application using the recommended launcher ``ros2 run slicer_ros2_module slicer``.

You can set up the subscriber in one of two ways:

**Option A – run the script directly from a terminal** using the ``slicer`` launcher's
``--python-script`` argument (the script executes automatically after Slicer starts):

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/point_cloud_subscriber.py

**Option B – paste the code** into Slicer's Python Interactor (Ctrl + 3 or through the
**Developer Tools** menu):

.. literalinclude:: ../code/point_cloud_subscriber.py
   :language: python

Once run, you will see a 10x20 grid of points in the 3D view representing a dynamic 2D wavelet propagating outwards from the center.

PointCloud publisher
--------------------

This example demonstrates how to create a static point cloud using 3D Slicer's built-in features and publish it over a ROS 2 topic to be visualized in RViz.

Creating and Publishing the Point Cloud in 3D Slicer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can run the publisher script in one of two ways:

**Option A – run the script directly from a terminal** using the ``slicer`` launcher's
``--python-script`` argument (the script executes automatically after Slicer starts):

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/point_cloud_publisher.py

**Option B – paste the code** into Slicer's Python Interactor (**Ctrl + 3** or through the
**Developer Tools** menu):

.. literalinclude:: ../code/point_cloud_publisher.py
   :language: python

Once executed, Slicer will create a local model displaying the sphere, and publish the points to the ROS 2 topic ``/slicer_point_cloud`` with the ``frame_id`` set to ``world``.

Visualizing in RViz
~~~~~~~~~~~~~~~~~~~

To visualize this published point cloud in RViz:

* **Open a terminal**, source the ROS 2 setup script, and launch RViz:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      rviz2

* **Configure RViz Display**:
   - In the **Global Options** panel, set the **Fixed Frame** to ``world``.
   - Click the **Add** button at the bottom left of the window.
   - Switch to the **By topic** tab, expand ``/slicer_point_cloud``, select **PointCloud2**, and click **OK**.
   - (Optional) In the newly added PointCloud2 display properties, increase the **Size (m)** parameter to ``0.2`` or larger to render the sphere points clearly in the 3D grid.

MoveIt Obstacle from a Slicer Model
-----------------------------------

This example demonstrates how to create a geometric obstacle directly in 3D Slicer
using VTK/MRML Python commands and push it to the MoveIt 2 planning scene so that
motion planning (IK and trajectory generation) avoids it.

The obstacle is positioned via a **MRML LinearTransformNode** (the *registration
transform*), so you can move it interactively with the 3D gizmo or update it
programmatically from a registration algorithm — MoveIt's planning scene updates
live without re-publishing the geometry.  This mirrors the clinical workflow of
registering anatomy to a robot coordinate system.

Prerequisites
~~~~~~~~~~~~~

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
~~~~~~~~~~~~~

All parameters are gathered in a single block at the top of the script.
Edit these values to match your robot and MoveIt setup before running
any of the sections below:

.. literalinclude:: ../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: config]
   :end-before: [end: config]

Setting Up the Robot and MoveIt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following code performs the same steps you would normally carry out
by hand in the **ROS2 Motion Control** module UI. By using a ``ParameterNode``,
this scripted setup synchronizes with the Slicer UI, allowing you
to switch between code and manual interaction:

* Create the robot node from the URDF on the ROS 2 parameter server.
* Wait asynchronously for the URDF to be parsed.
* Delegate the motion-control initialization to the module's logic class.

.. literalinclude:: ../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: setup]
   :end-before: [end: setup]

Once the code prints *"Setup complete"* you can proceed to create and
publish obstacles.

Creating the Obstacle Geometry
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following code creates a 200 mm × 200 mm × 200 mm box **centred at the
origin**.  The position is intentionally not baked into the mesh — it will
be controlled entirely by the transform node in the next step:

.. literalinclude:: ../code/moveit_obstacle.py
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
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instead of baking a position into the mesh, we create a
``vtkMRMLLinearTransformNode`` (the *registration transform*) and parent the
obstacle under it.  This is equivalent to registering a piece of anatomy to the
robot coordinate system:

.. literalinclude:: ../code/moveit_obstacle.py
   :language: python
   :start-after: [doc: transform]
   :end-before: [end: transform]

After running this section the cube appears at (400, 0, 300) mm in the 3D view.
A coloured gizmo is overlaid on it — drag it to move the obstacle anywhere in
the scene.

Publishing the Obstacle and Starting the TF2 Broadcast
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

.. literalinclude:: ../code/moveit_obstacle.py
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

Running the Complete Example from the Terminal
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To run all sections as a single unattended script (Slicer starts, loads
the robot, creates and publishes the obstacle, then stays open):

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/moveit_obstacle.py

Verifying the Obstacle in MoveIt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

Drag the gizmo in the Slicer 3D view and watch the translation values change
in the terminal output.

Alternatively, open RViz (``ros2 launch slicer_ros2_module ur_sim_control.launch.py launch_rviz:=true``),
add a **PlanningScene** display, and the orange box will appear in the 3D view
alongside the robot.  Move it with the Slicer gizmo and watch it update live
in RViz.

From this point, any IK or trajectory-planning request issued through the
**ROS2 Motion Control** module (or directly via MoveIt) will treat the box as a
hard collision constraint and plan around it.

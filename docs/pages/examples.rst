""""""""
Examples
""""""""

This page provides various step-by-step examples demonstrating how to use SlicerROS2 to connect 3D Slicer with ROS 2 workflows.

=====================
PointCloud subscriber
=====================

This example demonstrates how to stream a simulated point cloud from a terminal node using a ROS 2 topic, and visualize it dynamically in 3D Slicer.

Running the Point Cloud Publisher
=================================

We have provided a demo script, ``demo_point_cloud.py``, which publishes a 10x20 point cloud simulating a 2D wavelet at 10Hz.

To run the publisher, first make sure you have sourced your ROS 2 environment and built the workspace. Then run:

.. code-block:: bash

   # Source the workspace setup
   source ~/ros2_ws/install/setup.bash

   # Run the demo publisher script
   ros2 run slicer_ros2_module demo_point_cloud.py

The publisher will start generating the simulated point cloud and publishing it over the ROS topic ``/simulated_point_cloud``.

Visualizing in 3D Slicer
========================

Open the 3D Slicer application (launched using the recommended launcher ``ros2 launch slicer_ros2_module slicer.launch.py``).

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

====================
PointCloud publisher
====================

This example demonstrates how to create a static point cloud using 3D Slicer's built-in features and publish it over a ROS 2 topic to be visualized in RViz.

Creating and Publishing the Point Cloud in 3D Slicer
====================================================

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
===================

To visualize this published point cloud in RViz:

1. **Open a terminal**, source the ROS 2 setup script, and launch RViz:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      rviz2

2. **Configure RViz Display**:
   - In the **Global Options** panel, set the **Fixed Frame** to ``world``.
   - Click the **Add** button at the bottom left of the window.
   - Switch to the **By topic** tab, expand ``/slicer_point_cloud``, select **PointCloud2**, and click **OK**.
   - (Optional) In the newly added PointCloud2 display properties, increase the **Size (m)** parameter to ``0.2`` or larger to render the sphere points clearly in the 3D grid.

===================================
MoveIt Obstacle from a Slicer Model
===================================

This example demonstrates how to create a geometric obstacle directly in 3D Slicer
using VTK/MRML Python commands and push it to the MoveIt 2 planning scene so that
motion planning (IK and trajectory generation) avoids it.

Prerequisites
=============

You need a running UR5 simulation with MoveIt and the Slicer launcher.
Open **two terminals** and source your workspace in each:

.. code-block:: bash

   # Terminal 1 – UR5 with mock hardware + MoveIt
   source ~/ros2_ws/install/setup.bash
   ros2 launch slicer_ros2_module ur_sim_control.launch.py

.. code-block:: bash

   # Terminal 2 – 3D Slicer
   source ~/ros2_ws/install/setup.bash
   ros2 launch slicer_ros2_module slicer.launch.py

Once Slicer is open, load the robot via the **ROS2** module as described in
:ref:`load_robot`. Wait for the ``[move_group] You can start planning now!``
message in Terminal 1 before proceeding.

.. note::

   All coordinates in the Slicer Python interactor are in **millimetres** (Slicer's
   native unit). The SlicerROS2 module automatically converts them to metres when
   publishing to ROS 2.

Creating the Obstacle
=====================

Open the Slicer Python Interactor (**Ctrl + 3** or through the
**Developer Tools** menu) and paste the following snippet.

It creates a 200 mm × 200 mm × 200 mm box positioned in front of the UR5
base (400 mm along X, 300 mm up along Z — well within the robot's reach envelope):

.. code-block:: python

   import slicer
   import vtk

   # 1. Build a box geometry with vtkCubeSource (dimensions in mm)
   cube = vtk.vtkCubeSource()
   cube.SetXLength(200.0)
   cube.SetYLength(200.0)
   cube.SetZLength(200.0)
   cube.SetCenter(400.0, 0.0, 300.0)
   cube.Update()

   # 2. Wrap it in a vtkMRMLModelNode so it appears in the Slicer scene
   obstacleNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'MoveItObstacle')
   obstacleNode.SetAndObservePolyData(cube.GetOutput())
   obstacleNode.CreateDefaultDisplayNodes()

   # Style it so it is clearly visible in the 3D view
   dispNode = obstacleNode.GetDisplayNode()
   dispNode.SetColor(1.0, 0.5, 0.0)   # orange
   dispNode.SetOpacity(0.6)

   print("Obstacle node created:", obstacleNode.GetName())

You should see an orange semi-transparent cube appear in the 3D view.

.. note::

   The node **name** (here ``MoveItObstacle``) becomes the collision-object ``id``
   in MoveIt. Use a unique, descriptive name for each obstacle so that MoveIt can
   track and remove them individually.

Publishing the Obstacle to MoveIt
==================================

Still in the Python Interactor, paste the second snippet to create a
``CollisionObject`` publisher and push the obstacle to the ``/planning_scene`` topic:

.. code-block:: python

   # 3. Get the default SlicerROS2 node
   rosLogic = slicer.util.getModuleLogic('ROS2')
   rosNode = rosLogic.GetDefaultROS2Node()

   # 4. Create the CollisionObject publisher
   #    '/planning_scene' is the standard topic that move_group's
   #    PlanningSceneMonitor subscribes to for incremental scene diffs.
   pub = rosNode.CreateAndAddPublisherNode('CollisionObject', '/planning_scene')

   # 5. Point the publisher at our obstacle model and set the reference frame
   pub.SetSourceNodeID(obstacleNode.GetID())
   pub.SetFrameId('world')   # must match the MoveIt fixed frame

   # 6. Publish once – MoveIt adds it to its internal planning scene
   pub.Publish()

   print("Obstacle published to MoveIt planning scene.")

.. note::

   ``/planning_scene`` is the ROS 2 / MoveIt 2 standard topic for planning-scene
   diffs. The ``move_group`` node's ``PlanningSceneMonitor`` subscribes to it and
   updates its internal representation upon receipt. This is equivalent to what
   MoveIt's ``PlanningSceneInterface::applyCollisionObject()`` does under the hood.

Verifying the Obstacle in MoveIt
================================

You can verify that MoveIt received the obstacle in a third terminal:

.. code-block:: bash

   source ~/ros2_ws/install/setup.bash
   ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene \
       "{components: {components: 1}}"

Look for an entry whose ``id`` matches ``MoveItObstacle`` in the response's
``world.collision_objects`` list.

Alternatively, open RViz (``ros2 launch slicer_ros2_module ur_sim_control.launch.py launch_rviz:=true``),
add a **PlanningScene** display, and the orange box will appear in the 3D view
alongside the robot.

From this point, any IK or trajectory-planning request issued through the
**ROS2 Motion Control** module (or directly via MoveIt) will treat the box as a
hard collision constraint and plan around it.

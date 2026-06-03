PointCloud Publisher
====================

This example demonstrates how to create a static point cloud using 3D Slicer's
built-in features and publish it over a ROS 2 topic to be visualized in RViz.

Creating and Publishing the Point Cloud in 3D Slicer
-----------------------------------------------------

You can run the publisher script in one of two ways:

**Option A – run the script directly from a terminal** using the ``slicer`` launcher's
``--python-script`` argument (the script executes automatically after Slicer starts):

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --python-script $(ros2 pkg prefix slicer_ros2_module)/share/slicer_ros2_module/docs/code/point_cloud_publisher.py

**Option B – paste the code** into Slicer's Python Interactor (**Ctrl + 3** or through
the **Developer Tools** menu):

.. literalinclude:: ../../code/point_cloud_publisher.py
   :language: python
   :end-before: [end: code]

Once executed, Slicer will create a local model displaying the sphere, and publish
the points to the ROS 2 topic ``/slicer_point_cloud`` with the ``frame_id`` set to
``world``.

Visualizing in RViz
--------------------

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

"""""
Tests
"""""

============
Introduction
============

We provide some unit tests for the Slicer ROS module.  The unit tests
rely on the automatically generated Python wrapper to to test the C++
MRML nodes.  These tests assumes you've installed the default ROS2
desktop packages and use the turtlesim examples provided along ROS2.

To test the MRML ROS2 parameter node, the ROS2 turtlesim nodes are
started in the background so you will see some windows popping up.

To test the subscribers and publishers, the unit tests subscribe and
publish to the same topic so we can send and receive from the same
node.  This also indirectly tests the conversion methods.

Finally, for Tf2 lookups and broadcasts, we use the strategy used to
test the subscribers and publishers.  The test broadcasts a known
transform and uses a lookup to retrieve the value of the Tf2 buffer.

======================
Running the unit tests
======================

Interactive (Within Slicer)
---------------------------

You can run the tests interactively inside Slicer by opening the Python Console (``Ctrl+3``) and running:

.. code-block:: python

   tests = slicer.util.getModuleLogic('ROS2Tests')
   tests.run()

The tests intentionally attempt to perform commands that should fail, so you will see some warning and error messages displayed in the Slicer Python Console.

Using CTest
-----------

To run the unit tests automatically from the command line using CTest, source your setup files and run ``ctest`` pointing to the module's build directory:

.. code-block:: bash

   # Source your ROS 2 and workspace setup files
   source /opt/ros/<distro>/setup.bash
   source <workspace_install_dir>/setup.bash

   # Run ctest pointing to the build directory of slicer_ros2_module
   ctest --test-dir build/slicer_ros2_module -R py_ROS2Tests --verbose

.. note::
   When running via CTest or Colcon, you might see ``vtkDebugLeaks`` warning messages on shutdown after the tests have successfully completed. Slicer's CTest runner may report this leak as a test failure even if all individual unit tests passed successfully. You can verify that all individual tests passed by reviewing the console output (``Ran 23 tests ... OK``).

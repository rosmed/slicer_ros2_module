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
started in the backgound so you will see some windows popping up.

To test the subscribers and publishers, the unit tests subscribe and
publish to the same topic so we can send and receive from the same
node.  This also indirectly tests the conversion methods.

Finally, for Tf2 lookups and broadcasts, we use the strategy used to
test the subscribers and publishers.  The test boardcasts a known
transform and uses a lookup to retrieve the value of the Tf2 buffer.

======================
Running the unit tests
======================

.. code-block:: python

   tests = slicer.util.getModuleLogic('ROS2Tests')
   tests.run()

The tests intentionally attempts to perform commands that should fail
therefore you will see a few error and error messages displayed in the
Python console.  To see the result of the tests, you will have to
scroll up.

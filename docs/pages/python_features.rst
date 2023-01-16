===========
3D Slicer python interactor commands
===========

The following commands can be used in the python interactor in 3D Slicer to test the modules functionality.

Get the Slicer ROS2 Module logic to access functions:

.. code-block:: python

    ros2 = slicer.util.getModuleLogic('ROS2')

Create a subscriber/ publisher in python:

.. code-block:: python

    ros2 = slicer.util.getModuleLogic('ROS2')
    ros2.CreateAndAddPublisher('vtkMRMLROS2PublisherStringNode', '/new_pub')
    # or
    ros2.CreateAddAddSubscriber('vtkMRMLROS2SubscriberStringNode', '/new_sub')

Get a subscriber / publisher by topic:

.. code-block:: python

    ## Get the node
    node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:testNode')
    sub = node.GetSubscriberNodeByTopic('/string_sub')
    # or
    pub = node.GetPublisherNodeByTopic('/new_pub')

Print the last message:

.. code-block:: python

    sub.GetLastMessageYAML()

Publish from python:

.. code-block:: python

    pub.Publish('String message') # native implementation
    # To test the vtk implementation try the following
    ros2 = slicer.util.getModuleLogic('ROS2')
    pub = ros2.CreateAndAddPublisher('vtkMRMLROS2PublisherPoseStampedNode', '/pose_pub')
    # run ros2 topic echo /pose_pub in a terminal to see the output
    mat = vtk.vtkMatrix4x4()
    mat.SetElement(0,3,100) # Modify the matrix so we can see something changing
    pub.Publish(mat)

Print the node to see references

.. code-block:: python

    # Get the node
    node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:testNode')
    print(node)

Run the simple test script

.. code-block:: python

    # First import and run
    import slicer_ros2_module_test
    slicer_ros2_module_test.run()
    # Reload the script if it was modified, don't forget colcon build
    from importlib import reload
    reload(slicer_ros2_module_test)
    # Run the tests again
    slicer_ros2_module_test.run()

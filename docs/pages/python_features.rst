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

    pub.Publisher('String message')

Print the node to see references

.. code-block:: python

    ## Get the node
    node = slicer.mrmlScene.GetFirstNodeByName('ros2:node:testNode')
    print(node)




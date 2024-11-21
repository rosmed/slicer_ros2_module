======
Topics
======

In ROS, publishers and subscribers can send/receive any type of ROS
message (defined in `.msg` files).  These `.msg` files are then parsed
by a code generator that creates the C++ code needed to support said
message.  All the classes and functions needed for ROS topics can then
be templated and uses the "type traits pattern" since all the messages
have a similar API.  On the other hand, Slicer tends to avoid template
for end-user classes.  So we created a set of basic publishers and
subscribers to convert messages between ROS and Slicer.

.. list-table:: Publishers and subscribers
   :widths: 30 40 30
   :header-rows: 1

   * - Slicer type
     - ROS type
     - SlicerROS2 "name"
   * - std::string
     - std_msgs::msg::String
     - String
   * - bool
     - std_msgs::msg::Bool
     - Bool
   * - int
     - std_msgs::msg::Int64
     - Int
   * - double
     - std_msgs::msg::Float64
     - Double;
   * - vtkIntArray
     - std_msgs::msg::Int64MultiArray
     - IntArray
   * - vtkDoubleArray
     - std_msgs::msg::Float64MultiArray
     - DoubleArray
   * - vtkTable
     - std_msgs::msg::Int64MultiArray
     - IntTable
   * - vtkTable
     - std_msgs::msg::Float64MultiArray
     - DoubleTable
   * - vtkMatrix4x4
     - geometry_msgs::msg::Pose
     - Pose
   * - vtkDoubleArray
     - geometry_msgs::msg::WrenchStamped
     - WrenchStamped
   * - vtkTransformCollection
     - geometry_msgs::msg::PoseArray
     - PoseArray
   * - vtkUInt8Array
     - sensor_msgs::msg::Image
     - UInt8Image
   * - vtkPoints
     - sensor_msgs::msg::PointCloud
     - PointCloud

For example, if you need to create a publisher that will take a
`vtkMatrix4x4` on the Slicer side and publish a
`geometry_msgs::msg::PoseStamped` on the ROS side, the full SlicerROS2
node name will be `vtkMRMLROSPublisherPoseStampedNode`.

To find the current list of supported publishers and subscribers, one can do:

.. code-block:: python

   rosLogic = slicer.util.getModuleLogic('ROS2')
   rosNode = rosLogic.GetDefaultROS2Node()
   rosNode.RegisteredROS2PublisherNodes()
   rosNode.RegisteredROS2SubscriberNodes()

The method ``RegisteredROS2xxxxxNodes`` returns a long string with all the
publisher or subscriber classes available.

.. _publishers:

Publishers
==========

To create a new publisher, one should use the MRML ROS2 Node method
``vtkMRMLROS2NodeNode::CreateAndAddPublisherNode``.  This method takes
two parameters:

* the class (type) of publisher to be used (full or short name).  We
  provide some publishers for the most common data types (from Slicer
  to ROS messages).  The full list can be found in the Slicer ROS
  logic class (``Logic/vtkSlicerROS2Logic.cxx``) in the method
  ``RegisterNodes``.
* the topic name (``std::string``).

Publishers are triggered by calling the ``Publish`` method.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         # optional, shows which publishers are available
         rosNode.RegisteredROS2PublisherNodes()
         # example with full class name
         pubString = rosNode.CreateAndAddPublisherNode('vtkMRMLROS2PublisherStringNode', '/my_string')
         # run `ros2 topic echo /my_string` in a terminal to see the output
         pubString.Publish('my first string')

         # example with short class name, Pose will be expended to vtkMRMLROS2PublisherPoseNode
         pubMatrix = rosNode.CreateAndAddPublisherNode('Pose', '/my_matrix')
         # run `ros2 topic echo /my_matrix` in a terminal to see the output
         mat = pubMatrix.GetBlankMessage() # returns a vtkMatrix4x4
         mat.SetElement(0, 3, 3.1415) # Modify the matrix so we can see something changing
         pubMatrix.Publish(mat)

   .. tab:: **C++**

      .. code-block:: C++

         // example with full class name
         auto pubString = rosNode->CreateAndAddPublisherNode("vtkMRMLROS2PublisherStringNode", "/my_string");
         // run ros2 topic echo /my_string in a terminal to see the output
         pubString->Publish("my first string");

         // example with short class name, String will be expended to vtkMRMLROS2PublisherStringNode
         auto pubString2 = rosNode->CreateAndAddPublisherNode("String", "/my_second_string");


To remove the publisher node, use the method ``vtkMRMLROS2NodeNode::RemoveAndDeletePublisherNode``. This method takes
one parameter:

* the topic name (``std::string``)


Subscribers
===========

To create a new subscriber, one should use the MRML ROS2 Node method
``vtkMRMLROS2NodeNode::CreateAndAddSubscriberNode``.  This method
takes two parameters:

* the class (type) of subscriber to be used.  See ::ref:`publishers`.
* the topic name (``std::string``).

Subscriber nodes get updated when the ROS2 node is spun.  Users can
set their own callback to act on newly received messages using an
observer on the MRML ROS subscriber node.  The last message received
can be retrieved using ``GetLastMessage``.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         # optional, shows which subscribers are available
         rosNode.RegisteredROS2SubscriberNodes()
         subString = rosNode.CreateAndAddSubscriberNode('String', '/my_string')
         # run `ros2 topic pub /my_string` in a terminal to send a string to Slicer
         m_string = subString.GetLastMessage()
         # alternate, get a string with the full message
         m_string_yaml = subString.GetLastMessageYAML()


         # since the subscriber is a MRML node,
	 # you can also create an observer (callback)
         # to trigger some code when a new message is received.
         # example callback function:
         def myCallback(caller=None, event=None):
            message = subString.GetLastMessage()
            print("Last message received by subscriber: {}.".format(message))
         # add the observer with callback
         observerId = subString.AddObserver('ModifiedEvent', myCallback)
         # the last message received will print in the python console
	 # in Slicer when data is published to /my_string


         # another example - updating transforms based on subscribed pose data
	 # (ie. for an optical tracker with a ros wrapper)
         subPose = rosNode.CreateAndAddSubscriberNode('Pose', '/StylusToTracker')
         transform = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLLinearTransformNode', 'StylusToTracker')
	 # define the callback
         def updateTransforms(caller=None, event=None):
            pose = subPose.GetLastMessage()
            transform.SetMatrixTransformToParent(pose)
            print("Last message received by subscriber: {}.".format(message))
         # add the observer with callback
         observerId = subPose.AddObserver('ModifiedEvent', updateTransforms)

   .. tab:: **C++**

      .. code-block:: C++

         auto subString = rosNode->CreateAndAddSubscriberNode("String", "/my_string");
         // run `ros2 topic pub /my_string` in a terminal to send a string to Slicer
         auto result = subString->GetLastMessage();

To remove the subscriber node, use the method ``vtkMRMLROS2NodeNode::RemoveAndDeleteSubscriberNode``. This method takes
one parameter:

* the topic name (``std::string``)

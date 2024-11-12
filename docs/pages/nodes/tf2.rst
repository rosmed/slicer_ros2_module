===
Tf2
===

For Tf2, there is no need to support multiple data types since Tf2's
API exclusively uses ``geometry_msgs::msg::TransformStamped``.  On the
Slicer side, the classes ``vtkMRMLROS2Tf2BroadcasterNode`` and
``vtkMRMLROS2Tf2LookupNode`` support both ``vtkMatrix4x4`` and
``vtkMRMLTransformNode``.

Tf2 lookups use a Tf2 buffer to store all the Tf2 messages
(broadcasts) sent by all the ROS nodes.  For the SlicerROS2 module, we
decided to add a Tf2 buffer as a private data member of the
``vtkMRMLROS2NodeNode`` since most users will never need a direct
access to the Tf2 buffer.  The Tf2 lookups are performed when the node
node is spun.

Broadcasts
==========

To create a new Tf2 broadcaster, one should use the MRML ROS2 Node
method ``vtkMRMLROS2NodeNode::CreateAndAddTf2BroadcasterNode``.  This
method takes two parameters:

* the parent ID (``std::string``)
* the child ID (``std::string``)

Broadcasters are triggered by calling the ``Broadcast`` method.  It is
also possible to set the Tf2 broadcast as an observer for an existing
``vtkMRMLTransformNode`` using the method ``ObserveTransformNode``.
The broadcast will then automatically occur when the observed transform
node is modified.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         broadcaster = ros2Node.CreateAndAddTf2BroadcasterNode('Parent', 'Child')
         # Broadcast a 4x4 matrix
         broadcastedMat = vtk.vtkMatrix4x4()
         broadcastedMat.SetElement(0, 3, 66.0) # Set a default value
         broadcaster.Broadcast(broadcastedMat)

   .. tab:: **C++**

      .. code-block:: C++

         auto broadcaster = rosNode.CreateAndAddTf2BroadcasterNode("Parent", "Child");
         # Broadcast a 4x4 matrix
         vtkSmartPointer<vtkMatrix4x4> broadcastedMat = vtkMatrix4x4::New();
         broadcastedMat->SetElement(0, 3, 66.0);
         broadcaster->Broadcast(broadcastedMat);

To remove the broadcaster node, use the method
``vtkMRMLROS2NodeNode::RemoveAndDeleteTf2BroadcasterNode``. This
method takes two parameters:

* the parent ID (``std::string``)
* the child ID (``std::string``)

Lookups
=======

To create a new Tf2 lookup, one should use the MRML ROS2 Node method
``vtkMRMLROS2NodeNode::CreateAndAddTf2LookupNode``.  This method takes
two parameters:

* the parent ID (``std::string``)
* the child ID (``std::string``)

The class ``vtkMRMLROS2Tf2LookupNode`` is derived from
``vtkMRMLTransformNode`` so it can be used as any other transformation
in the MRML scene.

Lookup nodes get updated when the ROS2 node is spun.  Users can set
their own callback to act on updated transformations using an observer
on the MRML ROS subscriber node.  The last transformation received can
be retrieved using ``GetMatrixTransformToParent``.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         lookupNode = ros2Node.CreateAndAddTf2LookupNode('Parent', 'Child')
         # get the transform "manually"
         lookupMat = lookupNode.GetMatrixTransformToParent()
         # or use an observer
         observerId = lookupNode.AddObserver(slicer.vtkMRMLTransformNode.TransformModifiedEvent, observer.Callback)

   .. tab:: **C++**

      .. code-block:: C++

         auto lookup = rosNode.CreateAndAddTf2LookupNode("Parent", "Child");
         # Broadcast a 4x4 matrix
         vtkSmartPointer<vtkMatrix4x4> lookupMat = vtkMatrix4x4::New();
         lookupMat->GetMatrixTransformToParent(lookupMat);

To remove the lookup node, the method
``vtkMRMLROS2NodeNode::RemoveAndDeleteTf2LookupNode``. This method
takes two parameters:

* the parent ID (``std::string``)
* the child ID (``std::string``)

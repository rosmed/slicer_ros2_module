=====
Nodes
=====

The SlicerROS2 module always creates a default ROS node (internally a
``rclcpp::node``).  This node is both a ROS node and a MRML node,
hence the unfortunate name ``vtkMRMLROS2NodeNode``.  This node is
added to the MRML scene and should be used to add your custom
``vtkMRMLROS2`` nodes (topics, parameter...).  It is possible to add
more ROS nodes in SlicerROS2 but this feature has not been tested
extensively for the first release.

To retrieve the default ROS node:

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()

   .. tab:: **C++**

      If you're modifying the Slicer ROS module, you can access the
      default ROS node directly using the ``GetDefaultROS2Node``
      method.

      .. code-block:: C++

         vtkMRMLROS2NodeNode * rosNode = this->GetDefaultROS2Node();

      If you're working on a new module that relies on the SlicerROS2
      module, you can use the ``GetModuleLogic`` method and then the
      ``GetDefaultROS2Node``.

      .. code-block:: C++

         vtkMRMLAbstractLogic * logic = this->GetModuleLogic("ROS2");
         if (logic == nullptr) {
           vtkErrorMacro(<< "ROS2 logic not found");
         } else {
          vtkSlicerROS2Logic * rosLogic =
               vtkSlicerROS2Logic::SafeDownCast(logic);
           if (rosLogic == nullptr) {
             vtkErrorMacro(<< "Found what should be the default ROS2 logic but the type is wrong");
           } else {
             vtkMRMLROS2NodeNode * rosNode = rosLogic->GetDefaultROS2Node();
             // now we can use the node
           }
         }

      For all other cases, you can use node ID of
      the default ROS node (``vtkMRMLROS2NodeNode1``) to retrieve it
      from the MRML scene.

      .. code-block:: C++

         vtkMRMLNode * node = scene->GetNodeByID("vtkMRMLROS2NodeNode1");
         if (node == nullptr) {
           vtkErrorMacro(<< "ROS2 default node not in scene");
         } else {
           vtkMRMLROS2NodeNode * rosNode =
               vtkMRMLROS2NodeNode::SafeDownCast(node);
           if (rosNode == nullptr) {
             vtkErrorMacro(<< "Found what should be the default ROS2 node but the type is wrong");
           } else {
             // now we can use the node
           }
         }

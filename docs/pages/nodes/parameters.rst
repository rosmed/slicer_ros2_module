==========
Parameters
==========

This version of SlicerROS2 only supports parameter clients,
i.e. retrieving parameters held by other ROS nodes (i.e. ROS processes
running along Slicer).

The parameter MRML ROS node is slightly different from the other MRML
ROS nodes implemented in SlicerROS2.  A ROS parameter is fully
identified by the ROS node *n* that hold the parameter and the
parameter name *p* so we could have an implementation that would
require one MRML ROS node for each parameter.  In practice, this can
lead to way too many MRML nodes.  For example, if you have two
parameters *p1* and *p2* held by the same ROS node *n1*, we would have
to create two MRML ROS nodes, *n1p1* and *n1p2*.  Since the ROS 2
libraries provide a single message to retrieve all the parameters help
by a single node, we decided to require one MRML ROS node per ROS node
observed.  This MRML ROS node can then observe all the parameters held
by the ROS node.

To create a new parameter node, one should use the MRML ROS2 Node
method ``vtkMRMLROS2NodeNode::CreateAndAddParameterNode``.  This
method takes one parameter:

* the name of the ROS node that holds the parameters you wish to
  retrieve (``std::string``).  This is an actual ROS node, running
  along Slicer.

Once the `vtkMRMLROS2ParameterNode` observing the ROS node *n1* is
added, one needs to specify which parameters to observe using the
method `vtkMRMLROS2ParameterNode::AddParameter`.

Parameter nodes get updated when the ROS2 node is spun.  Users can
set their own callback to act on newly received messages using an
observer on the MRML ROS parameter node.

ROS supports a limited number of types to encode parameters (`ROS2
parameters
<https://docs.ros.org/en/galactic/Concepts/About-ROS-2-Parameters.html>`_).
Most types are supported in SlicerROS2 and we provide methods to
determine the type at runtime.  The recommended steps are:

* check if the parameter exists using ``IsParameterSet(parameterName)``
* get the parameter type using ``GetParameterType(parameterName)``
* use the correct accessor based on the parameter's type, for example
  ``GetParameterAsString(parameterName)``

.. list-table:: Parameter types and accessors
   :widths: 30 70
   :header-rows: 1

   * - Type
     - Accessor
   * - "bool"
     - ``GetParameterAsBool``
   * - "integer"
     - ``GetParameterAsInteger``
   * - "double"
     - ``GetParameterAsDouble``
   * - "string"
     - ``GetParameterAsString``
   * - "bool_array"
     - ``GetParameterAsVectorOfBools``
   * - "integer_array"
     - ``GetParameterAsVectorOfIntegers``
   * - "double_array"
     - ``GetParameterAsVectorOfDoubles``
   * - "string_array"
     - ``GetParameterAsVectorOfStrings``

For convenience, we also provide the method
``vtkMRMLROS2ParameterNode::PrintParameter`` which will provide a
human readable description of the parameter whatever the type is
(using ``std::string``).

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         # setup to get parameter robot_description from node state_publisher
         parameterNode = rosNode.CreateAndAddParameterNode('state_publisher')
         parameterNode.AddParameter('robot_description')
         # check if parameter is set, C++ example is more detailed
         if parameterNode.IsParameterSet('robot_description'):
           # then check the type
           if parameterNode.GetParameterType('robot_description') == 'string':
             robotDescription = parameterNode.GetParameterAsString('robot_description')

   .. tab:: **C++**

      Setup:

      .. code-block:: C++

         // setup to get parameter robot_description from node state_publisher
         auto parameterNode = rosNode->CreateAndAddParameterNode("state_publisher");
         parameterNode->AddParameter("robot_description");
         // add a callback
         parameterNode->AddObserver(vtkMRMLROS2ParameterNode::ParameterModifiedEvent,
                                    this, &myClassType::Callback);

      Callback:

      .. code-block:: C++

         // ideally in callback but can be used in a busy loop
         if (!parameterNode->IsParameterSet("robot_description")) {
           vtkErrorMacro(<< "parameter \"robot_description\" is not set");
           return;
         }
         if (parameterNode->GetParameterType("robot_description") != "string") {
           std::string outType = parameterNode->GetParameterType("robot_description");
           vtkErrorMacro(<< "parameter \"robot_description\" is of type " << outType << " and not string.");
           return;
         }
         std::string robotDescription = parameterNode->GetParameterAsString("robot_description");

To remove the broadcaster node, use the method
``vtkMRMLROS2NodeNode::RemoveAndDeleteParameterNode``. This method
takes one parameter:

* the monitored node name (``std::string``) i.e. 'robot_state_publisher'


""""""""""""""
MRML ROS Nodes
""""""""""""""

The high level ROS functionalities (publisher, subscriber, Tf2
broadcast, Tf2 lookup or parameter client) are all encapsulated as
Slicer MRML nodes, i.e. derived from the class ``vtkMRMLNode`` and
added to the MRML Scene.  This allows to leverage some of the Slicer
features:

* Using Slicer node observer pattern to trigger user code when a new
  ROS message has been received (subscribers, parameters or Tf2
  lookups).

* Data visualization using the MRML scene.

* Retrieve node by ID, name or type.

* Python API automatically generated on top of the native C++ code.

* Ability to save and restore your ROS nodes along the MRML scene.

All the SlicerROS2 classes follow the Slicer naming convention,
i.e. ``vtkMRMLxxxxNode``.  We added the ``ROS2`` "prefix" for all the
class names so we're using ``vtkMRMLROS2xxxxNode`` where ``xxxx``
represents a ROS functionality.  This works fairly well but makes it a
bit hard to read for the MRML node than encapsulates a ROS node,
i.e. a ``vtkMRMLROS2NodeNode``.

======
Design
======

C++ vs Python
=============

Since both Slicer and ROS provide a Python interface we first tried to
use Python as the glue between Slicer and ROS.  The main issue on
Ubuntu is that ROS relies on the system python (version 3.8 on Ubuntu
20.04) while Slicer builds it's own Python interpreter (e.g. 3.9).
Since both libraries are not just native Python code, loading
(``import``) both Slicer and ROS in the same interpreter is not
possible as they rely on different versions of Python C++ libraries.
We also considered using the same Python version for both Slicer and
ROS, i.e. build everything using Python 3.8 or Python 3.9.
Unfortunately, building Slicer against the Ubuntu provided Python
interpreter (3.8) is no currently supported by the Slicer "super
build".  Alternatively, using the Slicer provided Python 3.9 to build
the ROS Python bindings (``rclpy``) proved challenging unless one
wanted to recompile ROS from scratch instead of using the existing
Ubuntu binary packages.

We ultimately decided to implement the SlicerROS2 module in C++ and
rely on the VTK/Slicer build to provide the Python bindings.  The
SlicerROS2 module is compiled against the Slicer and ROS2 libraries
which is a bit challenging since both packages have their own CMake
macros and have some Python version requirements.  The
``CMakeLists.txt`` provided along SlicerROS2 works but you'll have to
ignore some error and warning messages.

Execution Model
===============

One of the challenges of integrating ROS in Slicer is to figure out
the execution model.  ROS relies heavily on callbacks triggered by
external messages.  This requires to either use a separate thread to
"spin" the ROS event loop or periodically call the ROS "spin" method
from the application's main thread.  Since we are heavily relying on
the MRML scene, using a separate thread is not trivial.  Therefore the
SlicerROS2 module relies on the main Slicer thread to trigger a
periodic call to the ROS spin.  We currently use a Qt timer to trigger
this periodic call.

.. note::
   The default frequency for the SlicerROS2 module is 50Hz, i.e. 20ms

Templates vs Inheritance
========================

The two packages also differ in their design patterns.  Slicer (and
VTK) strongly relies on base classes and inheritance to allow runtime
decisions.  Meanwhile, ROS heavily relies on templates and type traits
which are handled at compilation time.  Most ROS communication
mechanisms only support a finite number of data types (e.g. parameters
are booleans, integers, floating points, strings or vector of
aforementioned types, tf2 uses transforms only...) so this is not a
major issue.

The main difficulty lies in supporting many ROS topics and services
For our code, we ended up using templates for our internal data
structures and add some macros to generate the vtk user classes.  These
macros use template specialization and add some methods to create a
C++ class that can be used within Slicer (including the Python
bindings generation).

To make life easier we also added a code generator that can create a
vtk object mimicking a ROS message.  The code generator will also
create the overloaded conversion methods (``vtkSlicerToROS2` and
`vtkROS2ToSlicer`) as well as code to call all the macros required to
create the publisher or subscriber node.  The macro
``generate_ros2_nodes`` is used in `MRML/CMakeLists.txt` and one can
add new ROS messages types.  New publisher and subscriber nodes will
be available after the SlicerROS2 module is recompiled.

Coordinate Systems and Units
============================

The SlicerROS2 module will automatically convert between the default
3D frames conventions in Slicer and ROS.  Slicer (and by extension all
VTK objects in Slicer) follow the `RAS convention
<https://www.slicer.org/wiki/Coordinate_systems>`_ and distances are
provided in millimeters.  Meanwhile uses the `RHS convention
<https://https://en.wikipedia.org/wiki/Right-hand_rule>`_ and SI units
(meters).

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

We don't recommend that you delete the default node. If you create another node and need to delete it, use the method ``vtkMRMLROS2NodeNode::Destroy``.

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
     - geometry_msgs::msg::PoseStamped
     - PoseStamped
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
         subString = rosNode.CreateAndAddSubscriberNode('String', '/my_string')
         # run `ros2 topic pub /my_string` to send a string
         m_string = subString.GetLastMessage()
         # alternate, get a string with the full message
         m_string_yaml = subString.GetLastMessageYAML()
         # since the subscriber is a MRML node, you can also create an observer (callback)
         # to trigger some code when a new message is received
         observerId = subString.AddObserver('ModifiedEvent', myCallback)

   .. tab:: **C++**

      .. code-block:: C++

         auto subString = rosNode->CreateAndAddSubscriberNode("String", "/my_string");
         // run ros2 topic echo /my_string in a terminal to see the output
         subString->Publish("my first string");

To remove the subscriber node, use the method ``vtkMRMLROS2NodeNode::RemoveAndDeleteSubscriberNode``. This method takes
one parameter:

* the topic name (``std::string``)

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

======
Robots
======

To create a new Robot node, one can either use the UI (instructions in
Section 3.3) or create the robot programmatically with the following
commands. The convenience function
``vtkMRMLROS2NodeNode::CreateAndAddRobotNode`` was added to the module
logic that accepts three arguments (``std::string robotName``,
``std::string parameterNodeName``, ``std::string parameterName``).

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         rosNode.CreateAndAddRobotNode('PSM','PSM1/robot_state_publisher','robot_description') # Using the PSM as an example

   .. tab:: **C++**

      .. code-block:: C++

         auto robot = rosNode->CreateAndAddRobotNode("PSM","PSM1/robot_state_publisher","robot_description");

The robot node creates an observer on the parameter node that contains
the robot description. If the parameter node is modified (indicating
that the robot description is available), it begins the process of
loading the visuals for the robot into the scene. This process
involves: parsing the URDF file, creating a list of Tf2 lookups in the
scene, creating the models for each link of the robot and applying the
correct color and offset position relative to the base of the
robot. Once the visuals have been created, the Tf2 lookups start to
check the Tf2 buffer and update the position of the model according to
the joint state publisher.

To remove the robot, use the "Remove robot" button on the UI or the
method ``vtkMRMLROS2NodeNode::RemoveAndDeleteRobotNode``. This method
takes one parameter:

* the robot name (``std::string``)

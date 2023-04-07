
""""""""""""""
MRML ROS Nodes
""""""""""""""

The high level ROS functionalities (publisher, subscriber, Tf2
broadcast, Tf2 lookup or parameter client) are all encapsulated as
Slicer MRML nodes, i.e. derived from the class ``vtkMRMLNode`` and
added to the MRML Scene.  This allows to leverage some of the Slicer
features:

* Data visualization using the MRML scene.

* Retrieve node by ID, name or type.

* Python API automatically generated on top of the native C++ code.

* Ability to save and restore your ROS 2 nodes along the MRML scene.

All the SlicerROS2 classes follow the Slicer naming convention,
i.e. ``vtkMRMLxxxxNode``.  We added the ``ROS2`` "prefix" for all the
class names so we're using ``vtkMRMLROS2xxxxNode`` where ``xxxx``
represents a ROS functionality.  This works fairly well but makes it a bit
hard to read for the MRML node than encapsulates a ROS 2 node, i.e. a
``vtkMRMLROS2NodeNode``.

======
Design
======

C++ vs Python
=============

Since both Slicer and ROS 2 provide a Python interface we first tried
to use Python as the glue between Slicer and ROS 2.  The main issue on
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
the ROS 2 Python bindings (``rclpy``) proved challenging unless one
wanted to recompile ROS 2 from scratch instead of using the existing
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

.. warning:: Since we rely on a Qt timer to trigger the ROS spin, the
   module will not receive any ROS messages until the GUI is created,
   i.e. until the ROS 2 module is manually loaded in Slicer.

Templates vs Inheritance
========================

The two packages also differ in their design patterns.  Slicer (and
VTK) strongly relies on base classes and inheritance to allow runtime
decisions.  Meanwhile, ROS 2 heavily relies on templates and type
traits which are handled at compilation time.  Most ROS 2
communication mechanisms only support a finite number of data types
(e.g. parameters are booleans, integers, floating points, strings or
vector of aforementioned types, tf2 uses transforms only...) so this
is not a major issue.

The main difficulty lies in supporting ROS topics and services For
your code, we ended up using templates for our internal data
structures and add some macros to generate the user classes.  These
macros use template specialization and add some methods to create a
C++ class that can be used within Slicer (including the Python
bindings generation).

========
ROS Node
========

The SlicerROS2 module always creates a default ROS node (internally a
``rclcpp::node``).  This node is both a ROS 2 node and a MRML node,
hence the unfortunate name ``vtkMRMLROS2NodeNode``.  This node is
added to the MRML scene and should be used to add your custom
``vtkMRMLROS2`` nodes (topics, parameter...).  It should be possible
to add an extra ROS 2 node in SlicerROS2 but this feature has not been
tested extensively for the first release.

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

      If you're working on a new module that relies on the slicerROS2
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

======
Topics
======

.. _publishers:

Publishers
==========

To create a new publisher, one should use the ROS2 Node method
``CreateAndAddPublisherNode``.  This method take two parameters:

* the class (type) of publisher to be used.  We provide some
  publishers for the most commonn data types (from Slicer to ROS
  messages).  The full list can be found in the Slicer ROS logic class
  (``Logic/vtkSlicerROS2Logic.cc``) in the method ``RegisterNodes``.
* the topic name.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         pubString = rosNode.CreateAndAddPublisherNode('vtkMRMLROS2PublisherStringNode', '/my_string')
         # run ros2 topic echo /my_string in a terminal to see the output
         pubString.Publish('my first string')

         pubMatrix = ros2.CreateAndAddPublisher('vtkMRMLROS2PublisherPoseStampedNode', '/my_matrix')
         # run ros2 topic echo /my_matrix in a terminal to see the output
         mat = vtk.vtkMatrix4x4()
         mat.SetElement(0, 3, 3.1415) # Modify the matrix so we can see something changing
         pubMatrix.Publish(mat)

   .. tab:: **C++**

      .. code-block:: C++

         auto pubString = rosNode->CreateAndAddPublisherNode("vtkMRMLROS2PublisherStringNode", "/my_string");
         // run ros2 topic echo /my_string in a terminal to see the output
         pubString->Publish("my first string");


Subscribers
===========

To create a new subscriber, one should use the ROS2 Node method
``CreateAndAddSubscriberNode``.  This method take two parameters:

* the class (type) of subscriber to be used.  See ::ref:`publishers`.
* the topic name.

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         subString = rosNode.CreateAndAddSubscriberNode('vtkMRMLROS2SubscriberStringNode', '/my_string')
         # run ros2 topic pub /my_string
         m_string = sub.GetLastMessage()
         # alternate, get a string with the full message
         m_string_yaml = sub.GetLastMessageYAML()

   .. tab:: **C++**

      .. code-block:: C++

         auto pubString = rosNode->CreateAndAddSubscriberNode("vtkMRMLROS2SubscriberStringNode", "/my_string");
         // run ros2 topic echo /my_string in a terminal to see the output
         pubString->Publish("my first string");




==========
Parameters
==========


===
Tf2
===

Broadcasts
==========

Lookups
=======

======
Robots
======

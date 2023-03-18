
""""""""""""""
MRML ROS Nodes
""""""""""""""

The high level ROS functionalites are all encapsulated as Slicer MRML
nodes, i.e. derived from the class ``vtkMRMLNode`` and added to the
MRML Scene.  This allows to leverage some of the Slicer features:

* Data visualization using the MRML scene.

* Retrieve node by ID, name or type.

* Python API automatically generated on top of the native C++ code.

* Ability to save and restore your ROS 2 nodes along the MRML scene.


All the SlicerROS2 classes follow the Slicer naming convention,
i.e. ``vtkMRMLxxxxNode``.  We added the ``ROS2`` "prefix" for all the
class names so we're using ``vtkMRMLROS2xxxxNode`` where ``xxxx``
represents a ROS2 object.  This works fairly well but makes it a bit
hard to read for the MRML node than encapsulates a ROS 2 node, i.e. a
``vtkMRMLROS2NodeNode``.

========
Overview
========

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
   i.e. until the ROS 2 module is loaded in Slicer.

======
Topics
======

Publishers
==========

Subscribers
===========

==========
Parameters
==========

===
Tf2
===

Broadcasts
==========

Buffer and lookups
==================

======
Robots
======

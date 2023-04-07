===========
Limitations
===========

* We only support STL and OBJ meshes for the ``visual`` defined in the
  URDF.  If any ``visual`` is defined using a ``geometry`` (sphere,
  box...) or another mesh format, it will not be displayed in Slicer

* ROS namespaces are not supported yet.  The current implementation
  allows multiple nodes but doesn't provide a parameter to set the ROS
  namespace.  Let us know if this is something you need.

* The following ROS functionalities are missing:

  + Services

  + Parameter server (``vtkMRMLROS2ParameterNode`` only works as a
    client)

* The current implementation assumes that all ROS2 nodes
  (``vtkMRMLROS2NodeNode``) added to the scene should be spun by the
  module's logic.  This doesn't provide an option for users to control
  how their ROS2 nodes spin (including rate).

* Saving and reloading the scene as a MRML scene has not been
  extensively tested and might not work.

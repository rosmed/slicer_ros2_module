=================
Known Limitations
=================

* We only support STL and OBJ meshes for the ``visual`` defined in the URDF.  
    + If any ``visual`` is defined using a ``geometry`` (sphere, box...), it will not be displayed in Slicer
* The current module only supports one robot at a time
* There is no mechanism to synchronize the MRML scene in Slicer with tf2 in ROS 2 
===============================
Features
===============================
The main feature currently supported is the ability to visualize a robot within Slicer.  The default approach is similar to RViz, i.e.:

* Load a robot description from ros parameter ``robot_description``, create a MRML Slicer node for each link and load meshes
* Update the links positions using tf2, assuming there is a ``robot_state_publisher`` running

Alternatively, the module can load an URDF file directly and use KDL for the kinematic chain.  This feature only works with serial robots (no parallel mechanisms).  This is not the recommended approach.

**This is an early prototype and we hope to add more features soon.**


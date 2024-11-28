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

===========
Compiling additional extensions
===========

The 3D Slicer extension wizard is unfortunately not available when you compile Slicer from source.
Therefore, if you want to use another extension, you will need to compile it on your machine.

Here is an example for SlicerIGT (a module that supports pivot calibration, point-to-point registration, etc.).
Note that SlicerIGT has a dependency of SlicerIGSIO so we will build this too.

.. code-block:: bash
  git clone https://github.com/SlicerIGT/SlicerIGT.git
  mkdir SlicerIGT-build
  git clone https://github.com/IGSIO/SlicerIGSIO.git
  mkdir SlicerIGSIO-build
  cd SlicerIGSIO-build
  cmake ../SlicerIGSIO -DSlicer_DIR:PATH=/home/your_user_name_here/something_something/Slicer-SuperBuild-Debug/Slicer-build/ 
  make 
  cd ../
  cd SlicerIGT-build
  cmake ../SlicerIGT -DSlicer_DIR:PATH=/home/your_user_name_here/something_something/Slicer-SuperBuild-Debug/Slicer-build/ \
   -DSlicerIGSIO_DIR:PATH=/home/your_user_name_here/something_something/SlicerIGSIO-build/inner-build/ 
  make

After the extensions are built, open 3D Slicer and press "Edit" in the top left of the window, then "Applicaton settings". 
A popup will come up and you need to click the "Modules" menu within the popup. In the space that says additional module paths
you can press "Add" and enter the path to the modules you just build (ie. /home/your_user_name_here/something_something/SlicerIGSIO-build/
inner-build/lib/Slicer5.x/qt-loadable-modules and /home/your_user_name_here/something_something/SlicerIGSIO-build/lib/Slicer5.x/qt-loadable-modules 
and /home/your_user_name_here/something_something/SlicerIGSIO-build/lib/Slicer5.x/qt-scripted-modules). Not every extension will have loadable and scripted 
modules like SlicerIGT does. After you have added these paths, restart Slicer and you should now have access to the modules. 
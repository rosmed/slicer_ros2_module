Caveats
=======

.. warning::

   The current code to load a robot description assumes that ROS uses
   SI units while Slicer uses millimeters so it will perform a
   conversion.  Unfortunately, the conversion methods in SlicerROS2
   are not very consistent.

Missing features
----------------

* The following ROS functionalities are missing:

  + Service server

  + Parameter server (``vtkMRMLROS2ParameterNode`` only works as a
    client)

  + Actions

* The current implementation assumes that all ROS 2 nodes
  (``vtkMRMLROS2NodeNode``) added to the scene should be spun by the
  module's logic.  This doesn't provide an option for users to control
  how their ROS 2 nodes spin (including rate).

* Saving and reloading the scene as a MRML scene has not been
  extensively tested and might not work.

3D Slicer extensions
--------------------

When using SlicerROS2, you may want to use other Slicer extensions
(e.g., SlicerIGT). Since you are running Slicer from a ROS 2
environment, extensions should be managed using the
``manage_extensions.py`` script provided with this module.

The ``manage_extensions.py`` script allows you to search for,
download, and build extensions from the Slicer Extensions Index. It
automatically handles dependencies and registers the built extensions
so they are loaded when you use ``ros2 run slicer_ros2_module slicer``.

To use the extension manager:

.. code-block:: bash

   ros2 run slicer_ros2_module manage_extensions.py

This will open a graphical interface where you can manage your extensions.

Loading the module
------------------

Always use the provided ROS 2 launcher to start Slicer. This ensures
all environment variables (like ``LD_LIBRARY_PATH``) and Slicer module
paths are correctly configured to include both ``slicer_ros2_module``
and any extensions managed by ``manage_extensions.py``.

.. code-block:: bash

   ros2 run slicer_ros2_module slicer

You can also pass additional arguments directly to Slicer:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer --no-splash --python-script my_script.py

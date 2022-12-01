..    include:: <isonum.txt>

=============================
Using the Slicer ROS 2 module
=============================

In a terminal navigate to your Slicer inner build directory (``cd``).  Then:

.. code-block:: bash

    source ~/ros2_ws/install/setup.bash # or whatever your ROS 2 workspace is
    # you can ignore the error message related to slicer_ros2_module/local_setup.bash
    ./Slicer

.. note::

    The first time you run Slicer, you need to add the module directory should be in the application settings so that it can be loaded. 

To do so, open Slicer and navigate through the menus: 
    `Edit` |rarr| `Application Settings` |rarr| `Modules` |rarr| `Additional module paths` |rarr|  `Add`:

.. code-block:: bash

    ~ros2_ws/build/slicer_ros2_module/lib/Slicer-4.13/qt-loadable-modules


At that point, Slicer will offer to restart.  Do so and then load the module using the button:
     `Modules` |rarr| `Examples` |rarr| *Ros2*

The module's interface will appear on the left side of Slicer.  
You should leave the first two drop-down menus as-is, i.e. 
``State`` should be ``tf2`` and ``Description`` should be ``parameter``.  

The defaults ``/robot_state_publisher`` and ``robot_description`` should work 
for most cases so leave these as-is too.  

To activate your selection, just hit "Enter" in the ``/robot_state_publisher`` box.  
Please note that we plan to improve this user interface.  

At that point, the robot's model should be loaded and displayed in Slicer.

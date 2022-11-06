""""""""""""""""""""""""""""""""""""""""
Starting a ROS ``robot_state_publisher``
""""""""""""""""""""""""""""""""""""""""


The following are examples of robots we've used to test the Slicer ROS2 module. 

1. Note that we tried to follow the standard ROS approch, i.e. make the URDF description available as a ROS parameter and then launch the usual ROS ``robot_state_publisher`` node.  
2. The ``robot_state_publisher`` will use the joint state to compute the forward kinematic and broadcast the 3D position of each link to tf2.  
3. The Slicer ROS 2 module will then query the 3D positions to display the robot's links in the correct position. 

============================================
Sensable Phantom Omni aka Geomagic/3DS Touch
============================================

We created and used the following package for the Omni: `Omni Github Link <https://github.com/jhu-saw/ros2_sensable_omni_model>`_ 

This package contains the URDF, STL meshes and a launch file for the ``robot_state_publisher``. 
**Make sure you start the Omni nodes (using a couple of terminals) before loading the Slicer ROS 2 module.**:

.. code-block:: bash

    ros2 launch sensable_omni_model omni.launch.py # first terminal
    ros2 run sensable_omni_model pretend_omni_joint_state_publisher # second 

=========
dVRK PSM
=========

todo

======
Cobot
======

We also tested SlicerROS2 on `myCobot by Elephant Robotics <https://www.elephantrobotics.com/en/mycobot-en/>`_, specifically the myCobot 280 M5 Stack. 
The ROS 2 interface for the device can be found `here <https://github.com/elephantrobotics/mycobot_ros2>`_ 
and drivers can be installed from the Elephant Robotics website. 

Assuming the interface (mycobot_ros2) is cloned under the same ros2_ws, the state publisher can be started using the following steps: 

.. code-block:: bash

    cd ~/ros2_ws/src/mycobot_ros2/src/mycobot_ros2/mycobot_280/mycobot_280/config
    python3 listen_real.py


It's possible that you will need to change the port specified on line 14 of ``listen_real.py`` depending on your device.
The ``.dae`` files in the robot description also need to be converted to STLs (an online converter will work) and 
the paths in the URDF file should be updated to reflect this change. 

Once running - make sure your robot is in *Transponder Mode*. More instructions for basic operation of the myCobot can be found in the 
`Gitbook <https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.1-280/2.1-280.html>`_

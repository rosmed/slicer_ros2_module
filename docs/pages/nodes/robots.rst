======
Robots
======

To create a new Robot node, one can either use the UI
(:ref:`instructions <load_robot>`) or create the robot
programmatically with the following commands. The convenience function
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
takes up to 5 parameters:

* the robot name (``std::string``)
* the parameter node name (``std::string``)
* parameter name (``std::string``, default is "robot_description")
* fixed frame, aka reference frame used in RViz (``std::string``, default is ""),
* tf prefix (``std::string``, default is "");

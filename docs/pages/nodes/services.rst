========
Services
========


To find the current list of supported services, one can do:

.. code-block:: python

   rosLogic = slicer.util.getModuleLogic('ROS2')
   rosNode = rosLogic.GetDefaultROS2Node()
   rosNode.RegisteredROS2ServiceClientNodes()

The method ``RegisteredROS2xxxxxNodes`` returns a long string with all
the services client classes available.

.. _service_client:

Clients
=======

To create a new service client, one should use the MRML ROS2 Node method
``vtkMRMLROS2NodeNode::CreateAndAddServiceCleintNode``.  This method takes
two parameters:

* the class (type) of service to be used (full or short name).  We
  provide clients for the most common service types (from Slicer
  to ROS messages).
* the service name (``std::string``).

---- Publishers are triggered by calling the ``Publish`` method.

.. code-block:: bash

   # make sure service server is started
   # for this example we use the turtle sim package
   ros2 launch turtlesim multisim.launch.py

.. tabs::

   .. tab:: **Python**

      .. code-block:: python

	 # create clients
         rosLogic = slicer.util.getModuleLogic('ROS2')
         rosNode = rosLogic.GetDefaultROS2Node()
         # optional, shows which services are available
         rosNode.RegisteredROS2ServiceClientNodes()
         # example with full class name
	 spawn1 = rosNode.CreateAndAddServiceClientNode('vtkMRMLROS2ServiceClientSpawnNode', '/turtlesim1/spawn')
	 # to check if the client is started, in a shell: ros2 service info /turtlesim1/spawn

	 # now create a blank request, a new turtle will be created
	 req = spawn1.CreateBlankRequest()
	 req.SetX(4.0)
	 req.SetY(4.0)
	 # send the request
	 spawn1.SendAsyncRequest(req)
	 # get the response
	 res = spawn1.GetLastResponse()
	 # Spawn request returns the name given to the new turtle
	 res.GetName()

         # example with short class name, Spawn will be expended to vtkMRMLROS2ServiceClientSpawnNode
         spawn2 = rosNode.CreateAndAddServiceClientNode('Spawn', '/turtlesim2/spawn')

   .. tab:: **C++**

      .. code-block:: C++

         // example with full class name
         auto spawn1 = rosNode->CreateAndAddPublisherNode("vtkMRMLROS2ServiceClientSpawnNode", "/turtlesim1/spawn");
         --- pubString->Publish("my first string");

         // example with short class name, Spawn will be expended to vtkMRMLROS2ServiceClientSpawnNode
         auto spawn2 = rosNode->CreateAndAddPublisherNo("Spawn", "/turtlesim2/spawn");


To remove the service client node, use the method ``vtkMRMLROS2NodeNode::RemoveAndDeleteServiceClientNode``. This method takes
one parameter:

* the service name (``std::string``)


Servers
=======

Not yet implemented.

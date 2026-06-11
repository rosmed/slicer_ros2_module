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
-------

To create a new service client, one should use the MRML ROS2 Node method
``vtkMRMLROS2NodeNode::CreateAndAddServiceClientNode``.  This method takes
two parameters:

* the class (type) of service to be used (full or short name).  We
  provide clients for the most common service types (from Slicer
  to ROS messages).
* the service name (``std::string``).

Service requests are sent asynchronously with ``SendAsyncRequest``.  After
sending a request, use ``ROS2`` logic's ``WaitForServiceResponse`` helper before
reading ``GetLastResponse``.  The helper keeps Slicer's Qt event loop active so
the module-level ROS 2 spin timer can process the response; user scripts do not
need to call ``Spin`` directly.

For non-blocking workflows, observe
``vtkMRMLROS2ServiceClientNode.ResponseReceivedEvent`` on the service client
node.  This event is emitted when a valid response has been received and
``GetLastResponse`` is ready.

The example below calls turtlesim's ``Spawn`` service.  The request creates a
new turtle, and the response contains the name assigned to that turtle.

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
         # wait for the asynchronous response
         if not rosLogic.WaitForServiceResponse(spawn1, 5.0):
             raise TimeoutError(f"Timed out waiting for response from {spawn1.GetService()}")
         res = spawn1.GetLastResponse()
         # Spawn request returns the name given to the new turtle
         print(res.GetName())

         # example with short class name, Spawn will be expanded to vtkMRMLROS2ServiceClientSpawnNode
         spawn2 = rosNode.CreateAndAddServiceClientNode('Spawn', '/turtlesim2/spawn')
         req = spawn2.CreateBlankRequest()
         req.SetX(6.0)
         req.SetY(4.0)
         spawn2.SendAsyncRequest(req)
         if not rosLogic.WaitForServiceResponse(spawn2, 5.0):
             raise TimeoutError(f"Timed out waiting for response from {spawn2.GetService()}")
         res = spawn2.GetLastResponse()
         print(res.GetName())

         # Optional non-blocking pattern: attach the observer before sending.
         req = spawn2.CreateBlankRequest()
         req.SetX(8.0)
         req.SetY(4.0)

         def onSpawnResponse(caller, event):
             print(caller.GetLastResponse().GetName())

         observerId = spawn2.AddObserver(
             slicer.vtkMRMLROS2ServiceClientNode.ResponseReceivedEvent,
             onSpawnResponse)
         spawn2.SendAsyncRequest(req)

   .. tab:: **C++**

      .. code-block:: C++

         // example with full class name
         auto spawn1 = rosNode->CreateAndAddServiceClientNode("vtkMRMLROS2ServiceClientSpawnNode", "/turtlesim1/spawn");

         // example with short class name, Spawn will be expanded to vtkMRMLROS2ServiceClientSpawnNode
         auto spawn2 = rosNode->CreateAndAddServiceClientNode("Spawn", "/turtlesim2/spawn");


To remove the service client node, use the method ``vtkMRMLROS2NodeNode::RemoveAndDeleteServiceClientNode``. This method takes
one parameter:

* the service name (``std::string``)


Servers
-------

Not yet implemented.

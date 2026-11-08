dVRK Patient Cart and Virtual Camera
====================================

This example loads a simulated Classic dVRK patient cart into Slicer and
renders a live image from the ECM optical frame. The reusable classes and
their options are documented in :doc:`/pages/utilities/dvrk`.

.. figure:: /images/dvrk-patient-cart-slicer.png
   :width: 100%
   :align: center
   :alt: Classic dVRK patient cart and virtual ECM camera image in Slicer, with RViz in the foreground.

   The Classic patient cart in Slicer, with the live ECM camera image in the
   red slice view and RViz showing the same cart.

Start dVRK
----------

In a terminal, source your built dVRK workspace and start the patient cart:

.. code-block:: bash

   ros2 launch dvrk_model patient_cart.launch.py generation:=Classic

This launch starts the simulated dVRK system, the arm and SUJ state
publishers, and RViz. Keep it running. The workspace must contain the dVRK
robot, configuration, and model packages used by this launch.

In a second terminal, source your dVRK and SlicerROS2 workspaces, then start
Slicer:

.. code-block:: bash

   ros2 run slicer_ros2_module slicer

Slicer needs the dVRK workspace environment to resolve the models' mesh
packages.

Load the patient cart
---------------------

Open Slicer's Python Interactor (Ctrl + 3) and run:

.. code-block:: python

   from slicerros2.dvrk import PatientCart

   cart = PatientCart(include_SUJ=True)
   robots = cart.load(wait=True, fit_view=True)

This loads PSM1, PSM2, PSM3, ECM, and the Classic cart's supporting structure.
The robots also appear in the ROS2 module GUI.

Start the virtual camera
------------------------

In the same Python Interactor:

.. code-block:: python

   camera = cart.create_camera(width=640, height=480, fps=20)

The camera follows ``ECM_optical`` and updates the ``ECM camera RGB`` volume.
To show this image beside the 3D view, run:

.. code-block:: python

   import slicer

   layout = slicer.app.layoutManager()
   layout.setLayout(slicer.vtkMRMLLayoutNode.SlicerLayoutFourUpView)
   red = layout.sliceWidget("Red")
   red.mrmlSliceCompositeNode().SetBackgroundVolumeID(camera.image_node.GetID())
   red.mrmlSliceNode().SetOrientationToAxial()
   red.sliceLogic().FitSliceToAll()

When finished, stop the camera before unloading its robot transforms:

.. code-block:: python

   camera.close(remove_image=True)
   cart.unload()

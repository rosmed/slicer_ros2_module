dVRK Python Helpers
===================

``slicerros2.dvrk`` provides patient-cart loading and ECM camera helpers for
scripts running inside Slicer. For ROS startup commands and a complete
walkthrough, see :doc:`/pages/examples/dvrk`.

PatientCart
-----------

``PatientCart(*, include_SUJ=False, include_ECM=True, fixed_frame="world", ros_node=None)``

Constructs a loader without loading any robot visuals. ``ros_node`` selects
an existing MRML ROS2 node; ``None`` uses the ROS2 module's default node.
The constructor stores the options as attributes and initializes an empty
``robots`` dictionary.

.. list-table:: Constructor options
   :header-rows: 1
   :widths: 25 15 60

   * - Option
     - Default
     - Meaning
   * - ``include_SUJ``
     - ``False``
     - Include the supporting structure for a Classic/Si cart.
   * - ``include_ECM``
     - ``True``
     - Include ECM robot visuals in addition to the three PSMs.
   * - ``fixed_frame``
     - ``"world"``
     - Reference frame for robot TF lookups and cameras created by this loader.
   * - ``ros_node``
     - ``None``
     - MRML ROS2 node that owns the robot connections.

.. py:method:: PatientCart.load(*, wait=False, timeout=30.0, fit_view=False)

   Creates or reuses the selected MRML robot nodes and returns a dictionary
   keyed by arm name. The same dictionary is available as ``cart.robots``.
   Loading is asynchronous by default. ``wait=True`` processes Slicer's Qt/ROS
   events until every selected robot has at least one model reference, or raises
   ``TimeoutError`` after ``timeout`` seconds. This checks geometry availability;
   it does not verify TF connectivity. Call it on Slicer's main thread.
   If waiting times out, the created nodes remain in ``cart.robots`` and can
   be removed with ``cart.unload()``.
   ``fit_view=True`` resets the 3D views after the load operation.

   The loader uses these robot-description parameter servers:

   .. list-table:: Robot sources
      :header-rows: 1
      :widths: 20 80

      * - Robot
        - Parameter server
      * - PSM1, PSM2, PSM3
        - ``/PSM1/robot_state_publisher``, ``/PSM2/robot_state_publisher``, ``/PSM3/robot_state_publisher``
      * - ECM
        - ``/ECM/robot_state_publisher``
      * - SUJ
        - ``/SUJ/SUJ_robot_state_publisher``

   Each server supplies ``robot_description``. dVRK URDFs already contain arm
   frame names, so the loader adds no TF prefix. Existing robots with the same
   name are reused with their original configuration. Changing constructor
   options does not reconfigure an existing robot; unload it before replacing it.

.. py:method:: PatientCart.create_camera(**options)

   Returns an ``ECMCamera`` using the loaded ECM node when available, together
   with the cart's ROS2 node and fixed frame. Pass camera options such as
   ``width``, ``height``, and ``fps`` here. If ECM visuals are omitted, the camera
   can follow TF directly. When an ECM robot is present, wait for its visuals
   before creating the camera so its optical-frame lookup is available.

.. py:method:: PatientCart.unload()

   Removes the robots recorded in this loader's ``robots`` dictionary, including
   their managed visuals, and clears the dictionary. Reused robots are included
   in this cleanup. Close cameras that use these robots before unloading.

ECMCamera
---------

``ECMCamera(ecm_robot=None, *, ros_node=None, tip_frame=None, fixed_frame="world", name="ECM camera", **options)``

Extends ``slicerros2.cameras.VirtualCamera`` with dVRK optical-frame selection.
``tip_frame`` defaults to ``ECM_optical``. ROS optical axes are X right,
Y down, Z forward.

Lookup selection prefers an exact frame-name match. An unqualified name such
as ``ECM_optical`` can also match the suffix of a namespaced frame. A qualified
name such as ``cart/ECM_optical`` must match the complete frame name.

If ``ecm_robot`` is supplied, the helper searches its ``lookup`` references
for the optical frame. Otherwise, it first looks for an ECM robot on the
selected ROS2 node. When a robot exists but the optical lookup is missing,
construction raises ``RuntimeError``; wait for the robot visuals to load.
When no ECM robot exists, the helper creates a TF lookup from ``fixed_frame``
to the optical frame. The TF must be published even when ECM visuals are absent.

The camera owns a lookup it creates itself and removes it on close. It leaves
a borrowed robot lookup intact. If renderer initialization fails, it cleans
up its newly created lookup before propagating the error.

Camera options
~~~~~~~~~~~~~~

These options are forwarded to ``VirtualCamera``:

.. list-table:: Rendering options
   :header-rows: 1
   :widths: 30 20 50

   * - Option
     - Default
     - Meaning
   * - ``width``, ``height``
     - ``640``, ``480``
     - Positive integer image dimensions in pixels.
   * - ``fps``
     - ``20.0``
     - Requested update rate, implemented by a Qt timer.
   * - ``horizontal_fov_degrees``
     - ``60.0``
     - Horizontal field of view, strictly between 0 and 180 degrees.
   * - ``near_mm``, ``far_mm``
     - ``5.0``, ``10000.0``
     - Clipping distances, satisfying ``0 < near_mm < far_mm``.
   * - ``start``
     - ``True``
     - Capture immediately and begin periodic updates on construction.

Methods and output
~~~~~~~~~~~~~~~~~~

``capture()`` updates the camera pose and renders one frame, returning
``image_node``. ``start()`` captures immediately, starts periodic updates,
and returns the camera. ``stop()`` pauses updates while retaining the last
image and rendering resources. ``running`` reports whether automatic capture
is active.

``image_node`` is a ``vtkMRMLVectorVolumeNode`` containing a
``(width, height, 1)`` image with three unsigned-byte RGB components. It is a
rendered image, not a spatially calibrated medical volume. Its I/J directions
match screen-right/up in Slicer's standard axial slice view. ``frame_count``
counts successful captures; ``last_error`` records the exception if a timer
update fails. A failed timer update logs the error and closes the camera.

``close(*, remove_image=False)`` stops updates and releases the dedicated view,
active camera node, timer, and any owned TF lookup. By default the RGB volume
and its last image remain available; pass ``remove_image=True`` to remove them
too. This also works after an earlier close or an automatic error shutdown.
Repeated close calls release rendering resources only once. Closing the MRML
scene also closes the camera. Stop or close the camera before removing its
input transform or image.

Rendering internals
~~~~~~~~~~~~~~~~~~~

``VirtualCamera`` creates a dedicated offscreen ``qMRMLThreeDWidget`` and a
view node whose unique layout name is set before insertion into the scene.
The main Slicer views retain their cameras. On capture, the renderer uses the
optical transform's full hierarchy in Slicer world coordinates (millimeters),
including roll: the translation sets camera position, +Z sets its forward
direction, and -Y sets view-up. ``vtkWindowToImageFilter`` copies the rendered
RGB pixels into the output volume. All camera operations run on the Qt thread.
Keep a Python reference to the camera while it is in use.

Convenience functions
---------------------

``load_patient_cart(**options)`` constructs a ``PatientCart`` and returns its
robot dictionary. It accepts the constructor options plus ``wait`` and
``timeout``. Use the class directly when you also need ``fit_view`` or a loader
object for later cleanup.

``unload_patient_cart(*, ros_node=None)`` removes any standard PSM1, PSM2, PSM3,
ECM, and SUJ robot nodes on the selected ROS2 node. Unlike ``PatientCart.unload()``,
it is not limited to the robots recorded by a particular loader.

``create_ecm_camera(ecm_robot=None, **options)`` is a convenience constructor
for ``ECMCamera`` and accepts the same camera options.

Startup wrappers
----------------

``slicerros2.examples.patient_cart.run()`` loads a cart, waits for geometry,
fits the view by default, and returns the robot dictionary. Its ``cleanup()``
unloads the cart.

``slicerros2.examples.ecm_camera.run()`` loads a cart, waits for geometry, and
returns an ECM camera. Its ``cleanup()`` removes the camera and image while
leaving the cart loaded. Use ``unload_patient_cart()`` to remove that cart.
Both wrappers accept ``include_SUJ``, ``include_ECM``, and ``fixed_frame``;
the camera wrapper also forwards rendering options.

The installed ``patient_cart.py`` and ``ecm_camera.py`` startup stubs use these
wrappers and store their return value in ``demo``. See
:doc:`/pages/examples` for the common startup command. Image display setup is
explicit, as shown in the :doc:`dVRK walkthrough </pages/examples/dvrk>`.

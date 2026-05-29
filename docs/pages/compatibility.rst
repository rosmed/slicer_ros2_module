Compatibility
=============

SlicerROS2 depends on both the 3D Slicer build environment and the ROS 2
distribution installed on the system. The currently supported release target is:

.. list-table::
   :header-rows: 1

   * - SlicerROS2
     - Ubuntu
     - ROS 2
     - 3D Slicer
     - Status
   * - 1.2
     - 24.04
     - Jazzy
     - 5.10.0
     - Current release target
   * - 1.1
     - 24.04
     - Jazzy
     - 5.10.0
     - Tested
   * - 1.0
     - 20.04, 22.04, 24.04
     - Galactic, Humble, Jazzy
     - Earlier Slicer versions
     - Historical release

For SlicerROS2 1.2, use the Slicer ``v5.10.0`` tag:

.. code-block:: bash

   git clone https://github.com/slicer/slicer
   cd Slicer
   git checkout v5.10.0

Slicer should be built from source when building SlicerROS2, because SlicerROS2
contains C++ loadable modules and MRML logic.

Required ROS packages are declared in ``package.xml``. System packages that are
not represented as ROS package dependencies include:

.. code-block:: bash

   sudo apt install python3-colcon-common-extensions libassimp-dev mold

The CI Docker image described in :doc:`ci-docker-image` provides the tested
Ubuntu, ROS 2, and Slicer combination.

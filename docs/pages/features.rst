========
Features
========

Overview
========

This module is designed to enable direct communication between ROS 2
and 3D Slicer.

This module currently supports ROS 2 topics (subscribers and
publishers), tf2 (broadcasters and lookups) as well as a parameter
client (no server yet).

This module can also be used to visualize a ROS robot in action using a
parameter client and tf2 lookups.  The robot visualization
implementation is following the ROS logic, i.e. the robot description
(URDF) is retrieved as a parameter and the real-time link positions
are from Tf2.  As for RViz, for each robot, you will need a
``robot_state_publisher`` node that will:

* provide an URDF robot description using a ROS parameter ``robot_description``
* update the links positions and broadcast them to tf2


All the functionalities in this module are encapsulated in
``vtkMRMLNode`` so you use them in your own applications.  Since
Slicer automatically provides a Python interface for all classes
derived from ``vtkMRMLNode``, you can develop your application using
either C++ or Python.

Acknowledgement
===============

This project is supported by:

* The National Institute of Biomedical Imaging and Bioengineering of the U.S. National Institutes of Health (NIH) under award number R01EB020667, and 3R01EB020667-05S1 (MPI: Tokuda, Krieger, Leonard, and Fuge). The content is solely the responsibility of the authors and does not necessarily represent the official views of the NIH.
* The National Sciences and Engineering Research Council of Canada and the Canadian Institutes of Health Research.

Publications
============

You can find some details regarding an early version of this module in
`Bridging 3D Slicer and ROS2 for Image-Guided Robotic Interventions
<https://pubmed.ncbi.nlm.nih.gov/35891016/>`_.

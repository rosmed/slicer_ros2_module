
""""""""""""
Introduction
""""""""""""

========
Overview
========

This module is designed to enable direct communication between ROS 2
and 3D Slicer.

This module currently supports ROS 2 topics (subscribers and
publishers), tf2 (broadcasters and lookups) as well as a parameter
client (no server yet).  Users can add new publishers and subscribers
with the CMake macro ``generate_ros2_nodes``.

This module can also be used to visualize a ROS robot in action using a
parameter client and tf2 lookups.  The robot visualization
implementation follows ROS logic, i.e. the robot description
(URDF) is retrieved as a parameter and the real-time link positions
are from Tf2.  For each robot, you will need a
``robot_state_publisher`` node that will:

* provide an URDF robot description using a ROS parameter ``robot_description``
* update the links positions and broadcast them to tf2


All the functionalities in this module are encapsulated in
``vtkMRMLNode`` so you use them in your own applications.  Since
Slicer automatically provides a Python interface for all classes
derived from ``vtkMRMLNode``, you can develop your application using
either C++ or Python. For more information on each of these nodes,
please see the 'Methodology' section of the `most recent paper
<https://ieeexplore.ieee.org/abstract/document/10684721>`_.

===============
Acknowledgment
===============

The initial core developers are:

* Laura Connolly, EE PhD student at Queens University, Kingston, Ontario, Canada
* Aravind S. Kumar, CS Masters student at Johns Hopkins University, Baltimore, Maryland, USA
* Anton Deguet, Associate Research Engineer at Johns Hopkins University, Baltimore, Maryland, USA

This project is supported by:

* The National Institute of Biomedical Imaging and Bio-engineering of the U.S. National Institutes of Health (NIH) under award number R01EB020667, and 3R01EB020667-05S1 (MPI: Tokuda, Krieger, Leonard, and Fuge). The content is solely the responsibility of the authors and does not necessarily represent the official views of the NIH.
* The National Sciences and Engineering Research Council of Canada and the Canadian Institutes of Health Research, the Walter C. Sumner Memorial Award, the Mitacs Globalink Award and the Michael Smith Foreign Study Supplement.

============
Publications
============

.. note::

   For citations, please use the 2024 publication, :download:`BibTeX
   </images/connolly2024.bib>`

- Connolly L, Kumar AS, Mehta KK, Al-Zogbi L, Kazanzides P, Mousavi P,
  Fichtinger G, Krieger A, Tokuda J, Taylor RH, Leonard S,
  Deguet A. SlicerROS2: A Research and Development Module for
  Image-Guided Robotic Interventions. IEEE Trans Med Robot
  Bionics. 2024 Nov;6(4):1334-1344. doi:
  10.1109/TMRB.2024.3464683. [PMID: Not available]; [PMCID: Not
  available].  `IEEE online abstract
  <https://ieeexplore.ieee.org/abstract/document/10684721>`_.
  :download:`BibTeX </images/connolly2024.bib>`

- Connolly L, Deguet A, Leonard S, Tokuda J, Ungi T, Krieger A,
  Kazanzides P, Mousavi P, Fichtinger G, Taylor RH. Bridging 3D Slicer
  and ROS2 for Image-Guided Robotic Interventions. Sensors
  (Basel). 2022 Jul 17;22(14):5336. doi: 10.3390/s22145336. PMID:
  `[35891016] <https://pubmed.ncbi.nlm.nih.gov/35891016/>`_; PMCID:
  `[PMC9324680]
  <https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9324680/>`_.
  :download:`BibTeX </images/connolly2022.bib>`


=============
Documentation
=============

Besides this document, you can also check DeepWiki:
https://deepwiki.com/rosmed/slicer_ros2_module. DeepWiki might be able
to answer your questions.

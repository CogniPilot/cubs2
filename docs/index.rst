Cubs2 Documentation
===================

**Cubs2** is a comprehensive ROS 2 framework for fixed-wing aircraft simulation, control, and autonomous flight.

.. image:: https://img.shields.io/badge/ROS%202-Jazzy-blue
   :target: https://docs.ros.org/en/jazzy/index.html
   :alt: ROS 2 Jazzy

.. image:: https://github.com/jgoppert/cubs2/actions/workflows/ci.yml/badge.svg
   :target: https://github.com/jgoppert/cubs2/actions/workflows/ci.yml
   :alt: CI Status

.. image:: https://img.shields.io/badge/coverage-62%25-orange
   :alt: Coverage

Key Features
------------

* **Physics-Based Simulation**: CasADi-powered differentiable dynamics
* **Hierarchical Composition**: Type-safe model composition with single integration loop
* **Path Planning**: Dubins path generation for racecourse navigation
* **Custom RViz Panels**: Interactive joystick, HUD, and simulation controls
* **Real-Time Control**: Production-ready controllers for simulation and hardware
* **Replay Support**: Compare simulation against recorded flight data

Quick Start
-----------

.. code-block:: bash

   # Build workspace
   colcon build --symlink-install
   source install/setup.bash

   # Launch simulation with visualization
   ros2 launch cubs2_bringup sim.xml

   # Or just visualization (in separate terminal)
   ros2 launch cubs2_bringup viz.xml

.. toctree::
   :maxdepth: 2
   :caption: User Guide

   user_guide/installation
   user_guide/quickstart
   user_guide/simulation
   user_guide/control
   user_guide/planning
   user_guide/visualization

.. toctree::
   :maxdepth: 2
   :caption: Package Documentation

   packages/cubs2_msgs
   packages/cubs2_description
   packages/cubs2_dynamics
   packages/cubs2_control
   packages/cubs2_simulation
   packages/cubs2_planning
   packages/cubs2_rviz
   packages/cubs2_bringup
   packages/cubs2_data
   packages/racecourse_description

.. toctree::
   :maxdepth: 2
   :caption: API Reference

   api/dynamics
   api/control
   api/planning
   api/simulation

.. toctree::
   :maxdepth: 1
   :caption: Development

   development/contributing
   development/architecture
   development/testing

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

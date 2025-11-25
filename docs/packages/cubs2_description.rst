cubs2_description
=================

URDF robot description and 3D meshes for the SportCub aircraft.

Files
-----

URDF Model
^^^^^^^^^^

``urdf/sportcub.urdf`` - Robot description file defining:

* Links (fuselage, wings, control surfaces)
* Joints (control surface rotations)
* Visual meshes
* Collision geometry
* Inertial properties

3D Meshes
^^^^^^^^^

``meshes/`` directory contains:

* ``sportcub_body.dae`` - Fuselage and wings
* ``sportcub_propeller.dae`` - Propeller (optional)
* Texture files

Launch Files
------------

robot_state_publisher.launch.py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Publishes TF transforms from URDF:

.. code-block:: bash

   ros2 launch cubs2_description robot_state_publisher.launch.py

Usage
-----

In RViz
^^^^^^^

1. Add RobotModel display
2. Set Robot Description topic: ``/robot_description``
3. Visual representation will appear

In Launch Files
^^^^^^^^^^^^^^^

.. code-block:: xml

   <node pkg="robot_state_publisher" exec="robot_state_publisher">
     <param name="robot_description" 
            textfile="$(find-pkg-share cubs2_description)/urdf/sportcub.urdf"/>
   </node>

Customization
-------------

Modify Visual Appearance
^^^^^^^^^^^^^^^^^^^^^^^^^

Edit ``urdf/sportcub.urdf``:

.. code-block:: xml

   <visual>
     <geometry>
       <mesh filename="package://cubs2_description/meshes/sportcub_body.dae"/>
     </geometry>
     <material name="blue">
       <color rgba="0 0 1 1"/>
     </material>
   </visual>

Adjust Inertial Properties
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Update mass and inertia tensor:

.. code-block:: xml

   <inertial>
     <mass value="11.0"/>
     <inertia ixx="0.5" iyy="0.8" izz="1.2" 
              ixy="0" ixz="0" iyz="0"/>
   </inertial>

Dependencies
------------

* robot_state_publisher
* xacro (if using xacro format)

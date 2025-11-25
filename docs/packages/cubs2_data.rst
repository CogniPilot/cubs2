cubs2_data
==========

Configuration files and datasets for Cubs2.

Configuration Files
-------------------

sportcub.yaml
^^^^^^^^^^^^^

Aircraft physical parameters:

.. code-block:: yaml

   mass: 11.0  # kg
   inertia:
     Ixx: 0.5    # kg⋅m²
     Iyy: 0.8
     Izz: 1.2
   
   aerodynamics:
     S_wing: 0.5    # m²
     c_bar: 0.2     # m (mean chord)
     b: 2.5         # m (wingspan)
   
   coefficients:
     CL0: 0.3       # Lift coefficient at zero AoA
     CD0: 0.03      # Parasitic drag
     # ... additional coefficients

sim.yaml
^^^^^^^^

Simulation settings:

.. code-block:: yaml

   initial_state:
     position: [0, 0, -100]  # NED (m)
     velocity: [20, 0, 0]    # Body frame (m/s)
     orientation: [1, 0, 0, 0]  # Quaternion
   
   integration:
     dt: 0.01        # Time step (s)
     method: rk4     # Integration method

control.yaml
^^^^^^^^^^^^

Controller gains:

.. code-block:: yaml

   autolevel:
     roll:
       kp: 1.0
       ki: 0.1
       kd: 0.05
     pitch:
       kp: 0.8
       ki: 0.1
       kd: 0.04
   
   limits:
     max_bank_angle: 45.0  # degrees
     max_pitch_angle: 20.0

Datasets
--------

Flight Test Data
^^^^^^^^^^^^^^^^

Rosbag files are hosted on Zenodo and can be downloaded selectively using the provided Python script.

**Zenodo Setup (for maintainers):**

1. Create account at https://zenodo.org
2. Click "New upload" and upload .mcap files
3. Publish the dataset (you'll get a DOI)
4. Update the URLs in ``download_rosbags.py``

**List Available Files:**

.. code-block:: bash

   cd $(ros2 pkg prefix cubs2_data)/share/cubs2_data/data
   ./download_rosbags.py --list

**Download Specific Files:**

.. code-block:: bash

   # Download a specific file
   ./download_rosbags.py cub_stabilize_2025-10-21.mcap

   # Download multiple files
   ./download_rosbags.py cub_stabilize_2025-10-21.mcap cub_stabilize_2025_11_13.mcap

   # Download files matching pattern
   ./download_rosbags.py cub_stabilize_*.mcap

   # Download all available files
   ./download_rosbags.py --all

**Available Options:**

* ``--list, -l`` - List all available rosbag files
* ``--all, -a`` - Download all available files
* ``--output-dir, -o DIR`` - Specify output directory
* ``--force, -f`` - Force re-download even if file exists

Replay
^^^^^^

.. code-block:: bash

   ros2 bag play cub_stabilize_2025-10-21.mcap

Usage
-----

Loading Configuration
^^^^^^^^^^^^^^^^^^^^^

Python:

.. code-block:: python

   import yaml
   from ament_index_python.packages import get_package_share_directory
   
   config_path = get_package_share_directory('cubs2_data')
   with open(f'{config_path}/config/sportcub.yaml') as f:
       params = yaml.safe_load(f)

Launch files automatically load from ``config/`` directory.

Customization
-------------

Adding Parameters
^^^^^^^^^^^^^^^^^

1. Edit YAML files
2. Rebuild (if using symlink install, no rebuild needed)
3. Parameters take effect on next launch

.. code-block:: bash

   colcon build --symlink-install --packages-select cubs2_data

Creating Datasets
^^^^^^^^^^^^^^^^^

Record your own flight data:

.. code-block:: bash

   ros2 bag record -o my_flight /sportcub/pose /sportcub/velocity /control

Installation
============

Dependencies
------------

Cubs2 requires the following dependencies:

* **ROS 2**: Jazzy or later
* **Python**: 3.12 or later
* **Python Packages**:
  
  * NumPy
  * CasADi
  * Cyecca (dynamics library)
  * SciPy
  * Matplotlib (for visualization)

* **System Libraries**:
  
  * Qt5
  * GStreamer 1.0 (for video streaming)

ROS 2 Installation
------------------

Follow the official ROS 2 Jazzy installation guide:

https://docs.ros.org/en/jazzy/Installation.html

For Ubuntu:

.. code-block:: bash

   # Set up sources
   sudo apt update && sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo apt update && sudo apt install curl -y
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

   # Install ROS 2 Jazzy
   sudo apt update
   sudo apt install ros-jazzy-desktop

Python Dependencies
-------------------

Install Python dependencies using pip:

.. code-block:: bash

   pip install numpy casadi scipy matplotlib

Install Cyecca (CognPilot dynamics library):

.. code-block:: bash

   pip install git+https://github.com/cognipilot/cyecca.git

System Libraries
----------------

Install Qt5 and GStreamer:

.. code-block:: bash

   sudo apt install qtbase5-dev libgstreamer1.0-dev

Building Cubs2
--------------

Clone the repository and build with colcon:

.. code-block:: bash

   # Create workspace
   mkdir -p ~/cubs2_ws/src
   cd ~/cubs2_ws/src

   # Clone repository
   git clone https://github.com/cognipilot/cubs2.git

   # Build workspace
   cd ~/cubs2_ws
   colcon build --symlink-install

   # Source workspace
   source install/setup.bash

Verifying Installation
----------------------

Run the test suite to verify installation:

.. code-block:: bash

   colcon test --packages-select cubs2_dynamics cubs2_control cubs2_planning
   colcon test-result --verbose

You should see all tests passing.

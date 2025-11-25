Contributing
============

We welcome contributions to Cubs2! This guide will help you get started.

Development Setup
-----------------

Clone and Build
^^^^^^^^^^^^^^^

.. code-block:: bash

   # Clone repository
   git clone https://github.com/cognipilot/cubs2.git
   cd cubs2

   # Install dependencies
   sudo apt install ros-jazzy-desktop qtbase5-dev libgstreamer1.0-dev
   pip install numpy scipy casadi cyecca

   # Build
   colcon build --symlink-install

   # Source workspace
   source install/setup.bash

Code Style
----------

Python
^^^^^^

We follow PEP 8 with some modifications:

* Line length: 100 characters
* Use Black for formatting
* Use isort for import sorting

.. code-block:: bash

   # Format code
   black cubs2_dynamics/ cubs2_control/ cubs2_planning/

   # Sort imports
   isort cubs2_dynamics/ cubs2_control/ cubs2_planning/

C++
^^^

We follow Google C++ Style Guide:

* Line length: 100 characters
* Use clang-format

.. code-block:: bash

   # Format C++ code
   find cubs2_rviz/src -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i

Linting
^^^^^^^

.. code-block:: bash

   # Python
   flake8 cubs2_dynamics/ cubs2_control/ cubs2_planning/

   # C++
   ament_cpplint cubs2_rviz/

Testing
-------

Unit Tests
^^^^^^^^^^

.. code-block:: bash

   # Run all tests
   colcon test

   # Run specific package
   colcon test --packages-select cubs2_dynamics

   # View results
   colcon test-result --verbose

Coverage
^^^^^^^^

.. code-block:: bash

   # Run with coverage
   colcon test --pytest-args --cov=cubs2_dynamics --cov-report=html

   # View report
   firefox build/cubs2_dynamics/coverage_html/index.html

Integration Tests
^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Launch full system
   ros2 launch cubs2_bringup sim.xml

   # In another terminal, run integration tests
   pytest tests/integration/

Pull Request Process
--------------------

1. Fork the Repository
^^^^^^^^^^^^^^^^^^^^^^^

Click "Fork" on GitHub and clone your fork:

.. code-block:: bash

   git clone https://github.com/YOUR_USERNAME/cubs2.git
   cd cubs2
   git remote add upstream https://github.com/cognipilot/cubs2.git

2. Create a Branch
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git checkout -b feature/my-new-feature

Use branch naming:

* ``feature/`` - New features
* ``fix/`` - Bug fixes
* ``docs/`` - Documentation
* ``refactor/`` - Code refactoring

3. Make Changes
^^^^^^^^^^^^^^^

* Write code following style guidelines
* Add tests for new functionality
* Update documentation
* Keep commits atomic and well-described

4. Run Tests
^^^^^^^^^^^^

.. code-block:: bash

   # Format code
   black .
   isort .

   # Lint
   flake8 .

   # Test
   colcon test
   colcon test-result

5. Submit Pull Request
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   git push origin feature/my-new-feature

Then open a pull request on GitHub with:

* Clear description of changes
* Reference to related issues
* Screenshots/videos if applicable

Code Review
-----------

Checklist
^^^^^^^^^

Reviewers will check:

* ✓ Code follows style guide
* ✓ Tests pass
* ✓ Documentation updated
* ✓ No breaking changes (or properly documented)
* ✓ Commit messages are clear

Addressing Feedback
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   # Make requested changes
   git add .
   git commit -m "Address review comments"
   git push origin feature/my-new-feature

Documentation
-------------

Building Docs
^^^^^^^^^^^^^

.. code-block:: bash

   cd docs
   make html
   firefox _build/html/index.html

Writing Docstrings
^^^^^^^^^^^^^^^^^^

Use NumPy style:

.. code-block:: python

   def my_function(param1, param2):
       """
       Brief description.

       Longer description if needed.

       Parameters
       ----------
       param1 : float
           Description of param1
       param2 : str
           Description of param2

       Returns
       -------
       bool
           Description of return value

       Examples
       --------
       >>> my_function(1.0, "test")
       True
       """
       return True

License
-------

By contributing, you agree that your contributions will be licensed under the Apache-2.0 License.

Questions?
----------

* Open an issue on GitHub
* Join our Discord: [link]
* Email: [email]

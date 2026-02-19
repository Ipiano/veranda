Setting up Veranda
==================

This guide covers installing ROS 2 and building Veranda from source.

Installing ROS 2
---------------

Veranda targets ROS 2 Jazzy Jalisco. Follow the instructions in the `ROS 2 Installation Guide`_ to get a working ROS 2 install in your environment.

.. _ROS 2 Installation Guide: https://docs.ros.org/en/jazzy/Installation.html

**Platform Notes:**

* **Ubuntu 24.04** - Primary development platform with native ROS 2 Jazzy packages
* **Ubuntu 22.04** - Supported, but requires building ROS 2 from source
* **Windows** - Supported via ROS 2 Jazzy for Windows with MSVC compiler

Installing Veranda Dependencies
--------------------------------

On Ubuntu 24.04, install the Qt development libraries:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install -y \
      qtbase5-dev \
      qtdeclarative5-dev \
      libqt5svg5-dev

The project requires Qt 5.12 or later. Qt 5.15.2 is recommended and is available in Ubuntu 24.04's repositories.

**Note:** Tools like ``colcon`` and ``rosdep`` should already be installed as part of your ROS 2 setup.

Building Veranda
----------------

Building the Veranda project is straightforward once ROS 2 is installed. First, you need to create a workspace, which is a directory with a specific structure for ROS 2 packages.

NOTE: The steps below reference 'sourcing the ROS environment' and 'sourcing the workspace environment.' See the :ref:`bottom of the page <sec-sourcing>` for detailed examples.

Creating and Building the Workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    # Create workspace structure
    mkdir -p ~/veranda_ws/src
    cd ~/veranda_ws/src

    # Clone the Veranda repository
    git clone https://github.com/roboscienceorg/veranda.git

    # Return to workspace root
    cd ~/veranda_ws

    # Install ROS dependencies
    rosdep install --from-paths src/veranda/veranda/Packages --ignore-src -r -y

    # Source ROS 2 Jazzy environment
    source /opt/ros/jazzy/setup.bash

    # Build the workspace
    colcon build --symlink-install

Now the project is built! You can verify the build succeeded by running tests:

.. code-block:: bash

    source install/setup.bash
    colcon test
    colcon test-result --verbose

Running Veranda
^^^^^^^^^^^^^^^

To run the Veranda Qt frontend:

.. code-block:: bash

    # From the workspace root
    source install/setup.bash
    ros2 run veranda_qt_frontend veranda

At this point, the application should open, and you can start simulating robots. To verify your setup is correct, try the :ref:`demos <sec-demos>`.

**Windows Note:** If you encounter Python compatibility issues on Windows, you may need to specify the correct Python interpreter when building:

.. code-block:: bash

    colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/path/to/correct/python.exe

.. _sec-sourcing:

What is 'Sourcing your environment'?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Sourcing the ROS 2 environment or your workspace environment sets up system variables specific to your terminal instance. In ROS 2, these variables contain paths to header files, binaries, and packages. When you source your workspace, it makes your built packages available for ROS 2 to find and run.

Sourcing differs by operating system:

* **Linux**: Use ``source`` command
* **macOS**: Use ``source`` command (or ``.`` shorthand)
* **Windows**: Use ``call`` command

Examples for Ubuntu 24.04 with ROS 2 Jazzy installed from binaries:

.. code-block:: bash

    # Source the ROS 2 Jazzy environment
    source /opt/ros/jazzy/setup.bash

    # Source a ROS 2 workspace
    source ~/veranda_ws/install/setup.bash

Each directory has two setup files: ``local_setup.bash`` and ``setup.bash``. The ``local_setup.bash`` file only sets up that specific environment, while ``setup.bash`` also sources any environments that were active when it was created.

**Best Practice**: After building your workspace, you can source just ``setup.bash`` in the workspace to automatically source both the ROS 2 environment and your workspace:

.. code-block:: bash

    # This sources both ROS 2 Jazzy and your workspace
    source ~/veranda_ws/install/setup.bash

If you built ROS 2 from source instead of using pre-built binaries, source your ROS 2 build workspace instead of ``/opt/ros/jazzy``.

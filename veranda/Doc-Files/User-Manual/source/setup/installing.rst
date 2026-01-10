Setting up Veranda
==================

Setting up Veranda requires installing ROS 2 Jazzy Jalisco and its dependencies, then downloading and building the Veranda source code using Colcon.

Installing ROS 2
---------------

Veranda targets ROS 2 Jazzy Jalisco, which is the LTS release for Ubuntu 24.04. Instructions for downloading and installing ROS 2 Jazzy can be found at `this address`_.

.. _this address: https://docs.ros.org/en/jazzy/Installation.html

Installing Dependencies
-----------------------

On Ubuntu 24.04, install the required dependencies:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get install -y \
      python3-colcon-common-extensions \
      python3-rosdep \
      qtbase5-dev \
      qtdeclarative5-dev \
      libqt5svg5-dev

The project requires Qt 5.12 or later. Qt 5.15.2 is recommended and is available in Ubuntu 24.04's repositories.

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

    # Initialize rosdep (if not already done)
    sudo rosdep init
    rosdep update

    # Install ROS dependencies
    rosdep install --from-paths src/veranda/veranda/Packages --ignore-src -r -y

    # Source ROS 2 Jazzy environment
    source /opt/ros/jazzy/setup.bash

    # Build the workspace using Colcon
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
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    ros2 run veranda_qt_frontend veranda_qt_frontend

At this point, the application should open, and you can start simulating robots. To verify your setup is correct, try the :ref:`demos <sec-demos>`.

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

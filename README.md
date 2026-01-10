# Veranda: A 2-Dimensional Mobile Robotics Simulation Environment

This repository contains the code for a 2D Robotics simulation tool. The tool is written to interface with other applications through the ROS 2 communication layer, and is usable on all operating systems that can have Qt 5 and ROS 2 installed.

## Requirements

### Core Dependencies
* **ROS 2** - The project targets ROS 2 Jazzy Jalisco
* **Qt 5 LTS** - Qt 5.12.12 or Qt 5.15.2 (other Qt 5 versions may work)
* **CMake 3.28+** - Modern CMake with target-based patterns
* **C++17** - Standard required for compilation
* **Python 3.9+** - Required for build system and scripts

### Build System
* **Colcon** - ROS 2 build tool (`python3-colcon-common-extensions`)
* **ament_cmake** - CMake macros for ROS 2 packages
* **rosdep** - Dependency management tool

### Platform Support
* **Ubuntu**
  * 24.04 is the primary development platform with ROS 2 Jazzy
  * 22.04 is also supported, but requires building ROS 2 from source
* **Windows** - Supported via ROS 2 Jazzy for Windows with MSVC compiler

## Building the Project

This project is built as a set of packages using `colcon` and `ament_cmake`, just
like ROS 2.

### Setting up ROS

Follow the instructions in the [ROS 2 Setup Guide](https://docs.ros.org/en/jazzy/Installation.html)
to get a working ROS 2 install in your environment

### Building Veranda on Ubuntu 24.04

```bash
# Install system dependencies
sudo apt-get update
sudo apt-get install -y \
  qtbase5-dev \
  qtdeclarative5-dev \
  libqt5svg5-dev

# Install ROS dependencies
cd veranda
rosdep install --from-paths Packages --ignore-src -r -y

# Build workspace
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Run tests
source install/setup.bash
colcon test
colcon test-result --verbose
```

### Windows

Follow the ROS 2 Jazzy installation instructions for Windows; then build veranda using Colcon, as
shown above for the Ubuntu build.

Depending on the version of Python you have installed, you may run into compatibility issues - it
may be necessary to pass `--cmake-args -DPython3_EXECUTABLE=/path/to/correct/python.exe` to colcon
to force usage of the correct python version (the same one ROS 2 uses).

## Plugin Architecture

Veranda uses a plugin-based architecture to extend functionality with custom sensors, shapes, wheels, and UI components. Plugins are built as separate packages and installed into
the ROS workspace in a location the main veranda executable can find them.

### Plugin Types
* **Logic Plugins**
  * Core simulation components (sensors, shapes, wheels)
  * Define the physics rules, messaging, and properties of components
* **Qt Plugins**
  * Provide UI integration for individual logic plugins
  * Example - how to draw a lidar
* **Frontend Plugins**
  * Plugins for the Veranda UI itself
  * Things like file handlers for different formats

### Plugin Development
All plugins use:
* **CMake 3.28+** with target-based dependency management
* **ament_cmake** build macros for ROS 2 integration
* **C++17**
* **Qt 5 Plugin System** for dynamic loading

Plugins depend on one or more of the plugin API packages in the veranda project
* `veranda_core_api` - Core simulation plugins
* `veranda_qt_plugin_api` - UI component plugins
* `veranda_qt_frontend_plugin_api` - UI framework plugins

Plugins are linked using `find_package` and `ament_target_dependencies`.

Plugins must be installed into common location for the UI to discover them. This
path is exposed as an ament index resource; plugins should use a snippet like the
following to install themselves into the plugin path.

All `.so` files in this directory will be checked for plugin compatibility against
all plugins, using a `QPluginLoader`.

```
    ament_index_get_resource(PLUGIN_PATH "veranda_plugin_path" "veranda_qt_frontend")

    install(
      TARGETS myplugin
      DESTINATION ${PLUGIN_PATH}
    )
```

## Continuous Integration

The project uses GitHub Actions for automated testing on multiple platforms:
* **Ubuntu 24.04** with Qt 5.15
* **Windows** with Qt 5.12.12 and Qt 5.15.2

All builds enforce strict compiler warnings (`-Werror -Wall`) to maintain code quality.

## Running Veranda

After building the project, source the workspace and launch the Qt frontend:

```bash
source install/setup.bash
ros2 run veranda_qt_frontend veranda
```

### Demo Scripts

Example Python scripts for controlling robots are available in the `veranda/Demo/Scripts/` directory. These scripts include:
* Differential drive control (joystick and figure-8 patterns)
* Omnidirectional drive control
* Ackermann steering control
* Sensor listeners (LIDAR, GPS, touch sensors)

## Project Documentation

Detailed instructions for building, running, and developing this project are found in the User Manual and Design Document which can be built from this repository. Building these documents requires these command line tools to be installed:
* pdflatex
* doxygen
* sphinx

To build the documentation, clone the repository and run the make_documentation script in the veranda subfolder. This will generate a new Documentation folder with the following:
* Online Doxygen Reference
* Online Sphinx User Manual
* PDF Doxygen Reference
* PDF Sphinx User Manual
* Design Documentation

The design documentation contains the other two PDF documents within itself, as well as details of the internal design of the simulator and a record of its development.

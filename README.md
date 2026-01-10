# Veranda: A 2-Dimensional Mobile Robotics Simulation Environment

This repository contains the code for a 2D Robotics simulation tool. The tool is written to interface with other applications through the ROS 2 communication layer, and is usable on all operating systems that can have Qt 5 and ROS 2 installed.

## Requirements

### Core Dependencies
* **ROS 2 Jazzy Jalisco** - The project targets ROS 2 Jazzy (Ubuntu 24.04 LTS)
* **Qt 5 LTS** - Qt 5.12.12 or Qt 5.15.2 (other Qt 5 versions may work)
* **CMake 3.28+** - Modern CMake with target-based patterns
* **C++17** - Standard required for compilation
* **Python 3.9+** - Required for build system and scripts

### Build System
* **Colcon** - ROS 2 build tool (`python3-colcon-common-extensions`)
* **ament_cmake** - CMake macros for ROS 2 packages
* **rosdep** - Dependency management tool

### Platform Support
* **Ubuntu 24.04** - Primary development platform with ROS 2 Jazzy
* **Windows** - Supported via ROS 2 Jazzy for Windows with MSVC compiler

## Building the Project

The project uses modern CMake patterns with target-based dependency management and is built using the Colcon build system. All legacy qmake build files have been removed in favor of CMake.

### Ubuntu 24.04

```bash
# Install ROS 2 Jazzy and dependencies
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  qtbase5-dev \
  qtdeclarative5-dev \
  libqt5svg5-dev

# Initialize rosdep (if not already done)
sudo rosdep init
rosdep update

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

Follow the ROS 2 Jazzy installation instructions for Windows, then use the GitHub Actions workflow (`.github/workflows/build.yml`) as a reference for the complete build process.

## Plugin Architecture

Veranda uses a plugin-based architecture to extend functionality with custom sensors, shapes, wheels, and UI components. Plugins are built as shared libraries using modern CMake target-based patterns.

### Plugin Types
* **Logic Plugins** - Core simulation components (sensors, shapes, wheels)
* **Qt Plugins** - UI wrappers for logic plugins
* **Frontend Plugins** - File handlers and UI extensions

### Plugin Development
All plugins use:
* **CMake 3.28+** with target-based dependency management
* **ament_cmake** build macros for ROS 2 integration
* **Modern C++17** standards
* **Qt 5 Plugin System** for dynamic loading

Plugin CMakeLists.txt files use `find_package()` with imported targets instead of legacy variable-based patterns. Refer to the built-in plugin packages in `veranda/Packages/` for examples.

## Continuous Integration

The project uses GitHub Actions for automated testing on multiple platforms:
* **Ubuntu 24.04** with Qt 5.15
* **Windows** with Qt 5.12.12 and Qt 5.15.2

All builds enforce strict compiler warnings (`-Werror -Wall`) to maintain code quality.

## Running Veranda

After building the project, source the workspace and launch the Qt frontend:

```bash
source install/setup.bash
ros2 run veranda_qt_frontend veranda_qt_frontend
```

### Demo Scripts

Example Python scripts for controlling robots are available in the `veranda/Demo/Scripts/` directory. These scripts have been updated to use ROS 2 Jazzy syntax and include:
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

## Recent Modernization Changes

This project has undergone significant modernization:
* **ROS 2 Jazzy Jalisco** - Upgraded from ROS Ardent/Bouncy/Crystal
* **Qt 5 LTS** - Updated from Qt 4 to Qt 5.12+ with modern CMake integration
* **CMake 3.28** - Modernized to use target-based patterns and imported targets
* **Build System** - Removed all legacy qmake files (.pro, .pri); now CMake-only
* **Compiler Standards** - Enabled `-Werror -Wall` for strict compilation
* **CI/CD** - Added GitHub Actions workflows for Ubuntu 24.04 and Windows
* **Python Scripts** - Updated demo scripts to ROS 2 Jazzy API syntax
* **Testing** - Upgraded to Catch2 2.13.10 for unit testing

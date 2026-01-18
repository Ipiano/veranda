Making A Custom Plugin
======================

Custom plugins are ROS 2 packages built in the same workspace as Veranda. Plugins use CMake 3.28+ with ament_cmake for building and dependency management.

Package Dependencies
--------------------

The plugin's ``package.xml`` file must specify dependencies to ensure correct build order. For a logic plugin (sensor, shape, or wheel):

.. code-block:: xml

    <?xml version="1.0"?>
    <package format="3">
      <name>my_custom_plugin</name>
      <version>0.1.0</version>
      <description>My custom Veranda plugin</description>
      <maintainer email="you@example.com">Your Name</maintainer>
      <license>TODO</license>

      <buildtool_depend>ament_cmake</buildtool_depend>

      <build_depend>veranda_core_api</build_depend>
      <build_depend>veranda_box2d</build_depend>
      <build_depend>rclcpp</build_depend>

      <exec_depend>rclcpp</exec_depend>

      <export>
        <build_type>ament_cmake</build_type>
      </export>
    </package>

For a Qt plugin wrapper, also add:

.. code-block:: xml

    <build_depend>veranda_qt_plugin_api</build_depend>

CMakeLists.txt Configuration
----------------------------

Plugin CMakeLists.txt files use CMake 3.28+ with ament and the ``ament_target_dependencies`` function. Here's a complete example:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.28)

    project(my_custom_plugin
        VERSION 0.1.0
        LANGUAGES CXX
    )

    # C++17 standard
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Find ament and ROS 2 packages
    find_package(ament_cmake REQUIRED)
    find_package(veranda_core_api REQUIRED)
    find_package(veranda_box2d REQUIRED)
    find_package(rclcpp REQUIRED)

    # Find Qt 5 with modern imported targets
    find_package(Qt5 5.12 REQUIRED COMPONENTS Core Gui)

    # Enable Qt MOC
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

    # Export dependencies for downstream packages
    ament_export_dependencies(
        veranda_core_api
        veranda_box2d
        rclcpp
    )

    ament_export_include_directories(include/${PROJECT_NAME})
    ament_export_libraries(${PROJECT_NAME})

    # Source files
    set(SOURCES
        src/my_plugin.cpp
        src/my_component.cpp
    )

    set(HEADERS
        include/my_plugin/my_plugin.h
        include/my_plugin/my_component.h
    )

    # Create shared library
    add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

    # Set C++17 standard
    target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

    # Include directories with generator expressions
    target_include_directories(${PROJECT_NAME}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}
    )

    # Add compile definitions for Qt plugin
    target_compile_definitions(${PROJECT_NAME}
        PRIVATE
            QT_DEPRECATED_WARNINGS
            QT_PLUGIN
            QT_SHARED
    )

    # Link ROS dependencies using ament_target_dependencies
    # This handles include paths and libraries automatically
    ament_target_dependencies(${PROJECT_NAME}
        PUBLIC
            rclcpp
            veranda_core_api
            veranda_box2d
    )

    # Link Qt modules using target_link_libraries
    # Qt is a pure CMake package, not an ament package
    target_link_libraries(${PROJECT_NAME}
        PUBLIC
            Qt5::Core
            Qt5::Gui
    )

    # Get plugin installation path from ament index
    ament_index_get_resource(PLUGIN_PATH "veranda_plugin_path" "veranda_qt_frontend")

    # Install plugin to discovered plugin directory
    install(
        TARGETS ${PROJECT_NAME}
        DESTINATION ${PLUGIN_PATH}
    )

    # Install headers for downstream use
    install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
    )

    # Finalize ament package
    ament_package()

Plugin Discovery
----------------

Veranda discovers plugins through the ament index. The plugin path is exposed as an ament index resource named ``"veranda_plugin_path"`` registered by the ``veranda_qt_frontend`` package.

As shown in the CMakeLists.txt example above, use ``ament_index_get_resource()`` to get the plugin installation path, then install your plugin to that location.

At runtime, Veranda scans all ``.so`` files in this directory and uses Qt's ``QPluginLoader`` to check for plugin compatibility. The loader attempts to load each library and verifies it implements the required plugin interfaces (``WorldObjectComponent_If`` for logic plugins, ``Object_Ui_If`` for Qt wrappers, etc.).

A Note on ROS Communications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is believed by the project team that, because the rclcpp::spin() method may be called from a thread other than the main one, all ROS message callbacks may be run 
from a non-main thread. Care should be taken when defining callbacks in components to prevent race conditions which would result from this design. This issue was 
resolved in the Touch Sensor Ring plugin with the use of a Qt Signal and Slot. When a Qt Signal triggers a Slot of an object which resides in a different thread, 
the slot is queued for the second thread to receive naturally during its event loop, preventing any race conditions. The Touch Sensor Ring component has an internal 
signal and slot specifically for this purpose; the signal is emitted by the ROS callback function, and that copies the data from the callback into the main thread 
where it can be processed safely.


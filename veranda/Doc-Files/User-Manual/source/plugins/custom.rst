Making A Custom Plugin
======================

Custom plugins are ROS 2 packages built in the same workspace as Veranda. The project uses modern CMake 3.28+ patterns with target-based dependency management.

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

CMakeLists.txt - Modern CMake Configuration
--------------------------------------------

Modern CMake uses target-based patterns instead of global variables. Here's a complete example:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.28)

    project(my_custom_plugin
        VERSION 0.1.0
        LANGUAGES CXX
    )

    # C++17 standard
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Find ROS 2 packages using modern imported targets
    find_package(ament_cmake REQUIRED)
    find_package(veranda_core_api REQUIRED)
    find_package(veranda_box2d REQUIRED)
    find_package(rclcpp REQUIRED)

    # Find Qt 5 with modern imported targets
    find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

    # Enable Qt MOC
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)

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

    # Set target properties for Qt plugin
    set_target_properties(${PROJECT_NAME} PROPERTIES
        POSITION_INDEPENDENT_CODE ON
    )

    # Link using modern imported targets
    target_link_libraries(${PROJECT_NAME}
        veranda_core_api::veranda_core_api
        veranda_box2d::veranda_box2d
        rclcpp::rclcpp
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
    )

    # Include directories
    target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

    # Install plugin library
    install(
        TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )

    # Install headers
    install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
    )

    # Export dependencies
    ament_export_dependencies(
        veranda_core_api
        veranda_box2d
        rclcpp
    )

    # Generate ament package
    ament_package()

Key Differences from Legacy Patterns
-------------------------------------

The modern CMake approach differs from older patterns:

* **CMake 3.28**: Minimum version instead of older 2.x or 3.5
* **Imported Targets**: Use ``veranda_core_api::veranda_core_api`` instead of ``${veranda_core_api_LIBRARIES}``
* **Qt 5**: Use ``Qt5::Core`` targets instead of ``qt5_use_modules()``
* **No Manual MOC**: ``CMAKE_AUTOMOC`` handles Qt preprocessing automatically
* **Target-based Linking**: ``target_link_libraries()`` with imported targets
* **Modern Include Dirs**: Generator expressions for build/install interface
* **No Global Includes**: Avoid ``include_directories()``; use ``target_include_directories()``

Plugin Discovery
----------------

Veranda discovers plugins through the ament index. Plugins are automatically found when installed to the workspace ``lib`` directory. Ensure your plugin is properly installed using the ``install(TARGETS ...)`` command shown above.

A Note on ROS Communications
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is believed by the project team that, because the rclcpp::spin() method may be called from a thread other than the main one, all ROS message callbacks may be run 
from a non-main thread. Care should be taken when defining callbacks in components to prevent race conditions which would result from this design. This issue was 
resolved in the Touch Sensor Ring plugin with the use of a Qt Signal and Slot. When a Qt Signal triggers a Slot of an object which resides in a different thread, 
the slot is queued for the second thread to receive naturally during its event loop, preventing any race conditions. The Touch Sensor Ring component has an internal 
signal and slot specifically for this purpose; the signal is emitted by the ROS callback function, and that copies the data from the callback into the main thread 
where it can be processed safely.


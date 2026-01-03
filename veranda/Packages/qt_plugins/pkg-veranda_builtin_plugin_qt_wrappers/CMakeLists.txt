cmake_minimum_required(VERSION 3.16)
project(veranda_builtin_plugin_qt_wrappers)

## Compile as C++17 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(veranda_qt_plugin_api REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_builtin_sensors REQUIRED)
find_package(veranda_builtin_wheels REQUIRED)
find_package(veranda_builtin_shapes REQUIRED)

## Find and configure QT
## Qt 5.15 LTS is the latest Qt 5 version with long-term support
set(QT_COMPONENTS Core Gui)
find_package(Qt5 5.12 REQUIRED COMPONENTS
  ${QT_COMPONENTS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## Enable Qt deprecation warnings to prepare for future Qt 6 migration
add_compile_definitions(QT_DEPRECATED_WARNINGS QT_PLUGIN QT_SHARED)

include_directories( include )
include_directories( ${CMAKE_BINARY_DIR} )

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(
    rclcpp
    veranda_core_api
    veranda_qt_plugin_api
    veranda_builtin_sensors
    veranda_builtin_wheels
    veranda_builtin_shapes
)

# Need to include this early so that moc is happy
include_directories(${veranda_core_api_INCLUDE_DIRS})
include_directories(${veranda_qt_plugin_api_INCLUDE_DIRS})
include_directories(${veranda_builtin_sensors_INCLUDE_DIRS})
include_directories(${veranda_builtin_wheels_INCLUDE_DIRS})
include_directories(${veranda_builtin_shapes_INCLUDE_DIRS})

###########################################
## Make a plugin from source and headers ##
###########################################
function(make_plugin plugin_name plugin_moc_hdrs plugin_srcs plugin_roslib)

    ## Build the plugin lib
    ## AUTOMOC will automatically generate MOC files for headers with Q_OBJECT
    add_library(${plugin_name} SHARED ${plugin_srcs} ${plugin_moc_hdrs})

    ## Pull in stuff exported from other packages
    ament_target_dependencies(${plugin_name}
        PUBLIC
            "rclcpp"
            "veranda_qt_plugin_api"
            "veranda_core_api"
            "${plugin_roslib}")

    ## Link Qt modules using modern target-based approach
    target_link_libraries(${plugin_name}
        PUBLIC
            Qt5::Core
            Qt5::Gui
    )

    ament_index_get_resource(PLUGIN_PATH "veranda_plugin_path" "veranda_qt_frontend")

    ## Install lib into same directory as simulator executable
    install(
      TARGETS ${plugin_name}
      DESTINATION ${PLUGIN_PATH}
    )
endfunction()

###########
## Build ##
###########
# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

set(headers include/fixed_wheel_plugin.h)
set(sources src/fixed_wheel_plugin.cpp)
make_plugin(fixed_wheel_plugin "${headers}" "${sources}" veranda_builtin_wheels)

set(headers include/ackermann_steer_plugin.h)
set(sources src/ackermann_steer_plugin.cpp)
make_plugin(ackermann_steer_plugin "${headers}" "${sources}" veranda_builtin_wheels)

set(headers include/omni_drive_plugin.h)
set(sources src/omni_drive_plugin.cpp)
make_plugin(omni_drive_plugin "${headers}" "${sources}" veranda_builtin_wheels)

set(headers include/gps_sensor_plugin.h)
set(sources src/gps_sensor_plugin.cpp)
make_plugin(gps_sensor_plugin "${headers}" "${sources}" veranda_builtin_sensors)

set(headers include/lidar_sensor_plugin.h)
set(sources src/lidar_sensor_plugin.cpp)
make_plugin(lidar_sensor_plugin "${headers}" "${sources}" veranda_builtin_sensors)

set(headers include/touch_sensor_plugin.h)
set(sources src/touch_sensor_plugin.cpp)
make_plugin(touch_sensor_plugin "${headers}" "${sources}" veranda_builtin_sensors)

set(headers include/circle_plugin.h)
set(sources src/circle_plugin.cpp)
make_plugin(circle_plugin "${headers}" "${sources}" veranda_builtin_shapes)

set(headers include/rectangle_plugin.h)
set(sources src/rectangle_plugin.cpp)
make_plugin(rectangle_plugin "${headers}" "${sources}" veranda_builtin_shapes)

set(headers include/polygon_plugin.h)
set(sources src/polygon_plugin.cpp)
make_plugin(polygon_plugin "${headers}" "${sources}" veranda_builtin_shapes)

ament_package()


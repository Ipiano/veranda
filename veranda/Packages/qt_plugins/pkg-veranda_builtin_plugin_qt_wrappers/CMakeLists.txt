cmake_minimum_required(VERSION 3.28)
project(veranda_builtin_plugin_qt_wrappers)

## Compile as C++17 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(
    rclcpp
    veranda_core_api
    veranda_qt_plugin_api
    veranda_builtin_sensors
    veranda_builtin_wheels
    veranda_builtin_shapes
)

###########################################
## Make a plugin from source and headers ##
###########################################
function(make_plugin plugin_name plugin_moc_hdrs plugin_srcs plugin_roslib)

    ## Build the plugin lib
    ## AUTOMOC will automatically generate MOC files for headers with Q_OBJECT
    add_library(${plugin_name} SHARED ${plugin_srcs} ${plugin_moc_hdrs})

    target_compile_features(${plugin_name} PUBLIC cxx_std_17)

    target_include_directories(${plugin_name}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        PRIVATE
            ${CMAKE_CURRENT_BINARY_DIR}
    )

    target_compile_definitions(${plugin_name}
        PRIVATE
            QT_DEPRECATED_WARNINGS
            QT_PLUGIN
            QT_SHARED
    )

    target_compile_options(${plugin_name}
        PRIVATE
            $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Werror>
            $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
    )

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


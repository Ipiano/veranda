cmake_minimum_required(VERSION 3.28)
project(veranda_qt_frontend_image_file_handler)

## Compile as C++17 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_qt_frontend_plugin_api REQUIRED)
find_package(rclcpp REQUIRED)

## Find and configure QT
## Qt 5.15 LTS is the latest Qt 5 version with long-term support
find_package(Qt5 5.12 REQUIRED COMPONENTS
  Core
  Gui
  Widgets
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(    
    rclcpp
    std_msgs
    veranda_core_api
    veranda_box2d
    veranda_qt_frontend_plugin_api
)

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/image_loader.h
    include/image_loader_plugin.h
    include/optiondialog.h)

## Any other source files
set(CPP_SRCS
    src/image_loader.cpp
    src/image_loader_plugin.cpp
    src/imageparser.cpp)

###########
## Build ##
###########
# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Build the plugin library
## AUTOMOC will automatically generate MOC files for headers with Q_OBJECT
add_library(image_loader_plugin SHARED ${CPP_SRCS} ${MOC_HDRS})

target_compile_features(image_loader_plugin PUBLIC cxx_std_17)

target_include_directories(image_loader_plugin
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(image_loader_plugin
    PRIVATE
        QT_DEPRECATED_WARNINGS
        QT_PLUGIN
        QT_SHARED
)

target_compile_options(image_loader_plugin
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Werror>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
)

## Pull in stuff exported from other packages
ament_target_dependencies(image_loader_plugin
    PUBLIC
        "veranda_core_api"
        "veranda_box2d"
        "rclcpp"
        "veranda_qt_frontend_plugin_api"
)

## Link Qt modules using modern target-based approach
target_link_libraries(image_loader_plugin
    PUBLIC
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
)

ament_index_get_resource(PLUGIN_PATH "veranda_plugin_path" "veranda_qt_frontend")

## Install lib into same directory as simulator executable
install(
  TARGETS image_loader_plugin
  DESTINATION ${PLUGIN_PATH}
)

ament_package()


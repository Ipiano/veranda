###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.28)

project(veranda_qt_frontend_plugin_api
    VERSION 0.1.0
    LANGUAGES CXX
)

set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

## Compile as C++11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_qt_plugin_api REQUIRED)

ament_export_dependencies(
    veranda_box2d
    veranda_core_api
    veranda_qt_plugin_api
)

ament_export_include_directories(${HEADER_INSTALL_DIR})

###################
## FILE LISTINGS ##
###################
set(HEADERS_DIR include/veranda_qt_frontend)

###########
## Build ##
###########

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY ${HEADERS_DIR}
  DESTINATION ${HEADER_INSTALL_DIR}
)

ament_package()


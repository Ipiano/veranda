cmake_minimum_required(VERSION 3.28)

project(veranda_builtin_shapes
    VERSION 0.1.0
    LANGUAGES CXX
)

set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

## On ubuntu (and presumably other unix), install/lib is
## added to LD_LIBRARY_PATH; but on windows, install/bin
## is added
set(LIB_INSTALL_DIR lib)
if(WIN32)
    set(LIB_INSTALL_DIR bin)
endif()

## Compile as C++17 Library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_box2d REQUIRED)

## Find and configure QT
## Qt 5.15 LTS is the latest Qt 5 version with long-term support
set(QT_COMPONENTS Core Gui)
find_package(Qt5 5.12 REQUIRED COMPONENTS
  ${QT_COMPONENTS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

ament_export_dependencies(
    rclcpp
    veranda_core_api
    veranda_box2d
)

ament_export_include_directories(${HEADER_INSTALL_DIR})
ament_export_libraries(${PROJECT_NAME})

###################
## FILE LISTINGS ##
###################
set(HEADERS_DIR include/veranda_shapes)

set(MOC_HDRS
        ${HEADERS_DIR}/circle.h
        ${HEADERS_DIR}/polygon.h
        ${HEADERS_DIR}/rectangle.h)

## Any other source files
set(CPP_SRCS
    src/circle.cpp
    src/polygon.cpp
    src/rectangle.cpp
    src/triangulator.cpp)

###########
## Build ##
###########

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Build Library
## AUTOMOC will automatically generate MOC files for headers with Q_OBJECT
add_library(${PROJECT_NAME} SHARED ${CPP_SRCS} ${MOC_HDRS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

find_package(OpenGL REQUIRED)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/veranda_shapes
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        VERANDA_SHAPES_BUILD_DLL
        $<$<PLATFORM_ID:Windows>:WINDOWS>
        QT_DEPRECATED_WARNINGS
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

## Pull in stuff exported from other packages
ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
        "rclcpp"
        "veranda_box2d"
        "veranda_core_api"
)

## Link Qt modules and OpenGL using modern target-based approach
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Qt5::Core
        Qt5::Gui
        ${OPENGL_LIBRARIES}
)

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY ${HEADERS_DIR}
  DESTINATION ${HEADER_INSTALL_DIR}
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION ${LIB_INSTALL_DIR}
)

ament_package()


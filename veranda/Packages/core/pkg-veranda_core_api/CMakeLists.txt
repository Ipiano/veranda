###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.28)

project(veranda_core_api
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

## Compile as C++17
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(rclcpp REQUIRED)

## Find and configure QT
## Qt 5.15 LTS is the latest Qt 5 version with long-term support
set(QT_COMPONENTS Core Gui Widgets)
find_package(Qt5 5.12 REQUIRED COMPONENTS
    ${QT_COMPONENTS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

ament_export_dependencies(
    veranda_box2d
    rclcpp
)

ament_export_include_directories( ${HEADER_INSTALL_DIR} )
ament_export_libraries(${PROJECT_NAME})

###################
## FILE LISTINGS ##
###################
set(HEADERS_DIR include/veranda_core)

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    ${HEADERS_DIR}/api/world_object.h
    ${HEADERS_DIR}/api/model.h
    ${HEADERS_DIR}/api/property.h
    ${HEADERS_DIR}/api/world_object_component.h
    ${HEADERS_DIR}/api/filter.h
    ${HEADERS_DIR}/api/interfaces/simulator_ui_if.h
    ${HEADERS_DIR}/api/interfaces/simulator_physics_if.h
    ${HEADERS_DIR}/api/interfaces/world_object_wrappers.h
)

set(CPP_SRCS
    src/world_object.cpp
    src/world_object_component.cpp
    src/filter.cpp
)

###########
## Build ##
###########

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Run MOC on files with Qt Keywords and Macros
qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

## Build Library
add_library(${PROJECT_NAME} SHARED ${CPP_SRCS} ${MOC_SRCS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        VERANDA_CORE_API_BUILD_DLL
        QT_DEPRECATED_WARNINGS
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Werror>
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
)

## Pull in stuff exported from other packages
ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
        "veranda_box2d"
        "rclcpp"
)

## Link Qt modules using modern target-based approach
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
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

#############
## Tests ##
#############

if(BUILD_TESTING)
    find_package(veranda_catch2 REQUIRED)

    set(CATCH_CMAKE "${veranda_catch2_INCLUDE_DIRS}/catch2/ament_cmake_add_catch_test.cmake")
    include(${CATCH_CMAKE})

    ament_add_catch_test(
        "model_datatype_tests"
        CPP_SOURCES tests/test_model.cpp
        QT_LIBS Core Gui
        LIBS ${PROJECT_NAME}
        ROS_LIBS "veranda_box2d"
        )

    ament_add_catch_test(
        "property_datatype_tests"
        CPP_SOURCES tests/test_property.cpp
        QT_LIBS Core Gui
        LIBS ${PROJECT_NAME}
        )
endif()

ament_package()


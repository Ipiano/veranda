###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.16)

set(EXE_NAME veranda)
set(PROJECT_NAME veranda_qt_frontend)
project(${PROJECT_NAME})

## Compile as C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

## Find required ROS packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(veranda_core_api REQUIRED)
find_package(veranda_core REQUIRED)
find_package(veranda_qt_frontend_plugin_api REQUIRED)
find_package(veranda_qt_plugin_api REQUIRED)

## Find and configure QT
## Qt 5.15 LTS is the latest Qt 5 version with long-term support
set(QT_COMPONENTS Core Widgets Gui)
find_package(Qt5 5.12 REQUIRED COMPONENTS
  ${QT_COMPONENTS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

## Enable Qt deprecation warnings to prepare for future Qt 6 migration
add_compile_definitions(QT_DEPRECATED_WARNINGS)

include_directories( include ${CMAKE_BINARY_DIR} )

ament_export_dependencies(    
    rclcpp
    std_msgs
    sensor_msgs
    veranda_box2d
    veranda_core_api
    veranda_core
    veranda_qt_frontend_plugin_api
    veranda_qt_plugin_api
)

ament_index_register_resource(veranda_plugin_path CONTENT "${CMAKE_INSTALL_PREFIX}/lib/veranda_qt_frontend")

###################
## FILE LISTINGS ##
###################

## Files that have any Qt macros or keywords in them
## Q_OBJECT, Q_PROPERTY, signals, slots.... etc.
set(MOC_HDRS
    include/interfaces/simulator_visual_if.h

    include/ui/mainwindow.h
    include/ui/joystickprototype.h
    include/ui/mode_controller.h
    include/ui/settingspopup.h
    include/ui/qgraphicssimulationviewer.h
    include/ui/customgraphicsview.h
    )

## .ui qt widgets form files
set(UI_FILES
    ui/mainwindow.ui
    ui/joystickprototype.ui
    ui/settingspopup.ui
    ui/qgraphicssimulationviewer.ui)

## Any other source files
set(CPP_SRCS
    src/ui/mainwindow.cpp
    src/ui/joystickprototype.cpp
    src/ui/mode_controller.cpp
    src/ui/settingspopup.cpp
    src/ui/designer_widget.cpp
    src/ui/qgraphicssimulationviewer.cpp

    src/main.cpp
    )

set(RCC_FILES ui/resources.qrc)

###########
## Build ##
###########

# Need to include this early to make the MOC happy
include_directories("${veranda_box2d_INCLUDE_DIRS}")
include_directories("${veranda_core_api_INCLUDE_DIRS}")
include_directories("${veranda_qt_frontend_plugin_api_INCLUDE_DIRS}")

# Use new MOC policy if CMake 3.10+
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.10")
    cmake_policy(SET CMP0071 NEW)
endif()

## Wrap .ui files manually since they're in a different directory than sources
qt5_wrap_ui(UI_SRCS ${UI_FILES})

## Build executable
## AUTOMOC and AUTORCC will automatically handle MOC and resource files
add_executable(${EXE_NAME} ${CPP_SRCS} ${MOC_HDRS} ${UI_SRCS} ${RCC_FILES})

## Pull in stuff exported from other packages
ament_target_dependencies(${EXE_NAME}
    PUBLIC
        "rclcpp"
        "ament_index_cpp"
        "std_msgs"
        "sensor_msgs"
        "veranda_box2d"
        "veranda_core_api"
        "veranda_core"
        "veranda_qt_frontend_plugin_api"
        "veranda_qt_plugin_api"
)

## Link Qt modules using modern target-based approach
target_link_libraries(${EXE_NAME}
    PRIVATE
        Qt5::Core
        Qt5::Widgets
        Qt5::Gui
)

#############
## Install ##
#############

## Install executable
install(
  TARGETS ${EXE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
        config
        launch
        DESTINATION share/${PROJECT_NAME}/
        )

## On windows, we need to run windeployqt
## To set up plugins and dlls because Windows
if(WIN32)
    install(CODE 
        "
        MESSAGE(\"Running WindDeployQt.\")

        execute_process (
            COMMAND cmd /c echo ${CMAKE_INSTALL_PREFIX}
        )

        execute_process (
            COMMAND cmd /c \"%QT5_DIR%/bin/windeployqt ${EXE_NAME}.exe --no-compiler-runtime  --no-translations\"
            WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
        )
        "
    )
endif()

ament_package()


cmake_minimum_required(VERSION 3.28)
project(veranda_qt_frontend_json_file_handler)

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
  Core Gui
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

## TODO If possible: Figure out how to mark Qt as a dependency
ament_export_dependencies(    
    rclcpp
    veranda_core_api
    veranda_box2d
    veranda_qt_frontend_plugin_api
    veranda_qt_frontend
)


###########################################
## Make a plugin from source and headers ##
###########################################
function(make_plugin plugin_name plugin_moc_hdrs plugin_srcs)

    ## Build the plugin lib
    ## AUTOMOC will automatically generate MOC files for headers with Q_OBJECT
    add_library(${plugin_name} SHARED ${plugin_srcs} ${plugin_moc_hdrs})

    target_compile_features(${plugin_name} PUBLIC cxx_std_17)

    target_include_directories(${plugin_name}
        PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
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
            $<$<CXX_COMPILER_ID:GNU>:-Wall>
            $<$<CXX_COMPILER_ID:Clang>:-Wall>
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )

    ## Pull in stuff exported from other packages
    ament_target_dependencies(${plugin_name}
        PUBLIC
            "rclcpp"
            "veranda_box2d"
            "veranda_core_api"
            "veranda_qt_frontend_plugin_api")

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

set(headers include/json_object.h include/json_object_plugin.h)
set(sources src/json_object.cpp src/json_object_plugin.cpp)
make_plugin(json_object_file_plugin "${headers}" "${sources}")

set(headers include/json_world.h include/json_world_plugin.h)
set(sources src/json_world.cpp src/json_world_plugin.cpp)
make_plugin(json_world_file_plugin "${headers}" "${sources}")


ament_package()


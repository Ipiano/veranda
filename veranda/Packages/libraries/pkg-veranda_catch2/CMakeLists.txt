###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.28)

project(veranda_catch2
    VERSION 0.1.0
    LANGUAGES CXX
)

set(HEADER_INSTALL_DIR include/${PROJECT_NAME})

find_package(ament_cmake REQUIRED)

ament_export_include_directories(${HEADER_INSTALL_DIR})

#############
## Install ##
#############
set(CATCH2_DIR include/catch2)

## Install include dir
install(
  DIRECTORY ${CATCH2_DIR}
  DESTINATION ${HEADER_INSTALL_DIR}
)

ament_package()

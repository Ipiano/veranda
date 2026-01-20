###########
## SETUP ##
###########
cmake_minimum_required(VERSION 3.28)

project(veranda_box2d
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

ament_export_include_directories(${HEADER_INSTALL_DIR})
ament_export_libraries(${PROJECT_NAME})

###################
## FILE LISTINGS ##
###################

set(BOX2D_SUBPATH ./include)
set(BOX2D_PATH ${BOX2D_SUBPATH}/Box2D)
set(BOX2D_SRCS

    ${BOX2D_PATH}/Common/b2BlockAllocator.cpp
    ${BOX2D_PATH}/Common/b2Draw.cpp
    ${BOX2D_PATH}/Common/b2Math.cpp
    ${BOX2D_PATH}/Common/b2Settings.cpp
    ${BOX2D_PATH}/Common/b2StackAllocator.cpp
    ${BOX2D_PATH}/Common/b2Timer.cpp

    ${BOX2D_PATH}/Collision/b2BroadPhase.cpp
    ${BOX2D_PATH}/Collision/b2CollideCircle.cpp
    ${BOX2D_PATH}/Collision/b2CollidePolygon.cpp
    ${BOX2D_PATH}/Collision/b2CollideEdge.cpp
    ${BOX2D_PATH}/Collision/b2Collision.cpp
    ${BOX2D_PATH}/Collision/b2Distance.cpp
    ${BOX2D_PATH}/Collision/b2DynamicTree.cpp
    ${BOX2D_PATH}/Collision/b2TimeOfImpact.cpp
    
    ${BOX2D_PATH}/Collision/Shapes/b2ChainShape.cpp
    ${BOX2D_PATH}/Collision/Shapes/b2CircleShape.cpp
    ${BOX2D_PATH}/Collision/Shapes/b2EdgeShape.cpp
    ${BOX2D_PATH}/Collision/Shapes/b2PolygonShape.cpp
                        
    ${BOX2D_PATH}/Dynamics/b2Body.cpp
    ${BOX2D_PATH}/Dynamics/b2ContactManager.cpp
    ${BOX2D_PATH}/Dynamics/b2Fixture.cpp
    ${BOX2D_PATH}/Dynamics/b2Island.cpp
    ${BOX2D_PATH}/Dynamics/b2WorldCallbacks.cpp
    ${BOX2D_PATH}/Dynamics/b2World.cpp

    ${BOX2D_PATH}/Dynamics/Contacts/b2ChainAndCircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2ChainAndPolygonContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2CircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2Contact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2ContactSolver.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2EdgeAndCircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2EdgeAndPolygonContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2PolygonAndCircleContact.cpp
    ${BOX2D_PATH}/Dynamics/Contacts/b2PolygonContact.cpp
    
    ${BOX2D_PATH}/Dynamics/Joints/b2DistanceJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2Joint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2PrismaticJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2RopeJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2FrictionJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2MotorJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2PulleyJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2WeldJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2GearJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2MouseJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2RevoluteJoint.cpp
    ${BOX2D_PATH}/Dynamics/Joints/b2WheelJoint.cpp

    ${BOX2D_PATH}/Rope/b2Rope.cpp
)

###########
## Build ##
###########

## Build Box2d
add_library(${PROJECT_NAME} SHARED ${BOX2D_SRCS})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${BOX2D_SUBPATH}>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        BOX2D_DLL
)

#############
## Install ##
#############

## Install include dir
install(
  DIRECTORY ${BOX2D_PATH}
  DESTINATION ${HEADER_INSTALL_DIR}
)

## Install library

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Debug: Print installation details
install(CODE "
  message(STATUS \"========== VERANDA_BOX2D INSTALL DEBUG ==========\")
  message(STATUS \"CMAKE_INSTALL_PREFIX: \${CMAKE_INSTALL_PREFIX}\")
  message(STATUS \"Checking installed files in: \${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}\")

  file(GLOB_RECURSE INSTALLED_FILES \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/*\")
  foreach(FILE \${INSTALLED_FILES})
    message(STATUS \"  Installed: \${FILE}\")
  endforeach()

  set(CONFIG_FILE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Config.cmake\")
  if(EXISTS \"\${CONFIG_FILE}\")
    message(STATUS \"✓ Config file EXISTS: \${CONFIG_FILE}\")
  else()
    message(WARNING \"✗ Config file NOT FOUND: \${CONFIG_FILE}\")
  endif()

  set(TARGETS_FILE \"\${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake/${PROJECT_NAME}Targets.cmake\")
  if(EXISTS \"\${TARGETS_FILE}\")
    message(STATUS \"✓ Targets file EXISTS: \${TARGETS_FILE}\")
  else()
    message(WARNING \"✗ Targets file NOT FOUND: \${TARGETS_FILE}\")
  endif()
  message(STATUS \"=================================================\")
")

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

ament_package()

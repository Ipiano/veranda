# CMake Modernization Plan

This document outlines the work required to modernize the CMake build system from legacy CMake 3.5 patterns to modern CMake 3.16+ conventions.

> **Related documents**:
> - [TODO.md](TODO.md) - Phase 1.3 (Bare bones CMake), Phase 2.6 (Full modernization)
> - [TODO_upgrade_qt.md](TODO_upgrade_qt.md) - Qt6 upgrade requires CMake changes
> - [TODO_update_ros.md](TODO_update_ros.md) - ROS2 Jazzy requires CMake 3.16+
>
> **Note**: This document covers the **full modernization** (Phase 2.6). For bare minimum changes needed for Qt6/ROS2 compatibility, see TODO.md Phase 1.3.

## Current State

- **Current CMake Version**: 3.16 minimum (updated in wt-upgrade-qt)
- **Target CMake Version**: 3.16+ minimum (3.22+ recommended for full modern features)
- **Build System**: ament_cmake (ROS 2)

### Completed Work

The following CMake modernization tasks have been completed as part of the Qt 5.12/5.15 upgrade:

- ✅ Updated `cmake_minimum_required(VERSION 3.16)` in all packages
- ✅ Replaced `qt5_use_modules()` with `target_link_libraries(Qt5::Core, etc.)`
- ✅ Replaced `add_definitions()` with `target_compile_definitions()` for DLL exports
- ✅ Added `CMAKE_CXX_STANDARD_REQUIRED ON` to all packages
- ✅ Updated C++ standard to C++17
- ✅ Used generator expressions for platform-specific definitions
- ✅ Updated `ament_cmake_add_catch_test.cmake` with modern patterns

### Remaining Legacy Patterns

| Pattern | Count | Severity | Status |
|---------|-------|----------|--------|
| `include_directories()` (global scope) | 25+ | High | TODO |
| `add_definitions()` (global scope) | ~5 | High | Partially done |
| `CMAKE_CXX_FLAGS` modification | 12 | Medium | TODO |
| `qt5_use_modules()` (deprecated) | 0 | High | ✅ Done |
| `qt5_wrap_cpp()` (prefer AUTOMOC) | 1 | Low | Kept for header-only QObjects |
| `ament_target_dependencies()` | 14 | Medium | TODO |
| `set(PROJECT_NAME ...)` before `project()` | 10 | Low | TODO |

---

## Modern CMake Principles

### Core Philosophy

Modern CMake is **target-based** rather than **directory-based**:

1. **Targets are first-class citizens** - Everything attaches to targets
2. **Explicit scope** - Always use PUBLIC/PRIVATE/INTERFACE
3. **No global pollution** - Avoid commands that affect global state
4. **Imported targets** - Link against targets, not variables
5. **Generator expressions** - Use for conditional logic at generate time

### Key Modern CMake Commands

| Legacy Command | Modern Replacement |
|----------------|-------------------|
| `include_directories()` | `target_include_directories()` |
| `add_definitions()` | `target_compile_definitions()` |
| `CMAKE_CXX_FLAGS` | `target_compile_options()` |
| `link_directories()` | `target_link_directories()` (rarely needed) |
| `link_libraries()` | `target_link_libraries()` |

---

## Required Changes

### 1. Update Minimum CMake Version

**Current:**
```cmake
cmake_minimum_required(VERSION 3.5)
```

**Modern:**
```cmake
cmake_minimum_required(VERSION 3.16)
```

**Rationale**: CMake 3.16 provides:
- Full `CMAKE_UNITY_BUILD` support
- Improved `target_precompile_headers()`
- Better generator expression support
- Available on Ubuntu 20.04+

**Files affected (14 files):**
- All CMakeLists.txt files in `veranda/Packages/`

---

### 2. Fix Project Declaration Order

**Current (Anti-pattern):**
```cmake
set(PROJECT_NAME veranda_core_api)
project(${PROJECT_NAME})
```

**Modern:**
```cmake
project(veranda_core_api
    VERSION 0.1.0
    LANGUAGES CXX
)
```

**Rationale**:
- `PROJECT_NAME` is set by `project()` automatically
- Setting it before `project()` is confusing and non-idiomatic
- Adding VERSION enables `PROJECT_VERSION` variable
- Explicit LANGUAGES improves configuration speed

**Files affected:**
- `pkg-veranda_core_api/CMakeLists.txt`
- `pkg-veranda_core/CMakeLists.txt`
- `pkg-veranda_builtin_sensors/CMakeLists.txt`
- `pkg-veranda_builtin_wheels/CMakeLists.txt`
- `pkg-veranda_builtin_shapes/CMakeLists.txt`
- `pkg-veranda_qt_plugin_api/CMakeLists.txt`
- `pkg-veranda_qt_frontend_plugin_api/CMakeLists.txt`
- `pkg-veranda_qt_frontend/CMakeLists.txt`
- `pkg-veranda_box2d/CMakeLists.txt`
- `pkg-veranda_catch2/CMakeLists.txt`

---

### 3. Replace `include_directories()` with Target-Based Includes

**Current (Global pollution):**
```cmake
include_directories(include ${CMAKE_BINARY_DIR})
include_directories("${veranda_box2d_INCLUDE_DIRS}")
include_directories("${veranda_core_api_INCLUDE_DIRS}")
```

**Modern (Target-scoped):**
```cmake
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
)
```

**Scope guidelines:**
- `PUBLIC` - Headers needed by consumers of your library
- `PRIVATE` - Headers only needed for implementation
- `INTERFACE` - Headers only needed by consumers (header-only libs)

**Files affected (all library CMakeLists.txt):**
- `pkg-veranda_core_api/CMakeLists.txt` (3 include_directories calls)
- `pkg-veranda_core/CMakeLists.txt` (3 calls)
- `pkg-veranda_builtin_sensors/CMakeLists.txt` (4 calls)
- `pkg-veranda_builtin_wheels/CMakeLists.txt` (4 calls)
- `pkg-veranda_builtin_shapes/CMakeLists.txt` (4 calls)
- `pkg-veranda_qt_frontend/CMakeLists.txt` (4 calls)
- `pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt` (7 calls)
- `pkg-veranda_box2d/CMakeLists.txt` (1 call)

---

### 4. Replace `add_definitions()` with Target-Based Definitions

**Current (Global pollution):**
```cmake
add_definitions(-DVERANDA_CORE_API_BUILD_DLL)
add_definitions(-DQT_PLUGIN)
add_definitions(-DQT_SHARED)
```

**Modern (Target-scoped):**
```cmake
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        VERANDA_CORE_API_BUILD_DLL
        QT_PLUGIN
        QT_SHARED
)
```

**Files affected:**
| File | Definitions |
|------|-------------|
| `pkg-veranda_core_api/CMakeLists.txt` | `VERANDA_CORE_API_BUILD_DLL` |
| `pkg-veranda_core/CMakeLists.txt` | `VERANDA_CORE_IMPL_BUILD_DLL` |
| `pkg-veranda_builtin_sensors/CMakeLists.txt` | `VERANDA_SENSORS_BUILD_DLL` |
| `pkg-veranda_builtin_wheels/CMakeLists.txt` | `VERANDA_WHEELS_BUILD_DLL` |
| `pkg-veranda_builtin_shapes/CMakeLists.txt` | `VERANDA_SHAPES_BUILD_DLL`, `WINDOWS` |
| `pkg-veranda_box2d/CMakeLists.txt` | `BOX2D_DLL` |
| `pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt` | `QT_PLUGIN`, `QT_SHARED` |
| `pkg-veranda_qt_frontend_json_file_handler/CMakeLists.txt` | `QT_PLUGIN`, `QT_SHARED` |
| `pkg-veranda_qt_frontend_image_file_handler/CMakeLists.txt` | `QT_PLUGIN`, `QT_SHARED` |
| `pkg-veranda_qt_frontend_default_robot_file_handler/CMakeLists.txt` | `QT_PLUGIN`, `QT_SHARED` |

---

### 5. Replace `CMAKE_CXX_FLAGS` Modification

**Current (Global, platform-specific):**
```cmake
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()
```

**Modern (Target-scoped, portable):**
```cmake
# At top of file, after cmake_minimum_required
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# After target creation
target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
)
```

**Note**: Generator expressions allow portable warning flags across compilers.

**Files affected (12 files):**
- All files currently modifying `CMAKE_CXX_FLAGS`

---

### 6. Replace `qt5_use_modules()` (Deprecated in Qt 5.11)

**Current:**
```cmake
qt5_use_modules(${PROJECT_NAME} Core Gui Widgets)
```

**Modern:**
```cmake
target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
)
```

**Files affected (12 files):**
- `pkg-veranda_core_api/CMakeLists.txt`
- `pkg-veranda_core/CMakeLists.txt`
- `pkg-veranda_builtin_sensors/CMakeLists.txt`
- `pkg-veranda_builtin_wheels/CMakeLists.txt`
- `pkg-veranda_builtin_shapes/CMakeLists.txt`
- `pkg-veranda_qt_frontend/CMakeLists.txt`
- `pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt` (in function)
- `pkg-veranda_qt_frontend_json_file_handler/CMakeLists.txt`
- `pkg-veranda_qt_frontend_image_file_handler/CMakeLists.txt`
- `pkg-veranda_qt_frontend_default_robot_file_handler/CMakeLists.txt`
- `ament_cmake_add_catch_test.cmake`

---

### 7. Use AUTOMOC Instead of `qt5_wrap_cpp()`

**Current:**
```cmake
set(CMAKE_AUTOMOC ON)  # Already set, but then...

qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})  # Manually wrapping anyway
add_library(${PROJECT_NAME} SHARED ${CPP_SRCS} ${MOC_SRCS})
```

**Modern (Rely on AUTOMOC):**
```cmake
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

add_library(${PROJECT_NAME} SHARED
    ${CPP_SRCS}
    ${MOC_HDRS}  # Include headers directly, AUTOMOC handles them
)
```

**Rationale**: AUTOMOC scans source files and automatically runs moc on files containing Q_OBJECT. Manual `qt5_wrap_cpp()` is redundant when AUTOMOC is enabled.

**Files affected:**
- All files using `qt5_wrap_cpp()`

---

### 8. Modernize `ament_target_dependencies()`

**Current:**
```cmake
ament_target_dependencies(${PROJECT_NAME}
    "rclcpp"
    "std_msgs"
    "veranda_box2d"
    "veranda_core_api"
)
```

**Modern (Mixed approach - recommended for ROS 2):**
```cmake
# For ROS 2 packages that export modern CMake targets
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        rclcpp::rclcpp
    PRIVATE
        std_msgs::std_msgs__rosidl_typesupport_cpp
)

# For internal packages, use ament_target_dependencies
# until they export proper CMake targets
ament_target_dependencies(${PROJECT_NAME}
    veranda_box2d
    veranda_core_api
)
```

**Note**: `ament_target_dependencies()` is being deprecated in ROS 2 Kilted. However, for internal packages that don't export modern CMake targets, it's still useful. The recommended migration path is:

1. Update internal packages to export proper CMake targets
2. Gradually replace `ament_target_dependencies()` with `target_link_libraries()`

---

### 9. Export Targets Properly (for Libraries)

**Current:**
```cmake
ament_export_include_directories(${HEADER_INSTALL_DIR})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(veranda_box2d rclcpp)
```

**Modern:**
```cmake
# Create namespaced alias for use before installation
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Configure target properties for export
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# Install the target with export
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include/${PROJECT_NAME}
)

# Export targets for downstream packages
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp veranda_box2d)
```

**Rationale**: This allows downstream packages to use:
```cmake
find_package(veranda_core_api REQUIRED)
target_link_libraries(my_target PRIVATE veranda_core_api::veranda_core_api)
```

---

### 10. Use Generator Expressions for Conditional Logic

**Current:**
```cmake
if(WIN32)
    set(LIB_INSTALL_DIR bin)
else()
    set(LIB_INSTALL_DIR lib)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_DIR})
```

**Modern:**
```cmake
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
```

**Rationale**: CMake's `install(TARGETS)` with component destinations automatically handles the Windows vs Unix library location differences.

---

### 11. Consolidate Common CMake Logic

Create a shared CMake module for common patterns:

**New file: `cmake/VerandaCommon.cmake`**
```cmake
# Common settings for all Veranda packages

macro(veranda_common_setup)
    # C++ Standard
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Position Independent Code (for shared libraries)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)

    # Qt Auto-tools
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)

    # Include current binary dir for generated files
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endmacro()

macro(veranda_target_warnings target)
    target_compile_options(${target}
        PRIVATE
            $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic>
            $<$<CXX_COMPILER_ID:MSVC>:/W4>
    )
endmacro()

function(veranda_add_library target)
    # Parse arguments...
    # Common library setup...
endfunction()
```

**Usage in packages:**
```cmake
include(VerandaCommon)
veranda_common_setup()

add_library(${PROJECT_NAME} SHARED ${SOURCES})
veranda_target_warnings(${PROJECT_NAME})
```

---

## File-by-File Migration Checklist

### Core Packages

#### `pkg-veranda_core_api/CMakeLists.txt`
- [x] Update `cmake_minimum_required(VERSION 3.16)`
- [ ] Fix `project()` declaration (remove `set(PROJECT_NAME...)`)
- [x] Add `CMAKE_CXX_STANDARD_REQUIRED ON`
- [ ] Replace `include_directories()` → `target_include_directories()`
- [x] Replace `add_definitions()` → `target_compile_definitions()`
- [ ] Replace `CMAKE_CXX_FLAGS` → `target_compile_options()`
- [x] Keep `qt5_wrap_cpp()` for header-only QObjects (Model, Property, PropertyView)
- [x] Replace `qt5_use_modules()` → `target_link_libraries()`
- [ ] Update `install()` to export targets properly
- [ ] Add `ament_export_targets()`

#### `pkg-veranda_core/CMakeLists.txt`
- [x] Update `cmake_minimum_required(VERSION 3.16)`
- [ ] Fix `project()` declaration
- [x] Add `CMAKE_CXX_STANDARD_REQUIRED ON`
- [ ] Replace `include_directories()` → `target_include_directories()`
- [x] Replace `add_definitions()` → `target_compile_definitions()`
- [ ] Replace `CMAKE_CXX_FLAGS` → `target_compile_options()`
- [x] Replace `qt5_use_modules()` → `target_link_libraries()`
- [ ] Update `install()` to export targets properly

#### `pkg-veranda_box2d/CMakeLists.txt`
- [ ] Update `cmake_minimum_required(VERSION 3.16)`
- [ ] Fix `project()` declaration
- [ ] Replace `include_directories()` → `target_include_directories()`
- [ ] Replace `add_definitions()` → `target_compile_definitions()`
- [ ] Modernize `install()` commands

### Plugin Logic Packages

#### `pkg-veranda_builtin_sensors/CMakeLists.txt`
- [ ] All changes from core packages

#### `pkg-veranda_builtin_wheels/CMakeLists.txt`
- [ ] All changes from core packages

#### `pkg-veranda_builtin_shapes/CMakeLists.txt`
- [ ] All changes from core packages
- [ ] Remove `add_definitions(-DWINDOWS)` - use generator expression instead

### Qt Plugin Packages

#### `pkg-veranda_builtin_plugin_qt_wrappers/CMakeLists.txt`
- [ ] Update minimum version
- [ ] Modernize `make_plugin()` function to use target-based commands
- [ ] Replace `qt5_wrap_cpp()` and `qt5_use_modules()` in function
- [ ] Use `target_compile_definitions()` for `QT_PLUGIN`, `QT_SHARED`

### Frontend Packages

#### `pkg-veranda_qt_frontend/CMakeLists.txt`
- [ ] All standard changes
- [ ] Replace `qt5_add_resources()` → `qt_add_resources()` or AUTORCC
- [ ] Replace `qt5_wrap_ui()` → AUTOUIC

### Test Infrastructure

#### `ament_cmake_add_catch_test.cmake`
- [x] Replace `qt5_use_modules()` → `target_link_libraries(Qt5::Core, etc.)`
- [x] Use modern `target_link_libraries()` with Qt5 imported targets
- [ ] Replace `qt5_wrap_cpp()` → use AUTOMOC flag (currently needed for explicit header MOC)

---

## Example: Complete Modern CMakeLists.txt

**Before (Legacy):**
```cmake
cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME veranda_core_api)
set(HEADER_INSTALL_DIR include/${PROJECT_NAME})
set(LIB_INSTALL_DIR lib)
if(WIN32)
    set(LIB_INSTALL_DIR bin)
endif()

project(${PROJECT_NAME})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 11)
if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

find_package(ament_cmake REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(rclcpp REQUIRED)

set(QT_COMPONENTS Core Gui Widgets)
find_package(Qt5 REQUIRED COMPONENTS ${QT_COMPONENTS})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories(include ${CMAKE_BINARY_DIR})

ament_export_dependencies(veranda_box2d rclcpp)
ament_export_include_directories(${HEADER_INSTALL_DIR})
ament_export_libraries(${PROJECT_NAME})

include_directories("${veranda_box2d_INCLUDE_DIRS}")

qt5_wrap_cpp(MOC_SRCS ${MOC_HDRS})

add_definitions(-DVERANDA_CORE_API_BUILD_DLL)
add_library(${PROJECT_NAME} SHARED ${CPP_SRCS} ${MOC_SRCS})

qt5_use_modules(${PROJECT_NAME} ${QT_COMPONENTS})

ament_target_dependencies(${PROJECT_NAME} "veranda_box2d" "rclcpp")

install(DIRECTORY include/veranda_core DESTINATION ${HEADER_INSTALL_DIR})
install(TARGETS ${PROJECT_NAME} DESTINATION ${LIB_INSTALL_DIR})

ament_package()
```

**After (Modern):**
```cmake
cmake_minimum_required(VERSION 3.16)

project(veranda_core_api
    VERSION 0.1.0
    LANGUAGES CXX
)

# ============== Settings ==============
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ============== Dependencies ==============
find_package(ament_cmake REQUIRED)
find_package(veranda_box2d REQUIRED)
find_package(rclcpp REQUIRED)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Gui Widgets)

# ============== Sources ==============
set(HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/veranda_core)

set(PUBLIC_HEADERS
    ${HEADERS_DIR}/api/world_object.h
    ${HEADERS_DIR}/api/model.h
    ${HEADERS_DIR}/api/property.h
    ${HEADERS_DIR}/api/world_object_component.h
    ${HEADERS_DIR}/api/filter.h
    ${HEADERS_DIR}/api/interfaces/simulator_ui_if.h
    ${HEADERS_DIR}/api/interfaces/simulator_physics_if.h
    ${HEADERS_DIR}/api/interfaces/world_object_wrappers.h
)

set(SOURCES
    src/world_object.cpp
    src/world_object_component.cpp
    src/filter.cpp
)

# ============== Target ==============
add_library(${PROJECT_NAME} SHARED
    ${SOURCES}
    ${PUBLIC_HEADERS}  # AUTOMOC will process these
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

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
)

target_compile_options(${PROJECT_NAME}
    PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra>
        $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Qt${QT_VERSION_MAJOR}::Core
        Qt${QT_VERSION_MAJOR}::Gui
        Qt${QT_VERSION_MAJOR}::Widgets
    PRIVATE
        rclcpp::rclcpp
)

ament_target_dependencies(${PROJECT_NAME}
    veranda_box2d
)

# ============== Install ==============
install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# ============== Export ==============
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
    veranda_box2d
    rclcpp
    Qt${QT_VERSION_MAJOR}
)

ament_package()
```

---

## Implementation Phases

| Phase | Description |
|-------|-------------|
| 1. Create shared CMake module | `cmake/VerandaCommon.cmake` |
| 2. Update core packages | `core_api`, `core`, `box2d` |
| 3. Update plugin logic packages | `sensors`, `wheels`, `shapes` |
| 4. Update Qt frontend packages | All frontend packages |
| 5. Update Qt plugin packages | Plugin wrappers, APIs |
| 6. Update test infrastructure | `catch2`, test cmake |
| 7. Testing and debugging | Full build verification |

---

## Benefits of Modernization

1. **Better dependency tracking** - CMake understands the full dependency graph
2. **Faster builds** - Proper dependency propagation avoids unnecessary rebuilds
3. **Easier maintenance** - Target-based approach is self-documenting
4. **Portable** - Generator expressions handle platform differences
5. **Qt 6 ready** - Modern patterns are compatible with Qt 6 CMake integration
6. **IDE support** - Better integration with modern IDEs
7. **Export-ready** - Proper targets can be easily consumed by downstream projects

---

## References

- [Effective Modern CMake](https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1)
- [An Introduction to Modern CMake](https://cliutils.gitlab.io/modern-cmake/)
- [CMake 3.28 Release Notes](https://cmake.org/cmake/help/latest/release/3.28.html)
- [ament_cmake User Documentation](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [Deprecating ament_target_dependencies](https://github.com/ament/ament_cmake/pull/572)
- [CMake target_include_directories](https://cmake.org/cmake/help/latest/command/target_include_directories.html)
- [CMake Generator Expressions](https://cmake.org/cmake/help/latest/manual/cmake-generator-expressions.7.html)
